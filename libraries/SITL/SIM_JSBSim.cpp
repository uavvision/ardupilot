/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for JSBSim
*/

#include "SIM_JSBSim.h"

#if HAL_SIM_JSBSIM_ENABLED

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 1

/*
  Стеля частоти JSBSim.

  SITL типово має SIM_RATE_HZ = 1200, і `SIM_JSBSim` передає це JSBSim як
  `--simulation-rate`. Для SIM_Plane 1200 Гц — це просто інтегратор, а для
  JSBSim кожен крок ще й відправляє FGNetFDM на UDP, тобто 1200 пакетів за
  секунду на один прогін. Заміряно: SITL-процес з JSBSim на 1200 Гц їсть
  ~8 % CPU лише на FDM, тоді як вхід продюсера йде кроками ~10 Гц, а
  Lua-скрипт переплановується на 30 Гц (AGENTS §7.9).

  400 Гц дає 2.5 мс на крок — з запасом для 30 Гц скрипта і для 10 Гц
  продюсера. Параметр SIM_RATE_HZ на це НЕ впливає: він застосовується вже
  після того, як дочірній процес JSBSim запущено з початковим rate_hz.
*/
static constexpr float JSBSIM_MAX_RATE_HZ = 400.0f;

/*
  Фактична частота FDM для JSBSim.
 */
static float jsbsim_rate_hz(float rate_hz)
{
    return rate_hz > JSBSIM_MAX_RATE_HZ ? JSBSIM_MAX_RATE_HZ : rate_hz;
}

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    initialised(false),
    jsbsim_script(nullptr),
    jsbsim_fgout(nullptr),
    ground_disabled(false),
    created_templates(false),
    started_jsbsim(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    /*
      Frame string: "jsbsim:<model>[:<variant>]".

      Друга двокрапка — це ВАРІАНТ СТАРТОВОГО СТАНУ, а не частина назви
      моделі: `jsbsim:canard30:air` означає модель `canard30` і шаблон
      `aircraft/canard30/reset_template_air.xml`. Потрібно тому, що
      канард має два принципово різні старти (ЗПС і повітря), і вони не
      можуть жити в одному reset_template.xml: повітряний старт без
      GPS-швидкості ламає EKF (див. reset_template.xml у теці моделі).
    */
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        const char *spec = model_name + 1;
        const char *variant = strchr(spec, ':');
        if (variant == nullptr) {
            jsbsim_model = spec;
        } else {
            jsbsim_model.assign(spec, variant - spec);
            jsbsim_variant = variant + 1;
        }
    }
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;

    printf("JSBSim backend started: control_port=%u fdm_port=%u model='%s'\n",
           control_port, fdm_port, jsbsim_model.c_str());
    printf("JSBSim_model: '%s' variant: '%s'\n",
           jsbsim_model.c_str(),
           jsbsim_variant.empty() ? "(none)" : jsbsim_variant.c_str());
}

/*
  Замінює всі входження `what` на `with` у `s`.
 */
static void replace_all(std::string &s, const std::string &what, const std::string &with)
{
    size_t pos = 0;
    while ((pos = s.find(what, pos)) != std::string::npos) {
        s.replace(pos, what.size(), with);
        pos += with.size();
    }
}

/*
  Пише aircraft/<model>/reset.xml із aircraft/<model>/reset_template.xml.

  Навіщо: стандартний генератор у create_templates() зашиває
  alt = 1.3 м, vt = 0, theta = 13°, тобто старт лише з землі. Харнесу
  наведення потрібен детермінований старт у повітрі (висота/швидкість/
  тангаж), інакше кожен прогін починається з довгого набору висоти й
  залежить від зльоту. Шаблон дає моделі самій визначити ці числа, а
  lat/lon/psi підставляються з home, як і раніше.

  Повертає false, якщо шаблон не читається або в ньому немає жодного
  плейсхолдера — тоді виклик відкочується на старий жорсткий reset.
 */
static bool write_reset_from_template(const char *template_path, const char *out_path,
                                      double lat_deg, double lon_deg, double psi_deg)
{
    FILE *in = fopen(template_path, "r");
    if (in == nullptr) {
        return false;
    }
    std::string out;
    char chunk[1024];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), in)) > 0) {
        out.append(chunk, n);
    }
    fclose(in);
    if (out.empty()) {
        return false;
    }

    char lat[32], lon[32], psi[32];
    snprintf(lat, sizeof(lat), "%.9f", lat_deg);
    snprintf(lon, sizeof(lon), "%.9f", lon_deg);
    snprintf(psi, sizeof(psi), "%.3f", psi_deg);

    const bool had_tokens = out.find("%LAT%") != std::string::npos ||
                            out.find("%LON%") != std::string::npos ||
                            out.find("%PSI_DEG%") != std::string::npos;
    if (!had_tokens) {
        return false;
    }
    replace_all(out, "%LAT%", lat);
    replace_all(out, "%LON%", lon);
    replace_all(out, "%PSI_DEG%", psi);

    FILE *f = fopen(out_path, "w");
    if (f == nullptr) {
        return false;
    }
    const bool ok = fwrite(out.data(), 1, out.size(), f) == out.size();
    fclose(f);
    return ok;
}

/*
  create template files
 */
bool JSBSim::create_templates(void)
{
    if (created_templates) {
        return true;
    }

    asprintf(&jsbsim_script, "%s/jsbsim_start_%u.xml", autotest_dir, instance);
    asprintf(&jsbsim_fgout,  "%s/jsbsim_fgout_%u.xml", autotest_dir, instance);

    printf("JSBSim_script: '%s'\n", jsbsim_script);
    printf("JSBSim_fgout: '%s'\n", jsbsim_fgout);

    /*
      Якщо в теці моделі є reset_template.xml (або
      reset_template_<variant>.xml для фрейму з варіантом), стартовий стан
      беремо з нього (див. нижче), а trim-подію не додаємо: trim вимагає
      працюючої propulsion-моделі, а вона в нашому канарді замінена
      external_reactions. Без шаблону поведінка та сама, що завжди.
    */
    std::string jsbsim_reset_template = std::string(autotest_dir) +
        "/aircraft/" + jsbsim_model + "/reset_template";
    if (!jsbsim_variant.empty()) {
        jsbsim_reset_template += "_" + jsbsim_variant;
    }
    jsbsim_reset_template += ".xml";
    const bool use_reset_template = access(jsbsim_reset_template.c_str(), R_OK) == 0;
    printf("JSBSim_reset_template: '%s' %s\n",
           jsbsim_reset_template.c_str(),
           use_reset_template ? "(used)" : "(absent)");

    FILE *f = fopen(jsbsim_script, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim script %s", jsbsim_script);
    }
    fprintf(f,
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
"<?xml-stylesheet type=\"text/xsl\" href=\"http://jsbsim.sf.net/JSBSimScript.xsl\"?>\n"
"<runscript xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
"    xsi:noNamespaceSchemaLocation=\"http://jsbsim.sf.net/JSBSimScript.xsd\"\n"
"    name=\"Testing %s\">\n"
"\n"
"  <description>\n"
"    test ArduPlane using %s and JSBSim\n"
"  </description>\n"
"\n"
"  <use aircraft=\"%s\" initialize=\"reset\"/>\n"
"\n"
"  <!-- we control the servos via the jsbsim console\n"
"       interface on TCP 5124 -->\n"
"  <input port=\"%u\"/>\n"
"\n"
"  <run start=\"0\" end=\"10000000\" dt=\"%.6f\">\n"
"    <property value=\"0\"> simulation/notify-time-trigger </property>\n"
"\n"
"    <event name=\"start engine\">\n"
"      <condition> simulation/sim-time-sec le 0.01 </condition>\n"
"      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n",
            jsbsim_model.c_str(),
            jsbsim_model.c_str(),
            jsbsim_model.c_str(),
            control_port,
            1.0/jsbsim_rate_hz(rate_hz));
    if (!use_reset_template) {
        fprintf(f,
"\n"
"    <event name=\"Trim\">\n"
"      <condition>simulation/sim-time-sec ge 0.01</condition>\n"
"      <set name=\"simulation/do_simple_trim\" value=\"2\"/>\n"
"      <notify/>\n"
"    </event>\n");
    }
    fprintf(f,
"  </run>\n"
"\n"
"</runscript>\n");
    fclose(f);

    f = fopen(jsbsim_fgout, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim fgout script %s", jsbsim_fgout);
    }
    fprintf(f, "<?xml version=\"1.0\"?>\n"
            "<output name=\"127.0.0.1\" type=\"FLIGHTGEAR\" port=\"%u\" protocol=\"UDP\" rate=\"%f\">\n"
            "  <time type=\"simulation\" resolution=\"1e-6\"/>\n"
            "</output>",
            fdm_port, jsbsim_rate_hz(rate_hz));
    fclose(f);

    const std::string jsbsim_reset = std::string(autotest_dir) +
        "/aircraft/" + jsbsim_model + "/reset.xml";

    printf("JSBSim_reset: '%s'\n", jsbsim_reset.c_str());

    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    if (!use_reset_template ||
        !write_reset_from_template(jsbsim_reset_template.c_str(), jsbsim_reset.c_str(),
                                   home.lat*1.0e-7, home.lng*1.0e-7,
                                   degrees(y))) {
        f = fopen(jsbsim_reset.c_str(), "w");
        if (f == nullptr) {
            AP_HAL::panic("Unable to create jsbsim reset script %s", jsbsim_reset.c_str());
        }
        fprintf(f,
                "<?xml version=\"1.0\"?>\n"
                "<initialize name=\"Start up location\">\n"
                "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
                "  <longitude unit=\"DEG\"> %f </longitude>\n"
                "  <altitude unit=\"M\"> 1.3 </altitude>\n"
                "  <vt unit=\"FT/SEC\"> 0.0 </vt>\n"
                "  <gamma unit=\"DEG\"> 0.0 </gamma>\n"
                "  <phi unit=\"DEG\"> 0.0 </phi>\n"
                "  <theta unit=\"DEG\"> 13.0 </theta>\n"
                "  <psi unit=\"DEG\"> %f </psi>\n"
                "</initialize>\n",
                home.lat*1.0e-7,
                home.lng*1.0e-7,
                degrees(y));
        fclose(f);
    }
    created_templates = true;
    return true;
}


/*
  start JSBSim child
 */
bool JSBSim::start_JSBSim(void)
{
    if (started_jsbsim) {
        return true;
    }
    if (!open_fdm_socket()) {
        return false;
    }

    int p[2];
    int devnull = open("/dev/null", O_RDWR|O_CLOEXEC);
    if (pipe(p) != 0) {
        AP_HAL::panic("Unable to create pipe");
    }
    pid_t child_pid = fork();
    if (child_pid == 0) {
        // in child
        setsid();
        dup2(devnull, 0);
        dup2(p[1], 1);
        close(p[0]);
        for (uint8_t i=3; i<100; i++) {
            close(i);
        }
        char *logdirective;
        char *script;
        char *nice;
        char *rate;

        asprintf(&logdirective, "--logdirectivefile=%s", jsbsim_fgout);
        asprintf(&script, "--script=%s", jsbsim_script);
        asprintf(&nice, "--nice=%.8f", 10*1e-9);
        asprintf(&rate, "--simulation-rate=%f", jsbsim_rate_hz(rate_hz));

        if (chdir(autotest_dir) != 0) {
            perror(autotest_dir);
            exit(1);
        }

        int ret = execlp("JSBSim",
                         "JSBSim",
                         "--suspend",
                         rate,
                         nice,
                         logdirective,
                         script,
                         nullptr);
        if (ret != 0) {
            perror("JSBSim");
        }
        exit(1);
    }
    close(p[1]);
    jsbsim_stdout = p[0];

    // read startup to be sure it is running
    char c;
    if (read(jsbsim_stdout, &c, 1) != 1) {
        AP_HAL::panic("Unable to start JSBSim");
    }

    if (!expect("JSBSim Execution beginning")) {
        AP_HAL::panic("Failed to start JSBSim");
    }
    if (!open_control_socket()) {
        AP_HAL::panic("Failed to open JSBSim control socket");
    }

    fcntl(jsbsim_stdout, F_SETFL, fcntl(jsbsim_stdout, F_GETFL, 0) | O_NONBLOCK);

    started_jsbsim = true;
    check_stdout();
    close(devnull);
    return true;
}

/*
  check for stdout from JSBSim
 */
void JSBSim::check_stdout(void) const
{
    char line[100];
    ssize_t ret = ::read(jsbsim_stdout, line, sizeof(line));
    if (ret > 0) {
#if DEBUG_JSBSIM
        write(1, line, ret);
#endif
    }
}

/*
  a simple function to wait for a string on jsbsim_stdout
 */
bool JSBSim::expect(const char *str) const
{
    const char *basestr = str;
    while (*str) {
        char c;
        if (read(jsbsim_stdout, &c, 1) != 1) {
            return false;
        }
        if (c == *str) {
            str++;
        } else {
            str = basestr;
        }
#if DEBUG_JSBSIM
        write(1, &c, 1);
#endif
    }
    return true;
}

/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        check_stdout();
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}


/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    if (frame == FRAME_ELEVON) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;
    } else if (frame == FRAME_VTAIL) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    }
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    asprintf(&buf,
             "set fcs/aileron-cmd-norm %f\n"
             "set fcs/elevator-cmd-norm %f\n"
             "set fcs/rudder-cmd-norm %f\n"
             "set fcs/throttle-cmd-norm %f\n"
             "set atmosphere/psiw-rad %f\n"
             "set atmosphere/wind-mag-fps %f\n"
             "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
             "set atmosphere/turbulence/milspec/severity %f\n"
             "iterate 1\n",
             aileron, elevator, rudder, throttle,
             radians(input.wind.direction),
             wind_speed_fps,
             wind_speed_fps/3,
             input.wind.turbulence);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    check_stdout();

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
            check_stdout();
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location.lat = RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7;
    location.lng = RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7;
    location.alt = fdm.agl*100 + home.alt;
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];
    
    time_now_us = fdm.cur_time;
}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
/*
  Після відриву земля більше не «вбиває» апарат — так само, як у SIM_Plane,
  де `Aircraft::on_ground()` фіксує `has_taken_off` при hagl() > 0.2 м і далі
  завжди повертає false (AGENTS §7.10). Різниця в тому, що в JSBSim земля
  живе всередині FDM, тому її треба вимкнути явно.

  Механізм — штатна **ретракція шасі**: у `FGLGear::CalcForce()` увесь
  розрахунок контакту виконується лише при `gearPos > 0.99` («шасі
  випущене»). Тому всі контакти в моделі позначені
  `<retractable>1</retractable>`, а тут ми «прибираємо шасі»: `gear-pos-norm 0`
  вимикає всі опори одразу, per-unit `pos-norm` — страховка, якщо FCS колись
  чіпатиме глобальний канал.

  Що для цього НЕ працює (перевірено офлайн, щоб не витрачати прогін):
  `maximum-force-lbs 0` — no-op (StrutForce завжди ≤ 0, а кламп порівнює з
  додатним максимумом); `z-position -10000` — апарат провалюється, але
  JSBSim падає з `Floating point exception`.

  Навіщо це в харнесі: під час заходу на ціль апарат інколи проходить нижче
  рівня смуги, і без цього JSBSim «ловить» його об землю (удар, стрибок
  кутових швидкостей), що псує і політ, і запис. Для тестів наведення земля
  під час польоту не потрібна.
 */
void JSBSim::disable_ground_after_takeoff(void)
{
    if (ground_disabled || hagl() <= 1.0f) {
        return;
    }
    ground_disabled = true;
    /*
      per-unit імена залежать від типу контакту (FGLGear::bind():
      `gear/unit[i]` для BOGEY, `contact/unit[i]` для STRUCTURE). У моделі
      3 BOGEY + 2 STRUCTURE = 5 контактів; для неіснуючих індексів консоль
      JSBSim лише попереджає.
    */
    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
                     "set gear/gear-pos-norm 0\n"
                     "set gear/unit[0]/pos-norm 0\n"
                     "set gear/unit[1]/pos-norm 0\n"
                     "set gear/unit[2]/pos-norm 0\n"
                     "set gear/unit[3]/pos-norm 0\n"
                     "set gear/unit[4]/pos-norm 0\n"
                     "set contact/unit[0]/pos-norm 0\n"
                     "set contact/unit[1]/pos-norm 0\n"
                     "set contact/unit[2]/pos-norm 0\n"
                     "set contact/unit[3]/pos-norm 0\n"
                     "set contact/unit[4]/pos-norm 0\n");
    if (n > 0 && n < (int)sizeof(buf)) {
        sock_control.send(buf, n);
    }
    printf("JSBSim: hagl=%.1f м — землю вимкнено на решту польоту\n", hagl());
}

void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!create_templates() ||
            !start_JSBSim() ||
            !open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    disable_ground_after_takeoff();
    // Крок FDM обмежений тією ж стелею, що й `--simulation-rate`: інакше
    // SITL рахує час як 1/1200 с на ітерацію, а пакетів приходить 400/с,
    // і симуляція йде втричі повільніше за реальний час.
    adjust_frame_time(jsbsim_rate_hz(rate_hz));
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
