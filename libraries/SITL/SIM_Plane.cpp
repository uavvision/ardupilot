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
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 2.0f;

    // типові значення: поправка гвинта вимкнена, момент не ділиться на інерцію
    prop_v_pitch = 0.0f;
    throttle_exponent = 1.0f;
    prop_windmill_drag = 0.0f;
    prop_windmill_throttle = 0.25f;
    prop_windmill_exponent = 1.0f;
    inertia_roll = 1.0f;
    inertia_pitch = 1.0f;
    inertia_yaw = 1.0f;
    launch_accel_z = 0.0f;
    // утримання на рейці вимкнене (0 = як у старих фреймів: одразу на землі)
    launcher_fired = false;
    launcher_hold_height = 0.0f;
    launcher_pitch = 0.0f;
    launcher_yaw = 0.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-canard30")) {
        /*
          30-кг канард зі штовхаючим гвинтом 23x13 (реальний борт).

          Стара калібровка (Vp=35.6, CD0=0.027) замикалася на двох точках
          «59 % -> 27.4 м/с» і «100 % -> 30.6 м/с». Друга точка виявилась не
          усталеною: у real/00000006.BIN на 100 % борт тримає 30.6 м/с лише
          коротко, а в сталому наборі на 28–30 м/с лізе 3–5 м/с. Через це
          модель мала замало тяги на 20–30 м/с (клімб ~2.9 м/с замість 4–7)
          і не могла розігнатися вище 30.6 м/с, хоча в логу є 33.5–37.6 м/с.
          Нова калібровка — по non-ACRO вікнах (FBWA/FBWB/TAKEOFF):
              * 59 % / 27.4 м/с — усталений крейсер (збережено);
              * 91 % / 27.8 м/с — набір 3.5–4.4 м/с;
              * 98 % / 33.6 м/с — набір 2.2–2.5 м/с;
              * 72 % / 27.9 м/с — усталений рівень.
          Фіт (T0=180 Н, CD0=0.027, e=0.9, маса 30 кг) дає Vp=45 м/с.
          Нижче 60 % газу гвинт гальмує (вітрянка), вище — тяга лінійно
          від 0 до повної:
              T(V, thr) = 180 * max(0,(thr-0.6)/0.4) * clamp(1-V/45, 0, 1)
          Перевірка: 70 %/27.9 -> 17 Н (крейсер потребує ~75 %);
          91 %/27.8 -> 53 Н -> клімб 2.7 м/с; 100 %/33.5 -> 46 Н -> 2.2 м/с.
          На спуску 95–105 км/г при 55 % газу тяга 0, а вітрянка ~8 Н
          (sqrt-спад) — саме та від'ємна тяга, яку показує real-лог
          (T_req ≈ −8 Н, зниження ~3 м/с).
          Аеродинамічні похідні (c_lift_*, c_l_deltaa, c_m_*, ...) не чіпаємо:
          вони задають реакцію на рулі, а не швидкості, і вже матчаться гейнами
          ArduPlane (tests/sitl/sitl.parm, блок 4).
        */
        mass = 30;
        /*
          КЛЮЧОВЕ для зльоту: знімаємо "прилипання" до землі. Типовий для
          SIM_Plane GROUND_BEHAVIOR_FWD_ONLY на землі примусово ставить тангаж
          `p = MAX(p, 0)` (тобто не дає опустити ніс) і, головне, робить
          `velocity_ef.z = 0` для будь-якого руху вниз — але, що важливіше,
          ФІКСУЄ кут атаки близько нуля, поки апарат на землі. Для 2-кг планера
          це байдуже (він злітає на 30 м/с), а 30-кг апарату треба підняти ніс,
          щоб узяти кут атаки — інакше при alpha≈0 підйомна сила досягає ваги
          лише на 28.6 м/с. Заміряно: горизонтальна рейка розганяла до 23.4 м/с,
          апарат так і лишався на землі й гальмувався тертям 0.3*V.
          GROUND_BEHAVIOR_NONE лишає лише площину землі (position.z
          затискається), але кутову динаміку не чіпає — тоді апарат нормально
          обертається кермом висоти й сходить з рейки, як справжній.
        */
        ground_behavior = GROUND_BEHAVIOR_NONE;
        // статична тяга 17-20 кгс на 100 %: номінал 18.4 кгс = 180 Н
        thrust_scale = 180;
        // ефективна крокова швидкість за non-ACRO набором; не 35.6, бо реальний
        // гвинт у польоті розкручується вище 6470 об/хв і не втрачає тягу так рано
        prop_v_pitch = 45.0f;
        // Нижче 60 % ESC тримає гвинт у режимі гальмування: у real-логу на
        // спуску 95–105 км/г стоїть 30–60 % газу, але T_req ≈ −8 Н, тобто
        // гвинт все ще гальмує. Тому тяга вмикається лише після 60 %, а
        // нижче працює вітрянка (див. prop_windmill_throttle).
        prop_windmill_throttle = 0.60f;
        // sqrt: при 55 % газу опір ще ~8 Н (реальний T_req ≈ −8 Н на
        // 95–105 км/г і зниженні ~3 м/с), а при 0 % — ті самі ~28 Н, що
        // дають заміряне L/D 5.4
        prop_windmill_exponent = 0.5f;
        throttle_exponent = 1.0f;
        // планер: S = 1.05 м², розмах 2.4 м (подовження 5.5), САХ 0.44 м
        coefficient.s = 1.05f;
        coefficient.b = 2.4f;
        coefficient.c = 0.44f;
        // паразитний опір планера з тих самих двох точок (на крейсері L/D ~11)
        coefficient.c_drag_p = 0.027f;
        /*
          Інерції. Перша оцінка була Ixx = 7.4 (з формули
          tau = 2*I*V/(q*S*b^2*|c_l_p|) = 0.0197*I), але compare_real.py на
          прогоні 10-04-09 заміряв tau = 272.6 мс проти 145.7 мс у борта,
          тобто рівно в 1.87 раза повільніше. Оскільки tau ∝ I,
          Ixx = 7.4/1.87 ≈ 4.0.
        */
        inertia_roll = 4.0f;
        inertia_pitch = 6.0f;
        inertia_yaw = 13.4f;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
        // вертикальна складова як в апстрімі (у FRD додатна = вниз)
        launch_accel_z = launch_accel / 3.0f;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
        launch_accel_z = launch_accel / 3.0f;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
        launch_accel_z = launch_accel / 3.0f;
    }

    if (have_launcher && strstr(frame_str, "-canard30")) {
        /*
          Катапульта під 30 кг — нахилена рейка (12°).

          ЧОМУ НЕ ГОРИЗОНТАЛЬНА. Два прогони дали однакову картину:
          * розбіг (23-11-29): швидкість застрягла на 12.1-12.2 м/с. Це рівно
            рівновага земляного тертя SITL: 180*(1-V/35.6)/30 = 0.3*V -> 12.8;
          * горизонтальна рейка (23-16-51): рейка розігнала до 23.4 м/с, але
            апарат так і не відірвався — висота 0.0-0.3 м, і за 6 с швидкість
            упала до 12.5 м/с при 100 % газу.
            Причина — не тяга, а геометрія: на землі (`on_ground()`) SITL тримає
            нульовий тангаж (`GROUND_BEHAVIOR_FWD_ONLY`), тобто кут атаки не
            зростає. На 23 м/с при alpha≈0 підйомна сила 0.5*1.225*23²*1.05*0.56
            = 190 Н проти ваги 294 Н -> апарат їде по землі, а земляне тертя
            0.3*V його гальмує. Щоб зійти з рейки «на нулі», потрібно
            28.6 м/с (V при CL(0)=0.56, що дає L=W), і аж тоді підйомна сила
            перевищить вагу без тангажу.

          РІШЕННЯ: «стояти на рейці» — 1 м над землею, ніс 15° вгору, нульова
          швидкість, поки не смикнули гачок (див. launcher_hold_* у заголовку).
          Через це земляний код SITL взагалі не задіюється: `on_ground()`
          дає false (hagl = 1 м), кутова динаміка вільна, тертя 0.3*V відсутнє.

          Рейка йде ВЗДОВЖ ОСІ ФЮЗЕЛЯЖА, а ніс уже піднятий на 15°, тому
          нахил рейки виходить сам собою: прискорення 28 м/с² вздовж 15° — це
          ~7.2 м/с² вгору і ~27 м/с² вперед, окремої vertical-складової
          (launch_accel_z) не потрібно. На виході (0.9 с) — близько 26-28 м/с
          під 15° угору, тобто відразу в наборі висоти, вище зриву (18.4 м/с).
        */
        launch_accel = 28;
        launch_time = 0.9f;
        launch_accel_z = 0.0f;
        launcher_hold_height = 1.0f;          // метр над землею
        launcher_pitch = radians(15.0f);      // ніс угору
        /*
          Опір гвинта, що вільно обертається, Н на (м/с)². Без нього модель
          планерує з L/D ≈ 12 і в заході на нульовому газу розганяється до
          39 м/с (141 км/г), тоді як реальний борт у тому ж режимі тримає
          ~28 м/с (замір по логу: 28.1 м/с, зниження 5.38 м/с → L/D 5.4).

          КВАДРАТИЧНИЙ, а не лінійний (Step 98). Лінійна 1.0·V давала на
          28 м/с майже правильні 28 Н, тому дефект довго не було видно, але
          на 43 м/с — лише 43 Н замість 74, і в пікіруванні 17–19° апарат
          тікав до 155 км/г. Замір по real/00000006.BIN (газ <= 5 %, сталі
          ділянки): повний опір 55.4 Н при 28.1 м/с, з них аеродинаміка
          24.3 Н, решта 31.1 Н — гвинт, тобто 0.040·V². Фізика та сама:
          D ∝ ρ·n²·D⁴, а оберти вільного гвинта n ∝ V.
        */
        prop_windmill_drag = 0.040f;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    float throttle;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    /*
      Поки апарат зафіксований на рейці, тяга НЕ діє: інакше акселерометр
      бачить thrust + гравітацію, і EKF, який ще вирівнюється, отримує хибну
      «вертикаль» (перевірено: тангаж 89°, крен ±125° ще до пострілу). На рейці
      тягу тримає сама катапульта, двигун працює «на холостих».
    */
    const bool on_launcher = have_launcher && !launcher_fired && launcher_hold_height > 0.0f;

    /*
      Для -canard30 гвинт до prop_windmill_throttle не тягне, а гальмує:
      тяга = 0, а опір вітрянки росте до 0 при досягненні порогу. Вище порогу
      ефективний газ лінійно росте від 0 до 1, тому при 70 % тяга ~25 % від
      повної — саме стільки потрібно для крейсеру 27–28 м/с.
    */
    float thrust = on_launcher ? 0.0f : throttle;
    if (!on_launcher && prop_windmill_throttle > 0.25f) {
        if (throttle <= prop_windmill_throttle) {
            thrust = 0.0f;
        } else {
            thrust = (throttle - prop_windmill_throttle) / (1.0f - prop_windmill_throttle);
        }
    }

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }
    
    Vector3f force = getForce(aileron, elevator, rudder);
    // getTorque() віддає момент у Н·м, а update_dynamics() чекає кутове
    // прискорення — тому ділимо на інерцію (типово 1, тобто як раніше)
    rot_accel = getTorque(aileron, elevator, rudder, thrust, force);
    rot_accel.x /= inertia_roll;
    rot_accel.y /= inertia_pitch;
    rot_accel.z /= inertia_yaw;

    if (have_launcher && launcher_hold_height <= 0.0f) {
        /*
          simple simulation of a launcher для старих фреймів.
          Для -canard30 (launcher_hold_height > 0) рейка реалізована
          окремо в update_on_rail(): там швидкість/тангаж задаються
          жорстко, а не додаванням сили, яку гравітація з'їдає.
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
                // запам'ятовуємо, що рейка вже відпрацювала: інакше після
                // відпускання гачка launch_start_ms скидається в 0 і апарат
                // знову «прилип» би до стартової позиції
                launcher_fired = true;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel_z;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }
    
    // simulate engine RPM (actual throttle, not the deadbanded thrust)
    motor_mask |= (1U<<2);
    rpm[2] = throttle * 7000;
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    if (fabsf(throttle_exponent - 1.0f) > 1.0e-6f) {
        // емпірична крива ESC: тяга ~ throttle^e (для -canard30 e=2)
        thrust *= powf(constrain_float(throttle, 0.0f, 1.0f), throttle_exponent - 1.0f);
    }

    if (prop_v_pitch > 0) {
        // гвинт втрачає тягу зі швидкістю: T = T0 * (1 - V/V_pitch)
        thrust *= constrain_float(1.0f - airspeed / prop_v_pitch, 0.0f, 1.0f);
    }

    if (prop_windmill_drag > 0 && throttle < prop_windmill_throttle) {
        /*
          Вітрянка: на малому газі гвинт гальмує, а не тягне. Спад від
          prop_windmill_throttle до нуля задає prop_windmill_exponent, а сама
          сила пропорційна V² (див. prop_windmill_drag) — саме цей доданок
          прибирає «планерний» розгін у пікіруванні.
        */
        float windmill = (prop_windmill_throttle - throttle) / prop_windmill_throttle;
        if (fabsf(prop_windmill_exponent - 1.0f) > 1.0e-6f) {
            windmill = powf(windmill, prop_windmill_exponent);
        }
        force.x -= prop_windmill_drag * airspeed * airspeed * windmill;
    }

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
/*
  Тримає апарат на рейці ДО пострілу. Ніякої фізики: ні аеродинаміки, ні тяги,
  ні гравітації, ні земляного коду — апарат просто стоїть там, де його поставили
  (у т.ч. на висоті launcher_hold_height), з тангажем launcher_pitch.

  Чому саме так: будь-яка «жива» динаміка на землі чи біля неї дає IMU
  прискорення/обертання, і EKF у цей момент або не вирівнюється, або «крутить»
  апарат (заміряно: тангаж 89°, крен ±125°, «висота» до 277 м, GPS не фіксується).
*/
void Plane::hold_on_launcher()
{
    // position.z — NED (додатне вниз). Формула дзеркальна до Aircraft::hagl():
    // hagl = -position.z + home.alt*0.01 - ground_level - frame_height - ghd
    position.z = -(ground_level + frame_height - home.alt * 0.01f
                   + ground_height_difference()) - launcher_hold_height;

    velocity_ef.zero();
    velocity_air_ef.zero();
    velocity_air_bf.zero();
    gyro.zero();

    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    launcher_yaw = y;
    dcm.from_euler(0.0f, launcher_pitch, launcher_yaw);   // крила рівно, ніс під заданим кутом

    // Апарат стоїть нерухомо, тому акселерометр мусить показувати рівно 1 g:
    // те саме, що update_dynamics() рахує для стану спокою (рядок 648).
    accel_body = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS);

    // «мертва» швидкість для всіх споживачів (приймач швидкості, логи, EKF)
    airspeed = 0.0f;
    airspeed_pitot = 0.0f;
}

void Plane::update_on_rail(const struct sitl_input &input)
{
    const uint64_t now = AP_HAL::millis64();
    const float elapsed = constrain_float((now - launch_start_ms) * 0.001f, 0.0f, launch_time);
    const float rail_speed = launch_accel * elapsed;

    // рейка тримає апарат: крила рівно, ніс під launcher_pitch, курс не змінюється
    dcm.from_euler(0.0f, launcher_pitch, launcher_yaw);
    gyro.zero();

    // швидкість строго вздовж рейки (вісь +x тіла), а не «тяга мінус гравітація»
    velocity_ef = dcm * Vector3f(rail_speed, 0.0f, 0.0f);

    // акселерометр бачить прискорення рейки + 1 g (це те, що бачить реальний IMU)
    const Vector3f rail_accel_ef = dcm * Vector3f(launch_accel, 0.0f, 0.0f);
    accel_body = dcm.transposed() * (rail_accel_ef + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // повітряні величини — з тією ж конвенцією, що в update_dynamics()
    update_wind(input);
    velocity_air_ef = velocity_ef - wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    airspeed = velocity_air_ef.length();
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);

    // тяга та RPM для логів/ESC (фізику рейки не змінюють)
    const float throttle = filtered_servo_range(input, throttle_servo);
    battery_voltage = sitl->batt_voltage - 0.7f * throttle;
    battery_current = (battery_voltage / sitl->batt_voltage) * 50.0f * sq(throttle);
    motor_mask |= (1U << throttle_servo);
    rpm[throttle_servo] = throttle * 7000.0f;

    // інтегруємо позицію вздовж рейки
    const float dt = frame_time_us * 1.0e-6f;
    position += (velocity_ef * dt).todouble();
    update_position();
    time_advance();
    update_mag_field_bf();
}

void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    if (have_launcher && !launcher_fired && launcher_hold_height > 0.0f) {
        if (input.servos[6] > 1700) {
            // гачок смикнули — стартуємо рух по рейці
            launcher_fired = true;
            launch_start_ms = AP_HAL::millis64();
            update_on_rail(input);
            return;
        }
        // ПОВНІСТЮ без фізики: стоїмо на рейці там, де нас поставили
        hold_on_launcher();
        update_position();
        time_advance();
        update_mag_field_bf();
        return;
    }

    // перші launch_time секунд після пострілу — апарат ще на рейці:
    // тангаж/курс/напрямок швидкості задає катапульта, а не аеродинаміка
    if (have_launcher && launcher_fired && launch_start_ms != 0 &&
        (AP_HAL::millis64() - launch_start_ms) < uint64_t(launch_time * 1000.0f)) {
        update_on_rail(input);
        return;
    }

    update_wind(input);
    
    calculate_forces(input, rot_accel);
    
    update_dynamics(rot_accel);

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
