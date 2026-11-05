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
    prop_windmill_drag = 0.0f;
    inertia_roll = 1.0f;
    inertia_pitch = 1.0f;
    inertia_yaw = 1.0f;
    launch_accel_z = 0.0f;
    // утримання на рейці вимкнене (0 = як у старих фреймів: одразу на землі)
    launcher_fired = false;
    launcher_hold_height = 0.0f;
    launcher_pitch = 0.0f;

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
          Параметри зняті з логу real/00000006.BIN і з двох його усталених
          точок: крейсер 27.4 м/с при 59 % газу і 30.6 м/с (110 км/г) на 100 %.
          Дві невідомі (V_pitch гвинта і CD0 планера) розв'язані саме на цих
          двох точках, тому модель відтворює ОБИДВІ, а не одну:
              T0*(1 - 30.6/Vp) = q(30.6)*S*(CD0 + CDi)
              0.59*T0*(1 - 27.4/Vp) = q(27.4)*S*(CD0 + CDi)
          -> Vp = 35.6 м/с, CD0 = 0.027.
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
        // крок 13" = 0.3302 м; повний газ ~6470 об/хв -> крокова швидкість 35.6 м/с
        prop_v_pitch = 35.6f;
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
          Опір гвинта, що вільно обертається. Без нього модель планерує з
          L/D ≈ 12 і в заході на нульовому газу розганяється до 39 м/с
          (141 км/г), тоді як реальний борт у тому ж режимі тримає ~28 м/с
          (замір по логу: 28.2 м/с, зниження 5.2 м/с → L/D 5.4).
          На 28 м/с доданок дає 28 Н, що разом з аеродинамічним опором
          (24.6 Н) якраз урівноважує складову ваги 54 Н.
        */
        prop_windmill_drag = 1.0f;
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

    float thrust     = on_launcher ? 0.0f : throttle;

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

    if (have_launcher) {
        /*
          simple simulation of a launcher
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
    
    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    if (prop_v_pitch > 0) {
        // гвинт втрачає тягу зі швидкістю: T = T0 * (1 - V/V_pitch)
        thrust *= constrain_float(1.0f - airspeed / prop_v_pitch, 0.0f, 1.0f);
    }

    if (prop_windmill_drag > 0 && throttle < 0.25f) {
        /*
          Вітрянка: на холостих гвинт гальмує, а не тягне. Лінійно від
          25 % газу до нуля — саме цей доданок прибирає «планерний» розгін
          у пікіруванні (див. проп_windmill_drag у заголовку).
        */
        const float windmill = (0.25f - throttle) / 0.25f;
        force.x -= prop_windmill_drag * airspeed * windmill;
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
    dcm.from_euler(0.0f, launcher_pitch, y);   // крила рівно, ніс під заданим кутом

    // Апарат стоїть нерухомо, тому акселерометр мусить показувати рівно 1 g:
    // те саме, що update_dynamics() рахує для стану спокою (рядок 648).
    accel_body = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS);

    // «мертва» швидкість для всіх споживачів (приймач швидкості, логи, EKF)
    airspeed = 0.0f;
    airspeed_pitot = 0.0f;
}

void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    if (have_launcher && !launcher_fired && launcher_hold_height > 0.0f) {
        if (input.servos[6] > 1700) {
            // гачок смикнули — цей такт уже йде звичайна фізика
            launcher_fired = true;
        } else {
            // ПОВНІСТЮ без фізики: стоїмо на рейці там, де нас поставили
            hold_on_launcher();
            update_position();
            time_advance();
            update_mag_field_bf();
            return;
        }
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
