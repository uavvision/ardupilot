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
  simple plane simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>

namespace SITL {

/*
  a very simple plane simulator
 */
class Plane : public Aircraft {
public:
    Plane(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Plane(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } coefficient;

    float thrust_scale;

    /*
      Модель гвинта. Історично SIM_Plane множить тягу на throttle лінійно, без
      залежності від швидкості, тобто статична тяга діє і на 30 м/с. Для
      реального гвинта це неправда, тому вмикається класичне наближення диска:

          T(V, throttle) = thrust_scale * throttle^e * clamp(1 - V/V_pitch, 0, 1)

      `prop_v_pitch` = 0 вимикає поправку (поведінка всіх старих фреймів).
      V_pitch задається як крокова швидкість гвинта (крок * оберти/60).
      `throttle_exponent` типово 1.0 (лінійно, як історично); для реального
      борта він 2.0, бо ESC дає оберти ~ throttle, а тяга ~ обертів².
    */
    float prop_v_pitch;
    float throttle_exponent;

    /*
      Пасивний опір гвинта («вітрянка») на малому газі, Н на 1 м/с.
      Реальний борт на холостих має L/D ≈ 5.4 (замір: 28.2 м/с, зниження
      5.2 м/с, pitch −10.7°), а без цього доданка SIM_Plane виходить планером
      L/D ≈ 12 і в пікіруванні з нульовим газом розганяється до 39 м/с
      (замість ~28 у борта). Діє лінійно від `throttle` = 0.25 до 0.
      `0` вимикає (усі старі фрейми).
    */
    float prop_windmill_drag;

    /*
      Інерції обертання, кг·м². `getTorque()` повертає РОЗМІРНИЙ момент (Н·м),
      а `update_dynamics()` приймає кутове прискорення — тобто історично
      SIM_Plane неявно вважає I = 1 для всіх осей. Через це маса апарата
      взагалі не впливає на швидкість реакції по крену/тангажу. Щоб 30-кг
      модель мала реалістичну динаміку, момент ділиться на ці інерції.
      Типово 1.0 — усі попередні фрейми поводяться точно як раніше.
    */
    float inertia_roll;
    float inertia_pitch;
    float inertia_yaw;

    bool reverse_thrust;
    bool elevons;
    bool vtail;
    bool dspoilers;
    bool reverse_elevator_rudder;
    bool ice_engine;
    bool tailsitter;
    bool aerobatic;
    bool copter_tailsitter;
    /*
      Катапульта/банджі. Механіка (як в апстрімі): поки `input.servos[6]` > 1700
      (вихід каналу 7), до тіла прикладається `mass * launch_accel` уперед і
      `mass * launch_accel_z` по осі z (у FRD z — ВНИЗ, тому додатне значення
      штовхає в землю). `launch_accel_z` винесено окремо, щоб 30-кілограмовий
      апарат міг мати чисто горизонтальну рейку, не змінюючи поведінку старих
      фреймів (у них там історично `launch_accel/3`).
    */
    bool have_launcher;
    float launch_accel;
    float launch_accel_z;
    float launch_time;
    uint64_t launch_start_ms;

    /*
      «Стоїть на рейці» — стан ДО пострілу. Поки `launcher_fired == false`,
      апарат тримається нерухомо: `launcher_hold_height` метрів над землею з
      тангажем `launcher_pitch`. Це прибирає дві проблеми, які коштували
      кількох ітерацій:
        * земляний код SITL (`on_ground()`) на землі глушить кутову динаміку,
          тому апарат не міг «взяти ніс» і відірватися;
        * на землі діє тертя `0.3*V`, яке з'їдало швидкість рейки.
      `launcher_hold_height == 0` вимикає утримання (усі старі фрейми).
    */
    bool launcher_fired;
    float launcher_hold_height;
    float launcher_pitch;
    float launcher_yaw;

    const uint8_t throttle_servo = 2;
    const int8_t choke_servo = 14;
    const int8_t ignition_servo = 12;
    const int8_t starter_servo = 13;
    const float slewrate = 100;
    ICEngine icengine{
        throttle_servo,
        choke_servo,
        ignition_servo,
        starter_servo,
        slewrate,
        true
    };

    /* зафіксувати апарат на рейці: висота, тангаж, нульова швидкість */
    void hold_on_launcher();

    /*
      Рух по рейці після пострілу. На відміну від старого «просто додати
      mass*launch_accel до сили», тут апарат ЖОРСТКО ведеться вздовж рейки:
      тангаж/курс тримаються, швидкість = launch_accel*t, вертикальна
      складова = V*sin(launcher_pitch). Гравітація не «з'їдає» ніс, тому
      на виході апарат має реальні +5…7 м/с вертикальної швидкості, як
      справжня катапульта, а не провалюється на 0.2–0.3 м за постріл.
    */
    void update_on_rail(const struct sitl_input &input);

    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);
};

} // namespace SITL
