#ifndef VEHICLE_H
#define VEHICLE_H
/*
 * I'll break this down into separate files eventually
 * at some point
 * i promise
 */

#include <scene/3d/physics_body.h>
#include <scene/resources/curve.h>
#include <scene/audio/audio_stream_player.h>

class mf_vehicle_wheel;

class mf_vehicle_body:public RigidBody {
    GDCLASS(mf_vehicle_body, RigidBody);

    public:
        mf_vehicle_body(void);
        ~mf_vehicle_body(void);

        enum DIFF_TYPE {
            LIMITED_SLIP,
            OPEN_DIFF,
            LOCKED
        };

        enum DRIVE_TYPE {
            FWD,
            RWD,
            AWD
        };

        void process(real_t delta);
        void physics_process(real_t delta, Dictionary surface);

        real_t engine_torque(real_t r_p_m);
        void freewheel(real_t delta);
        void engage(real_t delta);
        real_t gear_ratio(void);
        void rear_wheel_drive(real_t drive, real_t delta);
        void front_wheel_drive(real_t drive, real_t delta);
        void all_wheel_drive(real_t drive, real_t delta);
        void drag_force(void);
        void burn_fuel(real_t delta);
        void shift_up(void);
        void shift_down(void);
        void play_engine_sound(void);
        void stop_engine_sound(void);

        // wheels
        Dictionary wheel_list = Dictionary();
        Array wheel_data = Array();
        //Vector<ng_vehicle_wheel *> wheels;

    protected:
        static void _bind_methods(void);

    private:
        real_t max_steer = real_t(0.3);
        real_t front_brake_bias = real_t(0.6); // range(0, 1)
        real_t steer_speed = real_t(5);
        real_t max_brake_force = real_t(500);
        real_t fuel_tank_size = real_t(40); // litres
        real_t fuel_percentage = real_t(100); // percent of full tank

        // Engine variables
        real_t max_torque = real_t(250);
        real_t max_engine_rpm = real_t(8000);
        real_t rpm_clutch_out = real_t(1500);
        real_t rpm_idle = real_t(900);
        Ref<Curve> torque_curve;
        real_t engine_drag = real_t(0.03);
        real_t engine_brake = real_t(10);
        real_t engine_moment = real_t(0.25);
        real_t engine_bsfc = real_t(0.3);
        Ref<AudioStream> engine_sound;

        // Drivetrain variables
        DRIVE_TYPE drivetype = RWD;
        Array gear_ratios = Array();
        real_t final_drive = real_t(3.7);
        real_t reverse_ratio = real_t(3.9);
        real_t gear_inertia = real_t(0.02);
        DIFF_TYPE rear_diff = LIMITED_SLIP;
        DIFF_TYPE front_diff = LIMITED_SLIP;
        real_t rear_diff_preload = real_t(50);
        real_t front_diff_preload = real_t(50);
        real_t rear_diff_power_ratio = real_t(3.5);
        real_t front_diff_power_ratio = real_t(3.5);
        real_t rear_diff_coast_ratio = real_t(1);
        real_t front_diff_coast_ratio = real_t(1);
        real_t center_split_f_r = real_t(0.4); // 4wd torque split front / rear; range(0, 1)
        real_t clutch_friction = real_t(500);

        // Aero
        real_t cd = real_t(0.3);
        real_t air_density = real_t(1.225);
        real_t frontal_area = real_t(2);

        // Constants
        const real_t PETROL_KG_L = real_t(0.7489);
        const int NM_2_KW = 9549;
        const real_t AV_2_RPM = real_t(60) / Math_TAU;

        // Controller inputs
        real_t throttle_input = real_t(0);
        real_t steering_input = real_t(0);
        real_t brake_input = real_t(0);
        real_t handbrake_input = real_t(0);
        real_t clutch_input = real_t(0);

        // Misc
        real_t fuel = real_t(0);
        real_t drag_torque = real_t(0);
        real_t torque_out = real_t(0);
        real_t net_drive = real_t(0);
        real_t engine_net_torque = real_t(0);

        real_t clutch_reaction_torque = real_t(0);
        real_t drive_reaction_torque = real_t(0);

        real_t rpm = real_t(0);
        real_t engine_angular_vel = real_t(0);

        real_t rear_brake_force = real_t(0);
        real_t front_brake_force = real_t(0);
        int selected_gear = 0;

        real_t drive_inertia = real_t(0); // includes every inertia after engine and before wheels (wheels include brakes inertia)

        real_t r_split = real_t(0.5);
        real_t f_split = real_t(0.5);

        real_t steering_amount = real_t(0);

        real_t speedo = real_t(0);
        real_t wheel_radius = real_t(0);
        Array susp_comp = Array();

        real_t avg_rear_spin = real_t(0);
        real_t avg_front_spin = real_t(0);

        Vector3 local_vel = Vector3(0, 0, 0);
        Vector3 prev_pos = Vector3(0, 0, 0);
        real_t z_vel = real_t(0);
        real_t x_vel = real_t(0);

        //Vector<ng_vehicle_wheel *> wheels;
        //Node wheel_fl;
        //Node wheel_fr;
        //Node wheel_rl;
        //Node wheel_rr;
        AudioStreamPlayer audioplayer;

    // We'll put our setters and getters here to help keep it easier to read the main
    // bulk of the class...
    public:
        void set_max_steer(real_t steering_amount);
        real_t get_max_steer(void);
        void set_front_brake_bias(real_t bias_amount);
        real_t get_front_brake_bias(void);
        void set_steer_speed(real_t speed_amount);
        real_t get_steer_speed(void);
        void set_max_brake_force(real_t force_amount);
        real_t get_max_brake_force(void);
        void set_fuel_tank_size(real_t tank_size);
        real_t get_fuel_tank_size(void);
        void set_fuel_percentage(real_t fuel_amount_percentage);
        real_t get_fuel_percentage(void);
        void set_max_torque(real_t torque_max);
        real_t get_max_torque(void);
        void set_max_engine_rpm(real_t engine_rpm_max);
        real_t get_max_engine_rpm(void);
        void set_rpm_clutch_out(real_t clutch_out);
        real_t get_rpm_clutch_out(void);
        void set_rpm_idle(real_t idle_rpm);
        real_t get_rpm_idle(void);
        void set_torque_curve(Ref<Curve> curve); // maybe reference?  I can't remember
        Ref<Curve> get_torque_curve(void);
        void set_engine_drag(real_t drag);
        real_t get_engine_drag(void);
        void set_engine_brake(real_t brake_amount);
        real_t get_engine_brake(void);
        void set_engine_moment(real_t moment);
        real_t get_engine_moment(void);
        void set_engine_bsfc(real_t bsfc);
        real_t get_engine_bsfc(void);
        void set_engine_sound(Ref<AudioStream> sound);
        Ref<AudioStream> get_engine_sound(void);
        void set_drivetype(DRIVE_TYPE drive_type);
        int get_drivetype(void);
        void set_gear_ratios(Array ratios);
        Array get_gear_ratios(void);
        void set_final_drive(real_t ratio);
        real_t get_final_drive(void);
        void set_reverse_ratio(real_t ratio);
        real_t get_reverse_ratio(void);
        void set_gear_inertia(real_t amount);
        real_t get_gear_inertia(void);
        void set_rear_diff(DIFF_TYPE diff_type);
        int get_rear_diff(void);
        void set_front_diff(DIFF_TYPE diff_type);
        int get_front_diff(void);
        void set_rear_diff_preload(real_t preload);
        real_t get_rear_diff_preload(void);
        void set_front_diff_preload(real_t preload);
        real_t get_front_diff_preload(void);
        void set_rear_diff_power_ratio(real_t ratio);
        real_t get_rear_diff_power_ratio(void);
        void set_front_diff_power_ratio(real_t ratio);
        real_t get_front_diff_power_ratio(void);
        void set_rear_diff_coast_ratio(real_t ratio);
        real_t get_rear_diff_coast_ratio(void);
        void set_front_diff_coast_ratio(real_t ratio);
        real_t get_front_diff_coast_ratio(void);
        void set_center_split_f_r(real_t center_split);
        real_t get_center_split_f_r(void);
        void set_clutch_friction(real_t friction);
        real_t get_clutch_friction(void);
        void set_cd(real_t amount);
        real_t get_cd(void);
        void set_air_density(real_t density);
        real_t get_air_density(void);
        void set_frontal_area(real_t area);
        real_t get_frontal_area(void);
        void set_throttle_input(real_t input);
        real_t get_throttle_input(void);
        void set_brake_input(real_t input);
        real_t get_brake_input(void);
        void set_clutch_input(real_t input);
        real_t get_clutch_input(void);
        void set_handbrake_input(real_t input);
        real_t get_handbrake_input(void);
        void set_steering_input(real_t input);
        real_t get_steering_input(void);
        void set_fuel(real_t amount);
        real_t get_fuel(void);
        void set_drag_torque(real_t amount);
        real_t get_drag_torque(void);
        void set_torque_out(real_t out);
        real_t get_torque_out(void);
        void set_net_drive(real_t amount);
        real_t get_net_drive(void);
        void set_engine_net_torque(real_t amount);
        real_t get_engine_net_torque(void);
        void set_clutch_reaction_torque(real_t amount);
        real_t get_clutch_reaction_torque(void);
        void set_drive_reaction_torque(real_t amount);
        real_t get_drive_reaction_torque(void);
        void set_rpm(real_t r_p_m);
        real_t get_rpm(void);
        void set_engine_angular_vel(real_t velocity);
        real_t get_engine_angular_vel(void);
        void set_rear_brake_force(real_t force);
        real_t get_rear_brake_force(void);
        void set_front_brake_force(real_t force);
        real_t get_front_brake_force(void);
        void set_selected_gear(int gear);
        real_t get_selected_gear(void);
        void set_drive_inertia(real_t amount);
        real_t get_drive_inertia(void);
        void set_r_split(real_t rear_split);
        real_t get_r_split(void);
        void set_f_split(real_t front_split);
        real_t get_f_split(void);
        void set_steering_amount(real_t amount);
        real_t get_steering_amount(void);
        void set_speedo(real_t speed);
        real_t get_speedo(void);
        void set_wheel_radius(real_t radius);
        real_t get_wheel_radius(void);
        void set_susp_comp(Array compression);
        Array get_susp_comp(void);
        void set_avg_rear_spin(real_t rear_spin);
        real_t get_avg_rear_spin(void);
        void set_avg_front_spin(real_t front_spin);
        real_t get_avg_front_spin(void);
        void set_local_vel(Vector3 velocity);
        Vector3 get_local_vel(void);
        void set_prev_pos(Vector3 pos);
        Vector3 get_prev_pos(void);
        void set_z_vel(real_t velocity);
        real_t get_z_vel(void);
        void set_x_vel(real_t velocity);
        real_t get_x_vel(void);
};

VARIANT_ENUM_CAST(mf_vehicle_body::DIFF_TYPE);
VARIANT_ENUM_CAST(mf_vehicle_body::DRIVE_TYPE);

#endif // VEHICLE_H

