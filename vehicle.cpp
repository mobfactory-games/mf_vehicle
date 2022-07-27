/*
 * bl = 2
 * br = 3
 * fr = 0
 * fl = 1
 */

#include <iostream>
#include <algorithm>
#include <core/class_db.h>

#include "vehicle.h"
#include "wheel.h"
#include "math.h"

mf_vehicle_body::mf_vehicle_body(void) {
    susp_comp.push_back(real_t(0.5));
    susp_comp.push_back(real_t(0.5));
    susp_comp.push_back(real_t(0.5));
    susp_comp.push_back(real_t(0.5));

    /*for (int i = 0; i < wheel_data.size(); i++) {
        wheel_list[wheel_data[i]->get_name()] = i;
        OS::get_singleton()->print("%ls: %d\n", String(wheel_data[i]->get_name()).c_str(), i);
    }*/

//    if (wheel_data.empty() != true)
//        wheel_radius = wheel_data[int(wheel_list["wheel_front_left"])]->tire_radius;
    fuel = fuel_tank_size * fuel_percentage * real_t(0.01);
    mass += fuel * PETROL_KG_L;
}

mf_vehicle_body::~mf_vehicle_body(void) {
    susp_comp.empty();
}

// named these the same as the functions they should be called in.
void mf_vehicle_body::process(real_t delta) {
    drive_inertia = engine_moment + Math::pow(abs(gear_ratio()), real_t(2)) * gear_inertia;

    front_brake_force = max_brake_force * brake_input * front_brake_bias * real_t(0.5);

    real_t rear_brake_input = std::max(brake_input, handbrake_input);
    rear_brake_force = max_brake_force * rear_brake_input * (real_t(1) - front_brake_bias) * real_t(0.5);
}

void mf_vehicle_body::physics_process(real_t delta, Dictionary surface) {
    local_vel = get_global_transform().get_basis().xform_inv((get_global_transform().get_origin() - prev_pos) / delta);
    prev_pos = get_global_transform().get_origin();
    z_vel = -local_vel.z;
    x_vel = local_vel.x;

    drag_force();

    // Anti-rollbar
    if (wheel_data.empty() == false) {
        if (surface.empty() == false) {
            Array prev_comp = Array(susp_comp);
            susp_comp[int(wheel_list["wheel_rear_left"])] = wheel_data[int(wheel_list["wheel_rear_left"])]->apply_forces(prev_comp[int(wheel_list["wheel_rear_right"])], delta, surface["wheel_rear_left"]);
            susp_comp[int(wheel_list["wheel_rear_right"])] = wheel_data[int(wheel_list["wheel_rear_right"])]->apply_forces(prev_comp[int(wheel_list["wheel_rear_left"])], delta, surface["wheel_rear_right"]);
            susp_comp[int(wheel_list["wheel_front_right"])] = wheel_data[int(wheel_list["wheel_front_right"])]->apply_forces(prev_comp[int(wheel_list["wheel_front_left"])], delta, surface["wheel_front_right"]);
            susp_comp[int(wheel_list["wheel_front_left"])] = wheel_data[int(wheel_list["wheel_front_left"])]->apply_forces(prev_comp[int(wheel_list["wheel_front_right"])], delta, surface["wheel_front_left"]);
        }
    }

    // steering with steer speed
    if (steering_input < steering_amount) {
        steering_amount -= steer_speed * delta;
        if (steering_input > steering_amount)
            steering_amount = steering_input;
    } else if (steering_input > steering_amount) {
        steering_amount += steer_speed * delta;
        if (steering_input < steering_amount)
            steering_amount = steering_input;
    }

    wheel_data[int(wheel_list["wheel_front_left"])]->steer(steering_amount, max_steer);
    wheel_data[int(wheel_list["wheel_front_right"])]->steer(steering_amount, max_steer);

    // Engine loop
    drag_torque = engine_brake + rpm + engine_drag;
    torque_out = (engine_torque(rpm) + drag_torque) * throttle_input;
    engine_net_torque = torque_out + clutch_reaction_torque - drag_torque;

    rpm += AV_2_RPM * delta * engine_net_torque / engine_moment;
    engine_angular_vel = rpm / AV_2_RPM;

    if (rpm >= max_engine_rpm) {
        torque_out = real_t(0);
        rpm -= real_t(500);
    }

    if (rpm <= (rpm_idle + real_t(10)) && z_vel <= real_t(2))
        clutch_input = real_t(1.0);

    if (selected_gear == 0)
        freewheel(delta);
    else
        engage(delta);

    rpm = std::max(rpm, rpm_idle);

    if (fuel <= real_t(0)) {
        torque_out = real_t(0);
        rpm = real_t(0);
        stop_engine_sound();
    }

    play_engine_sound();
    burn_fuel(delta);
}

real_t mf_vehicle_body::engine_torque(real_t r_p_m) {
    real_t rpm_factor = std::clamp(r_p_m / max_engine_rpm, real_t(0), real_t(1));
    real_t torque_factor = torque_curve->interpolate_baked(rpm_factor);
    return torque_factor * max_torque;
}

void mf_vehicle_body::freewheel(real_t delta) {
    clutch_reaction_torque = real_t(0);
    avg_front_spin = real_t(0);
    wheel_data[int(wheel_list["wheel_rear_left"])]->apply_torque(real_t(0), real_t(0), rear_brake_force, delta);
    wheel_data[int(wheel_list["wheel_rear_right"])]->apply_torque(real_t(0), real_t(0), rear_brake_force, delta);
    wheel_data[int(wheel_list["wheel_front_left"])]->apply_torque(real_t(0), real_t(0), front_brake_force, delta);
    wheel_data[int(wheel_list["wheel_front_right"])]->apply_torque(real_t(0), real_t(0), front_brake_force, delta);
    avg_front_spin += (wheel_data[int(wheel_list["wheel_front_left"])]->spin + wheel_data[int(wheel_list["wheel_front_right"])]->spin) * real_t(0.5);
    speedo = avg_front_spin * wheel_radius * real_t(3.6);
}

void mf_vehicle_body::engage(real_t delta) {
    avg_rear_spin = real_t(0);
    avg_front_spin = real_t(0);

    avg_rear_spin += (wheel_data[int(wheel_list["wheel_rear_left"])]->spin + wheel_data[int(wheel_list["wheel_rear_right"])]->spin) * real_t(0.5);
    avg_front_spin += (wheel_data[int(wheel_list["wheel_front_left"])]->spin + wheel_data[int(wheel_list["wheel_front_right"])]->spin) * real_t(0.5);

    real_t gearbox_shaft_speed = real_t(0);

    if (drivetype == RWD)
        gearbox_shaft_speed = avg_rear_spin * gear_ratio();
    else if (drivetype == FWD)
        gearbox_shaft_speed = avg_front_spin * gear_ratio();
    else if (drivetype == AWD)
        gearbox_shaft_speed = (avg_front_spin + avg_rear_spin) * real_t(0.5) * gear_ratio();

    real_t clutch_torque = clutch_friction * (real_t(1) - clutch_input);

    if (engine_angular_vel > gearbox_shaft_speed) {
        clutch_reaction_torque = -clutch_torque;
        drive_reaction_torque = clutch_torque;
    } else {
        clutch_reaction_torque = clutch_torque;
        drive_reaction_torque = -clutch_torque;
    }

    net_drive = drive_reaction_torque * gear_ratio() * (real_t(1) - clutch_input);

    if (drivetype == RWD) {
        rear_wheel_drive(net_drive, delta);
        wheel_data[int(wheel_list["wheel_front_left"])]->apply_torque(real_t(0), real_t(0), front_brake_force, delta);
        wheel_data[int(wheel_list["wheel_front_right"])]->apply_torque(real_t(0), real_t(0), front_brake_force, delta);
    } else if (drivetype == AWD) {
        all_wheel_drive(net_drive, delta);
    } else if (drivetype == FWD) {
        front_wheel_drive(net_drive, delta);
        wheel_data[int(wheel_list["wheel_rear_left"])]->apply_torque(real_t(0), real_t(0), rear_brake_force, delta);
        wheel_data[int(wheel_list["wheel_rear_right"])]->apply_torque(real_t(0), real_t(0), rear_brake_force, delta);
    }

    speedo = avg_front_spin * wheel_radius * real_t(3.6);
}

real_t mf_vehicle_body::gear_ratio(void) {
    if (selected_gear > 0) {
        return real_t(gear_ratios[selected_gear - 1]) * final_drive;
    } else if (selected_gear == -1) {
        return -reverse_ratio * final_drive;
    }
    return real_t(0);
}

void mf_vehicle_body::rear_wheel_drive(real_t drive, real_t delta) {
    bool diff_locked = true;
    real_t t_error = wheel_data[int(wheel_list["wheel_rear_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_left"])]->tire_radius
                    - wheel_data[int(wheel_list["wheel_rear_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_right"])]->tire_radius;

    if (drive * signum(gear_ratio()) > 0) {
        if (Math::abs(t_error) > rear_diff_preload * rear_diff_power_ratio)
            diff_locked = false;
    } else {
        if (Math::abs(t_error) > rear_diff_preload * rear_diff_coast_ratio)
            diff_locked = false;
    }

    if (rear_diff == LOCKED)
        diff_locked = true;
    else if (rear_diff == OPEN_DIFF)
        diff_locked = false;

    if (diff_locked == false) {
        real_t diff_sum = real_t(0);

        diff_sum -= wheel_data[int(wheel_list["wheel_rear_right"])]->apply_torque(drive * (real_t(1) - r_split), drive_inertia, rear_brake_force, delta);
        diff_sum += wheel_data[int(wheel_list["wheel_rear_left"])]->apply_torque(drive * r_split, drive_inertia, rear_brake_force, delta);

        r_split = real_t(0.5) * (std::clamp(diff_sum, real_t(-1), real_t(1)) + real_t(1));
    } else {
        r_split = real_t(0.5);
        // initialize net_torque with previous frames friction
        real_t net_torque = (wheel_data[int(wheel_list["wheel_rear_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_left"])]->tire_radius
                            + wheel_data[int(wheel_list["wheel_rear_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_right"])]->tire_radius);
        net_torque += drive;
        real_t axle_spin = real_t(0);
        // stop wheel if brakes overwhelm other forces
        if (avg_rear_spin < real_t(5) && rear_brake_force > Math::abs(net_torque))
            axle_spin = real_t(0);
        else {
            real_t f_rr = (wheel_data[int(wheel_list["wheel_rear_left"])]->rolling_resistance + wheel_data[int(wheel_list["wheel_rear_right"])]->rolling_resistance);
            net_torque -= (real_t(2) * rear_brake_force + f_rr) * signum(avg_rear_spin);
            axle_spin = avg_rear_spin + (delta * net_torque / (wheel_data[int(wheel_list["wheel_rear_left"])]->wheel_moment
                        + drive_inertia + wheel_data[int(wheel_list["wheel_rear_right"])]->wheel_moment));
        }

        wheel_data[int(wheel_list["wheel_rear_right"])]->apply_solid_axle_spin(axle_spin);
        wheel_data[int(wheel_list["wheel_rear_left"])]->apply_solid_axle_spin(axle_spin);
    }
}

// We'll probably rewrite these functions instead of repeating code except for front
// wheel versus rear wheel.  Since we're using a vector for the wheel_data instead of
// having four variables for each, this should make it easier.
void mf_vehicle_body::front_wheel_drive(real_t drive, real_t delta) {
    bool diff_locked = true;
    real_t t_error = wheel_data[int(wheel_list["wheel_front_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_left"])]->tire_radius - wheel_data[int(wheel_list["wheel_front_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_right"])]->tire_radius;

    if (drive * signum(gear_ratio()) > 0) { // we are powering
        if (Math::abs(t_error) > front_diff_preload * front_diff_power_ratio)
            diff_locked = false;
        else { // we are coasting
            if (Math::abs(t_error) > front_diff_preload * front_diff_coast_ratio)
                diff_locked = false;
        }
    }

    if (front_diff == LOCKED)
        diff_locked = true;
    else if (front_diff == OPEN_DIFF)
        diff_locked = false;

    if (diff_locked == false) {
        real_t diff_sum = real_t(0);

        diff_sum -= wheel_data[int(wheel_list["wheel_front_right"])]->apply_torque(drive * (real_t(1) - f_split), drive_inertia, front_brake_force, delta);
        diff_sum += wheel_data[int(wheel_list["wheel_front_left"])]->apply_torque(drive * f_split, drive_inertia, front_brake_force, delta);

        f_split = real_t(0.5) * (std::clamp(diff_sum, real_t(-1), real_t(1)) + real_t(1));
    } else {
        f_split = real_t(0.5);

        real_t net_torque = (wheel_data[int(wheel_list["wheel_front_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_left"])]->tire_radius
                            + wheel_data[int(wheel_list["wheel_front_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_right"])]->tire_radius);
        net_torque += drive;
        real_t axle_spin = real_t(0);
        if (avg_front_spin < real_t(5) && front_brake_force > Math::abs(net_torque))
            axle_spin = real_t(0);
        else {
            real_t f_rr = (wheel_data[int(wheel_list["wheel_front_left"])]->rolling_resistance + wheel_data[int(wheel_list["wheel_front_right"])]->rolling_resistance);
            net_torque -= (real_t(2) * front_brake_force + f_rr) * signum(avg_front_spin);
            axle_spin = avg_front_spin + (delta * net_torque / wheel_data[int(wheel_list["wheel_front_left"])]->wheel_moment + drive_inertia + wheel_data[int(wheel_list["wheel_front_right"])]->wheel_moment);
        }

        wheel_data[int(wheel_list["wheel_front_right"])]->apply_solid_axle_spin(axle_spin);
        wheel_data[int(wheel_list["wheel_front_left"])]->apply_solid_axle_spin(axle_spin);
    }
}

void mf_vehicle_body::all_wheel_drive(real_t drive, real_t delta) {
    real_t rear_drive = drive * (real_t(1) - center_split_f_r);
    real_t front_drive = drive * center_split_f_r;

    bool front_diff_locked = true;
    bool rear_diff_locked = true;

    real_t front_t_error = wheel_data[int(wheel_list["wheel_front_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_left"])]->tire_radius - wheel_data[int(wheel_list["wheel_front_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_right"])]->tire_radius;
    real_t rear_t_error = wheel_data[int(wheel_list["wheel_rear_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_left"])]->tire_radius - wheel_data[int(wheel_list["wheel_rear_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_right"])]->tire_radius;

    if (drive * signum(gear_ratio()) > 0) {
        if (Math::abs(rear_t_error) > rear_diff_preload * rear_diff_power_ratio)
            rear_diff_locked = false;

        if (Math::abs(front_t_error) > front_diff_preload * front_diff_power_ratio)
            front_diff_locked = false;
    } else {
        if (Math::abs(rear_t_error) > rear_diff_preload * rear_diff_coast_ratio)
            rear_diff_locked = false;

        // should this be coast_ratio?
        if (Math::abs(front_t_error) > front_diff_preload * front_diff_power_ratio)
            front_diff_locked = false;
    }

    if (front_diff == LOCKED)
        front_diff_locked = true;

    if (rear_diff == LOCKED)
        rear_diff_locked = true;

    if (rear_diff_locked == false) {
        real_t rear_diff_sum = real_t(0);

        rear_diff_sum -= wheel_data[int(wheel_list["wheel_rear_right"])]->apply_torque(rear_drive * (real_t(1) - r_split), drive_inertia, rear_brake_force, delta);
        rear_diff_sum += wheel_data[int(wheel_list["wheel_rear_left"])]->apply_torque(rear_drive * r_split, drive_inertia, rear_brake_force, delta);

        r_split = real_t(0.5) * (std::clamp(rear_diff_sum, real_t(-1), real_t(1)) + real_t(1));
    } else {
        r_split = real_t(0.5);

        real_t net_torque = (wheel_data[int(wheel_list["wheel_rear_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_left"])]->tire_radius + wheel_data[int(wheel_list["wheel_rear_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_rear_right"])]->tire_radius);
        net_torque += rear_drive;
        real_t axle_spin = real_t(0);
        if (avg_rear_spin < real_t(5) && rear_brake_force > Math::abs(net_torque))
            axle_spin = real_t(0);
        else {
            real_t f_rr = (wheel_data[int(wheel_list["wheel_rear_left"])]->rolling_resistance + wheel_data[int(wheel_list["wheel_rear_right"])]->rolling_resistance);
            net_torque -= (real_t(2) * rear_brake_force + f_rr) * signum(avg_rear_spin);
            axle_spin = avg_rear_spin + (delta * net_torque / (wheel_data[int(wheel_list["wheel_rear_left"])]->wheel_moment + drive_inertia + wheel_data[int(wheel_list["wheel_rear_right"])]->wheel_moment));
        }

        wheel_data[int(wheel_list["wheel_rear_right"])]->apply_solid_axle_spin(axle_spin);
        wheel_data[int(wheel_list["wheel_rear_left"])]->apply_solid_axle_spin(axle_spin);
    }

    if (front_diff_locked == false) {
        real_t front_diff_sum = real_t(0);

        front_diff_sum -= wheel_data[int(wheel_list["wheel_front_right"])]->apply_torque(front_drive * (real_t(1) - f_split), drive_inertia, front_brake_force, delta);
        front_diff_sum += wheel_data[int(wheel_list["wheel_front_left"])]->apply_torque(front_drive * f_split, drive_inertia, front_brake_force, delta);

        f_split = real_t(0.5) * (std::clamp(front_diff_sum, real_t(-1), real_t(1)) + real_t(1));
    } else {
        f_split = real_t(0.5);

        real_t net_torque = (wheel_data[int(wheel_list["wheel_front_left"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_left"])]->tire_radius + wheel_data[int(wheel_list["wheel_front_right"])]->force_vec.y * wheel_data[int(wheel_list["wheel_front_right"])]->tire_radius);
        net_torque += front_drive;
        real_t axle_spin = real_t(0);

        if (avg_front_spin < real_t(5) && front_brake_force > Math::abs(net_torque))
            axle_spin = real_t(0);
        else {
            real_t f_rr = (wheel_data[int(wheel_list["wheel_front_left"])]->rolling_resistance + wheel_data[int(wheel_list["wheel_front_right"])]->rolling_resistance);
            net_torque -= (real_t(2) * front_brake_force + f_rr) * signum(avg_front_spin);
            axle_spin = avg_front_spin + (delta * net_torque / (wheel_data[int(wheel_list["wheel_front_left"])]->wheel_moment + drive_inertia + wheel_data[int(wheel_list["wheel_front_right"])]->wheel_moment));
        }

        wheel_data[int(wheel_list["wheel_front_right"])]->apply_solid_axle_spin(axle_spin);
        wheel_data[int(wheel_list["wheel_front_left"])]->apply_solid_axle_spin(axle_spin);
    }
}

void mf_vehicle_body::drag_force(void) {
    real_t spd = Math::sqrt(x_vel * x_vel + z_vel * z_vel);
    real_t cdrag = real_t(0.5) * cd * frontal_area * air_density;

    // fdrag.y is the positive in this case because forward is -z in godot
    Vector2 fdrag = Vector2(0, 0);
    fdrag.y = cdrag * z_vel * spd;
    fdrag.x = -cdrag * x_vel * spd;

    add_central_force(get_global_transform().basis.elements[2] * fdrag.y);
    add_central_force(get_global_transform().basis.elements[0] * fdrag.x);
}

void mf_vehicle_body::burn_fuel(real_t delta) {
    real_t fuel_burned = engine_bsfc * torque_out * rpm * delta / (3600 * PETROL_KG_L * NM_2_KW);
    fuel -= fuel_burned;
    mass -= fuel_burned * PETROL_KG_L;
}

void mf_vehicle_body::shift_up(void) {
    if (selected_gear < gear_ratios.size())
        selected_gear += 1;
}

void mf_vehicle_body::shift_down(void) {
    if (selected_gear > -1)
        selected_gear -= 1;
}

void mf_vehicle_body::play_engine_sound(void) {
    real_t pitch_scaler = rpm / 1000;
    if (rpm >= rpm_idle && rpm < max_engine_rpm) {
        if (audioplayer.get_stream() != engine_sound)
            audioplayer.set_stream(engine_sound);
        if (audioplayer.is_playing() == false)
            audioplayer.play();
    }

    if (pitch_scaler > real_t(0.1))
        audioplayer.set_pitch_scale(pitch_scaler);
}

void mf_vehicle_body::stop_engine_sound(void) {
    audioplayer.stop();
}

void mf_vehicle_body::_notification(int p_what) {
    switch(p_what) {
        case NOTIFICATION_ENTER_TREE:
        break;
        case NOTIFICATION_EXIT_TREE:
        break;
    }
}

void mf_vehicle_body::_bind_methods(void) {
    ClassDB::bind_method(D_METHOD("process", "delta"), &mf_vehicle_body::process);
    ClassDB::bind_method(D_METHOD("physics_process", "delta", "surface"), &mf_vehicle_body::physics_process);
    ClassDB::bind_method(D_METHOD("engine_torque", "r_p_m"), &mf_vehicle_body::engine_torque);
    ClassDB::bind_method(D_METHOD("freewheel", "delta"), &mf_vehicle_body::freewheel);
    ClassDB::bind_method(D_METHOD("engage", "delta"), &mf_vehicle_body::engage);
    ClassDB::bind_method(D_METHOD("gear_ratio"), &mf_vehicle_body::gear_ratio);
    ClassDB::bind_method(D_METHOD("rear_wheel_drive", "drive", "delta"), &mf_vehicle_body::rear_wheel_drive);
    ClassDB::bind_method(D_METHOD("front_wheel_drive", "drive", "delta"), &mf_vehicle_body::front_wheel_drive);
    ClassDB::bind_method(D_METHOD("all_wheel_drive", "drive", "delta"), &mf_vehicle_body::all_wheel_drive);
    ClassDB::bind_method(D_METHOD("drag_force"), &mf_vehicle_body::drag_force);
    ClassDB::bind_method(D_METHOD("burn_fuel", "delta"), &mf_vehicle_body::burn_fuel);
    ClassDB::bind_method(D_METHOD("shift_up"), &mf_vehicle_body::shift_up);
    ClassDB::bind_method(D_METHOD("shift_down"), &mf_vehicle_body::shift_down);
    ClassDB::bind_method(D_METHOD("play_engine_sound"), &mf_vehicle_body::play_engine_sound);
    ClassDB::bind_method(D_METHOD("stop_engine_sound"), &mf_vehicle_body::stop_engine_sound);

    // setters and getters
    ClassDB::bind_method(D_METHOD("get_wheel_list"), &mf_vehicle_body::get_wheel_list);
    ClassDB::bind_method(D_METHOD("set_max_steer", "steering_amount"), &mf_vehicle_body::set_max_steer);
    ClassDB::bind_method(D_METHOD("get_max_steer"), &mf_vehicle_body::get_max_steer);
    ClassDB::bind_method(D_METHOD("set_front_brake_bias", "bias_amount"), &mf_vehicle_body::set_front_brake_bias);
    ClassDB::bind_method(D_METHOD("get_front_brake_bias"), &mf_vehicle_body::get_front_brake_bias);
    ClassDB::bind_method(D_METHOD("set_steer_speed", "speed_amount"), &mf_vehicle_body::set_steer_speed);
    ClassDB::bind_method(D_METHOD("get_steer_speed"), &mf_vehicle_body::get_steer_speed);
    ClassDB::bind_method(D_METHOD("set_max_brake_force", "force_amount"), &mf_vehicle_body::set_max_brake_force);
    ClassDB::bind_method(D_METHOD("get_max_brake_force"), &mf_vehicle_body::get_max_brake_force);
    ClassDB::bind_method(D_METHOD("set_fuel_tank_size", "tank_size"), &mf_vehicle_body::set_fuel_tank_size);
    ClassDB::bind_method(D_METHOD("get_fuel_tank_size"), &mf_vehicle_body::get_fuel_tank_size);
    ClassDB::bind_method(D_METHOD("set_fuel_percentage", "fuel_amount_percentage"), &mf_vehicle_body::set_fuel_percentage);
    ClassDB::bind_method(D_METHOD("get_fuel_percentage"), &mf_vehicle_body::get_fuel_percentage);
    ClassDB::bind_method(D_METHOD("set_max_torque", "torque_max"), &mf_vehicle_body::set_max_torque);
    ClassDB::bind_method(D_METHOD("get_max_torque"), &mf_vehicle_body::get_max_torque);
    ClassDB::bind_method(D_METHOD("set_max_engine_rpm", "engine_rpm_max"), &mf_vehicle_body::set_max_engine_rpm);
    ClassDB::bind_method(D_METHOD("get_max_engine_rpm"), &mf_vehicle_body::get_max_engine_rpm);
    ClassDB::bind_method(D_METHOD("set_rpm_clutch_out", "clutch_out"), &mf_vehicle_body::set_rpm_clutch_out);
    ClassDB::bind_method(D_METHOD("get_rpm_clutch_out"), &mf_vehicle_body::get_rpm_clutch_out);
    ClassDB::bind_method(D_METHOD("set_rpm_idle", "idle_rpm"), &mf_vehicle_body::set_rpm_idle);
    ClassDB::bind_method(D_METHOD("get_rpm_idle"), &mf_vehicle_body::get_rpm_idle);
    ClassDB::bind_method(D_METHOD("set_torque_curve", "curve"), &mf_vehicle_body::set_torque_curve); // maybe reference?  I can't remember
    ClassDB::bind_method(D_METHOD("get_torque_curve"), &mf_vehicle_body::get_torque_curve);
    ClassDB::bind_method(D_METHOD("set_engine_drag", "drag"), &mf_vehicle_body::set_engine_drag);
    ClassDB::bind_method(D_METHOD("get_engine_drag"), &mf_vehicle_body::get_engine_drag);
    ClassDB::bind_method(D_METHOD("set_engine_brake", "brake_amount"), &mf_vehicle_body::set_engine_brake);
    ClassDB::bind_method(D_METHOD("get_engine_brake"), &mf_vehicle_body::get_engine_brake);
    ClassDB::bind_method(D_METHOD("set_engine_moment", "moment"), &mf_vehicle_body::set_engine_moment);
    ClassDB::bind_method(D_METHOD("get_engine_moment"), &mf_vehicle_body::get_engine_moment);
    ClassDB::bind_method(D_METHOD("set_engine_bsfc", "bsfc"), &mf_vehicle_body::set_engine_bsfc);
    ClassDB::bind_method(D_METHOD("get_engine_bsfc"), &mf_vehicle_body::get_engine_bsfc);
    ClassDB::bind_method(D_METHOD("set_engine_sound", "sound"), &mf_vehicle_body::set_engine_sound);
    ClassDB::bind_method(D_METHOD("get_engine_sound"), &mf_vehicle_body::get_engine_sound);
    ClassDB::bind_method(D_METHOD("set_drivetype", "drive_type"), &mf_vehicle_body::set_drivetype);
    ClassDB::bind_method(D_METHOD("get_drivetype"), &mf_vehicle_body::get_drivetype);
    ClassDB::bind_method(D_METHOD("set_gear_ratios", "ratios"), &mf_vehicle_body::set_gear_ratios);
    ClassDB::bind_method(D_METHOD("get_gear_ratios"), &mf_vehicle_body::get_gear_ratios);
    ClassDB::bind_method(D_METHOD("set_final_drive", "ratio"), &mf_vehicle_body::set_final_drive);
    ClassDB::bind_method(D_METHOD("get_final_drive"), &mf_vehicle_body::get_final_drive);
    ClassDB::bind_method(D_METHOD("set_reverse_ratio", "ratio"), &mf_vehicle_body::set_reverse_ratio);
    ClassDB::bind_method(D_METHOD("get_reverse_ratio"), &mf_vehicle_body::get_reverse_ratio);
    ClassDB::bind_method(D_METHOD("set_gear_inertia", "amount"), &mf_vehicle_body::set_gear_inertia);
    ClassDB::bind_method(D_METHOD("get_gear_inertia"), &mf_vehicle_body::get_gear_inertia);
    ClassDB::bind_method(D_METHOD("set_rear_diff", "diff_type"), &mf_vehicle_body::set_rear_diff);
    ClassDB::bind_method(D_METHOD("get_rear_diff"), &mf_vehicle_body::get_rear_diff);
    ClassDB::bind_method(D_METHOD("set_front_diff", "diff_type"), &mf_vehicle_body::set_front_diff);
    ClassDB::bind_method(D_METHOD("get_front_diff"), &mf_vehicle_body::get_front_diff);
    ClassDB::bind_method(D_METHOD("set_rear_diff_preload", "preload"), &mf_vehicle_body::set_rear_diff_preload);
    ClassDB::bind_method(D_METHOD("get_rear_diff_preload"), &mf_vehicle_body::get_rear_diff_preload);
    ClassDB::bind_method(D_METHOD("set_front_diff_preload", "preload"), &mf_vehicle_body::set_front_diff_preload);
    ClassDB::bind_method(D_METHOD("get_front_diff_preload"), &mf_vehicle_body::get_front_diff_preload);
    ClassDB::bind_method(D_METHOD("set_rear_diff_power_ratio", "ratio"), &mf_vehicle_body::set_rear_diff_power_ratio);
    ClassDB::bind_method(D_METHOD("get_rear_diff_power_ratio"), &mf_vehicle_body::get_rear_diff_power_ratio);
    ClassDB::bind_method(D_METHOD("set_front_diff_power_ratio", "ratio"), &mf_vehicle_body::set_front_diff_power_ratio);
    ClassDB::bind_method(D_METHOD("get_front_diff_power_ratio"), &mf_vehicle_body::get_front_diff_power_ratio);
    ClassDB::bind_method(D_METHOD("set_rear_diff_coast_ratio", "ratio"), &mf_vehicle_body::set_rear_diff_coast_ratio);
    ClassDB::bind_method(D_METHOD("get_rear_diff_coast_ratio"), &mf_vehicle_body::get_rear_diff_coast_ratio);
    ClassDB::bind_method(D_METHOD("set_front_diff_coast_ratio", "ratio"), &mf_vehicle_body::set_front_diff_coast_ratio);
    ClassDB::bind_method(D_METHOD("get_front_diff_coast_ratio"), &mf_vehicle_body::get_front_diff_coast_ratio);
    ClassDB::bind_method(D_METHOD("set_center_split_f_r", "center_split"), &mf_vehicle_body::set_center_split_f_r);
    ClassDB::bind_method(D_METHOD("get_center_split_f_r"), &mf_vehicle_body::get_center_split_f_r);
    ClassDB::bind_method(D_METHOD("set_clutch_friction", "friction"), &mf_vehicle_body::set_clutch_friction);
    ClassDB::bind_method(D_METHOD("get_clutch_friction"), &mf_vehicle_body::get_clutch_friction);
    ClassDB::bind_method(D_METHOD("set_cd", "amount"), &mf_vehicle_body::set_cd);
    ClassDB::bind_method(D_METHOD("get_cd"), &mf_vehicle_body::get_cd);
    ClassDB::bind_method(D_METHOD("set_air_density", "density"), &mf_vehicle_body::set_air_density);
    ClassDB::bind_method(D_METHOD("get_air_density"), &mf_vehicle_body::get_air_density);
    ClassDB::bind_method(D_METHOD("set_frontal_area", "area"), &mf_vehicle_body::set_frontal_area);
    ClassDB::bind_method(D_METHOD("get_frontal_area"), &mf_vehicle_body::get_frontal_area);
    ClassDB::bind_method(D_METHOD("set_throttle_input", "input"), &mf_vehicle_body::set_throttle_input);
    ClassDB::bind_method(D_METHOD("get_throttle_input"), &mf_vehicle_body::get_throttle_input);
    ClassDB::bind_method(D_METHOD("set_brake_input", "input"), &mf_vehicle_body::set_brake_input);
    ClassDB::bind_method(D_METHOD("get_brake_input"), &mf_vehicle_body::get_brake_input);
    ClassDB::bind_method(D_METHOD("set_clutch_input", "input"), &mf_vehicle_body::set_clutch_input);
    ClassDB::bind_method(D_METHOD("get_clutch_input"), &mf_vehicle_body::get_clutch_input);
    ClassDB::bind_method(D_METHOD("set_handbrake_input", "input"), &mf_vehicle_body::set_handbrake_input);
    ClassDB::bind_method(D_METHOD("get_handbrake_input"), &mf_vehicle_body::get_handbrake_input);
    ClassDB::bind_method(D_METHOD("set_steering_input", "input"), &mf_vehicle_body::set_steering_input);
    ClassDB::bind_method(D_METHOD("get_steering_input"), &mf_vehicle_body::get_steering_input);
    ClassDB::bind_method(D_METHOD("set_fuel", "amount"), &mf_vehicle_body::set_fuel);
    ClassDB::bind_method(D_METHOD("get_fuel"), &mf_vehicle_body::get_fuel);
    ClassDB::bind_method(D_METHOD("set_drag_torque", "amount"), &mf_vehicle_body::set_drag_torque); 
    ClassDB::bind_method(D_METHOD("get_drag_torque"), &mf_vehicle_body::get_drag_torque); 
    ClassDB::bind_method(D_METHOD("set_torque_out", "out"), &mf_vehicle_body::set_torque_out);
    ClassDB::bind_method(D_METHOD("get_torque_out"), &mf_vehicle_body::get_torque_out);
    ClassDB::bind_method(D_METHOD("set_net_drive", "amount"), &mf_vehicle_body::set_net_drive);
    ClassDB::bind_method(D_METHOD("get_net_drive"), &mf_vehicle_body::get_net_drive);
    ClassDB::bind_method(D_METHOD("set_engine_net_torque", "amount"), &mf_vehicle_body::set_engine_net_torque);
    ClassDB::bind_method(D_METHOD("get_engine_net_torque"), &mf_vehicle_body::get_engine_net_torque);
    ClassDB::bind_method(D_METHOD("set_clutch_reaction_torque", "amount"), &mf_vehicle_body::set_clutch_reaction_torque);
    ClassDB::bind_method(D_METHOD("get_clutch_reaction_torque"), &mf_vehicle_body::get_clutch_reaction_torque);
    ClassDB::bind_method(D_METHOD("set_drive_reaction_torque", "amount"), &mf_vehicle_body::set_drive_reaction_torque);
    ClassDB::bind_method(D_METHOD("get_drive_reaction_torque"), &mf_vehicle_body::get_drive_reaction_torque);
    ClassDB::bind_method(D_METHOD("set_rpm", "r_p_m"), &mf_vehicle_body::set_rpm);
    ClassDB::bind_method(D_METHOD("get_rpm"), &mf_vehicle_body::get_rpm);
    ClassDB::bind_method(D_METHOD("set_engine_angular_vel", "velocity"), &mf_vehicle_body::set_engine_angular_vel);
    ClassDB::bind_method(D_METHOD("get_engine_angular_vel"), &mf_vehicle_body::get_engine_angular_vel);
    ClassDB::bind_method(D_METHOD("set_rear_brake_force", "force"), &mf_vehicle_body::set_rear_brake_force);
    ClassDB::bind_method(D_METHOD("get_rear_brake_force"), &mf_vehicle_body::get_rear_brake_force);
    ClassDB::bind_method(D_METHOD("set_front_brake_force", "force"), &mf_vehicle_body::set_front_brake_force);
    ClassDB::bind_method(D_METHOD("get_front_brake_force"), &mf_vehicle_body::get_front_brake_force);
    ClassDB::bind_method(D_METHOD("set_selected_gear", "gear"), &mf_vehicle_body::set_selected_gear);
    ClassDB::bind_method(D_METHOD("get_selected_gear"), &mf_vehicle_body::get_selected_gear);
    ClassDB::bind_method(D_METHOD("set_drive_inertia", "amount"), &mf_vehicle_body::set_drive_inertia);
    ClassDB::bind_method(D_METHOD("get_drive_inertia"), &mf_vehicle_body::get_drive_inertia);
    ClassDB::bind_method(D_METHOD("set_r_split", "rear_split"), &mf_vehicle_body::set_r_split);
    ClassDB::bind_method(D_METHOD("get_r_split"), &mf_vehicle_body::get_r_split);
    ClassDB::bind_method(D_METHOD("set_f_split", "front_split"), &mf_vehicle_body::set_f_split);
    ClassDB::bind_method(D_METHOD("get_f_split"), &mf_vehicle_body::get_f_split);
    ClassDB::bind_method(D_METHOD("set_steering_amount", "amount"), &mf_vehicle_body::set_steering_amount);
    ClassDB::bind_method(D_METHOD("get_steering_amount"), &mf_vehicle_body::get_steering_amount);
    ClassDB::bind_method(D_METHOD("set_speedo", "speed"), &mf_vehicle_body::set_speedo);
    ClassDB::bind_method(D_METHOD("get_speedo"), &mf_vehicle_body::get_speedo);
    ClassDB::bind_method(D_METHOD("set_wheel_radius", "radius"), &mf_vehicle_body::set_wheel_radius);
    ClassDB::bind_method(D_METHOD("get_wheel_radius"), &mf_vehicle_body::get_wheel_radius);
    ClassDB::bind_method(D_METHOD("set_susp_comp", "compression"), &mf_vehicle_body::set_susp_comp);
    ClassDB::bind_method(D_METHOD("get_susp_comp"), &mf_vehicle_body::get_susp_comp);
    ClassDB::bind_method(D_METHOD("set_avg_rear_spin", "rear_spin"), &mf_vehicle_body::set_avg_rear_spin);
    ClassDB::bind_method(D_METHOD("get_avg_rear_spin"), &mf_vehicle_body::get_avg_rear_spin);
    ClassDB::bind_method(D_METHOD("set_avg_front_spin", "front_spin"), &mf_vehicle_body::set_avg_front_spin);
    ClassDB::bind_method(D_METHOD("get_avg_front_spin"), &mf_vehicle_body::get_avg_front_spin);
    ClassDB::bind_method(D_METHOD("set_local_vel", "velocity"), &mf_vehicle_body::set_local_vel);
    ClassDB::bind_method(D_METHOD("get_local_vel"), &mf_vehicle_body::get_local_vel);
    ClassDB::bind_method(D_METHOD("set_prev_pos", "pos"), &mf_vehicle_body::set_prev_pos);
    ClassDB::bind_method(D_METHOD("get_prev_pos"), &mf_vehicle_body::get_prev_pos);
    ClassDB::bind_method(D_METHOD("set_z_vel", "velocity"), &mf_vehicle_body::set_z_vel);
    ClassDB::bind_method(D_METHOD("get_z_vel"), &mf_vehicle_body::get_z_vel);
    ClassDB::bind_method(D_METHOD("set_x_vel", "velocity"), &mf_vehicle_body::set_x_vel);
    ClassDB::bind_method(D_METHOD("get_x_vel"), &mf_vehicle_body::get_x_vel);

    // properties
    // ADD_PROPERTY(PropertyInfo(Variant::, ""), "set_", "get_");
    // ADD_PROPERTYI(PropertyInfo(Variant::OBJECT, "anim_offset_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_param_curve", "get_param_curve", PARAM_ANIM_OFFSET);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_steer"), "set_max_steer", "get_max_steer");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "front_brake_bias", PROPERTY_HINT_RANGE, "0,1,0.1"), "set_front_brake_bias", "get_front_brake_bias");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "steer_speed"), "set_steer_speed", "get_steer_speed");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_brake_force"), "set_max_brake_force", "get_max_brake_force");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "fuel_tank_size"), "set_fuel_tank_size", "get_fuel_tank_size");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "fuel_percentage"), "set_fuel_percentage", "get_fuel_percentage");
    ADD_GROUP("Engine Variables", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_torque"), "set_max_torque", "get_max_torque");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "max_engine_rpm"), "set_max_engine_rpm", "get_max_engine_rpm");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "rpm_clutch_out"), "set_rpm_clutch_out", "get_rpm_clutch_out");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "rpm_idle"), "set_rpm_idle", "get_rpm_idle");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "torque_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_torque_curve", "get_torque_curve");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "engine_drag"), "set_engine_drag", "get_engine_drag");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "engine_brake"), "set_engine_brake", "get_engine_brake");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "engine_moment"), "set_engine_moment", "get_engine_moment");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "engine_bsfc"), "set_engine_bsfc", "get_engine_bsfc");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine_sound", PROPERTY_HINT_RESOURCE_TYPE, "AudioStream"), "set_engine_sound", "get_engine_sound");
    ADD_GROUP("Drivetrain Variables", "");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "drivetype", PROPERTY_HINT_ENUM, "FWD,RWD,AWD"), "set_drivetype", "get_drivetype");
    ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "final_drive"), "set_final_drive", "get_final_drive");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "reverse_ratio"), "set_reverse_ratio", "get_reverse_ratio");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "gear_inertia"), "set_gear_inertia", "get_gear_inertia");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "rear_diff", PROPERTY_HINT_ENUM, "Limited Slip,Open Diff,Locked"), "set_rear_diff", "get_rear_diff");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "front_diff", PROPERTY_HINT_ENUM, "Limited Slip,Open Diff,Locked"), "set_front_diff", "get_front_diff");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "rear_diff_preload"), "set_rear_diff_preload", "get_rear_diff_preload");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "front_diff_preload"), "set_front_diff_preload", "get_front_diff_preload");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "rear_diff_power_ratio"), "set_rear_diff_power_ratio", "get_rear_diff_power_ratio");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "front_diff_coast_ratio"), "set_front_diff_coast_ratio", "get_front_diff_coast_ratio");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "center_split_f_r", PROPERTY_HINT_RANGE, "0,1,0.1"), "set_center_split_f_r", "get_center_split_f_r");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "clutch_friction"), "set_clutch_friction", "get_clutch_friction");
    ADD_GROUP("Aero", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "cd"), "set_cd", "get_cd");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "air_density"), "set_air_density", "get_air_density");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "frontal_area"), "set_frontal_area", "get_frontal_area");

    BIND_ENUM_CONSTANT(LIMITED_SLIP);
    BIND_ENUM_CONSTANT(OPEN_DIFF);
    BIND_ENUM_CONSTANT(LOCKED);
    BIND_ENUM_CONSTANT(FWD);
    BIND_ENUM_CONSTANT(RWD);
    BIND_ENUM_CONSTANT(AWD);
}

Dictionary mf_vehicle_body::get_wheel_list(void) {
    return wheel_list;
}

void mf_vehicle_body::set_max_steer(real_t steering_amount) {
    max_steer = steering_amount;
}

real_t mf_vehicle_body::get_max_steer(void) {
    return max_steer;
}

void mf_vehicle_body::set_front_brake_bias(real_t bias_amount) {
    front_brake_bias = bias_amount;
}

real_t mf_vehicle_body::get_front_brake_bias(void) {
    return front_brake_bias;
}

void mf_vehicle_body::set_steer_speed(real_t speed_amount) {
    steer_speed = speed_amount;
}

real_t mf_vehicle_body::get_steer_speed(void) {
    return steer_speed;
}

void mf_vehicle_body::set_max_brake_force(real_t force_amount) {
    max_brake_force = force_amount;
}

real_t mf_vehicle_body::get_max_brake_force(void) {
    return max_brake_force;
}

void mf_vehicle_body::set_fuel_tank_size(real_t tank_size) {
    fuel_tank_size = tank_size;
}

real_t mf_vehicle_body::get_fuel_tank_size(void) {
    return fuel_tank_size;
}

void mf_vehicle_body::set_fuel_percentage(real_t fuel_amount_percentage) {
    fuel_percentage = fuel_amount_percentage;
}

real_t mf_vehicle_body::get_fuel_percentage(void) {
    return fuel_percentage;
}

void mf_vehicle_body::set_max_torque(real_t torque_max) {
    max_torque = torque_max;
}

real_t mf_vehicle_body::get_max_torque(void) {
    return max_torque;
}

void mf_vehicle_body::set_max_engine_rpm(real_t engine_rpm_max) {
    max_engine_rpm = engine_rpm_max;
}

real_t mf_vehicle_body::get_max_engine_rpm(void) {
    return max_engine_rpm;
}

void mf_vehicle_body::set_rpm_clutch_out(real_t clutch_out) {
    rpm_clutch_out = clutch_out;
}

real_t mf_vehicle_body::get_rpm_clutch_out(void) {
    return rpm_clutch_out;
}

void mf_vehicle_body::set_rpm_idle(real_t idle_rpm) {
    rpm_idle = idle_rpm;
}

real_t mf_vehicle_body::get_rpm_idle(void) {
    return rpm_idle;
}

void mf_vehicle_body::set_torque_curve(Ref<Curve> curve) {
    torque_curve = curve;
}
 // maybe reference?  I can't remember
Ref<Curve> mf_vehicle_body::get_torque_curve(void) {
    return torque_curve;
}

void mf_vehicle_body::set_engine_drag(real_t drag) {
    engine_drag = drag;
}

real_t mf_vehicle_body::get_engine_drag(void) {
    return engine_drag;
}

void mf_vehicle_body::set_engine_brake(real_t brake_amount) {
    engine_brake = brake_amount;
}

real_t mf_vehicle_body::get_engine_brake(void) {
    return engine_brake;
}

void mf_vehicle_body::set_engine_moment(real_t moment) {
    engine_moment = moment;
}

real_t mf_vehicle_body::get_engine_moment(void) {
    return engine_moment;
}

void mf_vehicle_body::set_engine_bsfc(real_t bsfc) {
    engine_bsfc = bsfc;
}

real_t mf_vehicle_body::get_engine_bsfc(void) {
    return engine_bsfc;
}

void mf_vehicle_body::set_engine_sound(Ref<AudioStream> sound) {
    engine_sound = sound;
}

Ref<AudioStream> mf_vehicle_body::get_engine_sound(void) {
    return engine_sound;
}

void mf_vehicle_body::set_drivetype(DRIVE_TYPE drive_type) {
    drivetype = drive_type;
}

int mf_vehicle_body::get_drivetype(void) {
    return drivetype;
}

void mf_vehicle_body::set_gear_ratios(Array ratios) {
    gear_ratios = ratios;
}

Array mf_vehicle_body::get_gear_ratios(void) {
    return gear_ratios;
}

void mf_vehicle_body::set_final_drive(real_t ratio) {
    final_drive = ratio;
}

real_t mf_vehicle_body::get_final_drive(void) {
    return final_drive;
}

void mf_vehicle_body::set_reverse_ratio(real_t ratio) {
    reverse_ratio = ratio;
}

real_t mf_vehicle_body::get_reverse_ratio(void) {
    return reverse_ratio;
}

void mf_vehicle_body::set_gear_inertia(real_t amount) {
    gear_inertia = amount;
}

real_t mf_vehicle_body::get_gear_inertia(void) {
    return gear_inertia;
}

void mf_vehicle_body::set_rear_diff(DIFF_TYPE diff_type) {
    rear_diff = diff_type;
}

int mf_vehicle_body::get_rear_diff(void) {
    return rear_diff;
}

void mf_vehicle_body::set_front_diff(DIFF_TYPE diff_type) {
    front_diff = diff_type;
}

int mf_vehicle_body::get_front_diff(void) {
    return front_diff;
}

void mf_vehicle_body::set_rear_diff_preload(real_t preload) {
    rear_diff_preload = preload;
}

real_t mf_vehicle_body::get_rear_diff_preload(void) {
    return rear_diff_preload;
}

void mf_vehicle_body::set_front_diff_preload(real_t preload) {
    front_diff_preload = preload;
}

real_t mf_vehicle_body::get_front_diff_preload(void) {
    return front_diff_preload;
}

void mf_vehicle_body::set_rear_diff_power_ratio(real_t ratio) {
    rear_diff_power_ratio = ratio;
}

real_t mf_vehicle_body::get_rear_diff_power_ratio(void) {
    return rear_diff_power_ratio;
}

void mf_vehicle_body::set_front_diff_power_ratio(real_t ratio) {
    front_diff_power_ratio = ratio;
}

real_t mf_vehicle_body::get_front_diff_power_ratio(void) {
    return front_diff_power_ratio;
}

void mf_vehicle_body::set_rear_diff_coast_ratio(real_t ratio) {
    rear_diff_coast_ratio = ratio;
}

real_t mf_vehicle_body::get_rear_diff_coast_ratio(void) {
    return rear_diff_coast_ratio;
}

void mf_vehicle_body::set_front_diff_coast_ratio(real_t ratio) {
    front_diff_coast_ratio = ratio;
}

real_t mf_vehicle_body::get_front_diff_coast_ratio(void) {
    return front_diff_coast_ratio;
}

void mf_vehicle_body::set_center_split_f_r(real_t center_split) {
    center_split_f_r = center_split;
}

real_t mf_vehicle_body::get_center_split_f_r(void) {
    return center_split_f_r;
}

void mf_vehicle_body::set_clutch_friction(real_t friction) {
    clutch_friction = friction;
}

real_t mf_vehicle_body::get_clutch_friction(void) {
    return clutch_friction;
}

void mf_vehicle_body::set_cd(real_t amount) {
    cd = amount;
}

real_t mf_vehicle_body::get_cd(void) {
    return cd;
}

void mf_vehicle_body::set_air_density(real_t density) {
    air_density = density;
}

real_t mf_vehicle_body::get_air_density(void) {
    return air_density;
}

void mf_vehicle_body::set_frontal_area(real_t area) {
    frontal_area = area;
}

real_t mf_vehicle_body::get_frontal_area(void) {
    return frontal_area;
}

void mf_vehicle_body::set_throttle_input(real_t input) {
    throttle_input = input;
}

real_t mf_vehicle_body::get_throttle_input(void) {
    return throttle_input;
}

void mf_vehicle_body::set_brake_input(real_t input) {
    brake_input = input;
}

real_t mf_vehicle_body::get_brake_input(void) {
    return brake_input;
}

void mf_vehicle_body::set_clutch_input(real_t input) {
    clutch_input = input;
}

real_t mf_vehicle_body::get_clutch_input(void) {
    return clutch_input;
}

void mf_vehicle_body::set_handbrake_input(real_t input) {
    handbrake_input = input;
}

real_t mf_vehicle_body::get_handbrake_input(void) {
    return handbrake_input;
}

void mf_vehicle_body::set_steering_input(real_t input) {
    steering_input = input;
}

real_t mf_vehicle_body::get_steering_input(void) {
    return steering_input;
}

void mf_vehicle_body::set_fuel(real_t amount) {
    fuel = amount;
}

real_t mf_vehicle_body::get_fuel(void) {
    return fuel;
}

void mf_vehicle_body::set_drag_torque(real_t amount) {
    drag_torque = amount;
}

real_t mf_vehicle_body::get_drag_torque(void) {
    return drag_torque;
}

void mf_vehicle_body::set_torque_out(real_t out) {
    torque_out = out;
}

real_t mf_vehicle_body::get_torque_out(void) {
    return torque_out;
}

void mf_vehicle_body::set_net_drive(real_t amount) {
    net_drive = amount;
}

real_t mf_vehicle_body::get_net_drive(void) {
    return net_drive;
}

void mf_vehicle_body::set_engine_net_torque(real_t amount) {
    engine_net_torque = amount;
}

real_t mf_vehicle_body::get_engine_net_torque(void) {
    return engine_net_torque;
}

void mf_vehicle_body::set_clutch_reaction_torque(real_t amount) {
    clutch_reaction_torque = amount;
}

real_t mf_vehicle_body::get_clutch_reaction_torque(void) {
    return clutch_reaction_torque;
}

void mf_vehicle_body::set_drive_reaction_torque(real_t amount) {
    drive_reaction_torque = amount;
}

real_t mf_vehicle_body::get_drive_reaction_torque(void) {
    return drive_reaction_torque;
}

void mf_vehicle_body::set_rpm(real_t r_p_m) {
    rpm = r_p_m;
}

real_t mf_vehicle_body::get_rpm(void) {
    return rpm;
}

void mf_vehicle_body::set_engine_angular_vel(real_t velocity) {
    engine_angular_vel = velocity;
}

real_t mf_vehicle_body::get_engine_angular_vel(void) {
    return engine_angular_vel;
}

void mf_vehicle_body::set_rear_brake_force(real_t force) {
    rear_brake_force = force;
}

real_t mf_vehicle_body::get_rear_brake_force(void) {
    return rear_brake_force;
}

void mf_vehicle_body::set_front_brake_force(real_t force) {
    front_brake_force = force;
}

real_t mf_vehicle_body::get_front_brake_force(void) {
    return front_brake_force;
}

void mf_vehicle_body::set_selected_gear(int gear) {
    selected_gear = gear;
}

real_t mf_vehicle_body::get_selected_gear(void) {
    return selected_gear;
}

void mf_vehicle_body::set_drive_inertia(real_t amount) {
    drive_inertia = amount;
}

real_t mf_vehicle_body::get_drive_inertia(void) {
    return drive_inertia;
}

void mf_vehicle_body::set_r_split(real_t rear_split) {
    r_split = rear_split;
}

real_t mf_vehicle_body::get_r_split(void) {
    return r_split;
}

void mf_vehicle_body::set_f_split(real_t front_split) {
    f_split = front_split;
}

real_t mf_vehicle_body::get_f_split(void) {
    return f_split;
}

void mf_vehicle_body::set_steering_amount(real_t amount) {
    steering_amount = amount;
}

real_t mf_vehicle_body::get_steering_amount(void) {
    return steering_amount;
}

void mf_vehicle_body::set_speedo(real_t speed) {
    speedo = speed;
}

real_t mf_vehicle_body::get_speedo(void) {
    return speedo;
}

void mf_vehicle_body::set_wheel_radius(real_t radius) {
    wheel_radius = radius;
}

real_t mf_vehicle_body::get_wheel_radius(void) {
    return wheel_radius;
}

void mf_vehicle_body::set_susp_comp(Array compression) {
    susp_comp = compression;
}

Array mf_vehicle_body::get_susp_comp(void) {
    return susp_comp;
}

void mf_vehicle_body::set_avg_rear_spin(real_t rear_spin) {
    avg_rear_spin = rear_spin;
}

real_t mf_vehicle_body::get_avg_rear_spin(void) {
    return avg_rear_spin;
}

void mf_vehicle_body::set_avg_front_spin(real_t front_spin) {
    avg_front_spin = front_spin;
}

real_t mf_vehicle_body::get_avg_front_spin(void) {
    return avg_front_spin;
}

void mf_vehicle_body::set_local_vel(Vector3 velocity) {
    local_vel = velocity;
}

Vector3 mf_vehicle_body::get_local_vel(void) {
    return local_vel;
}

void mf_vehicle_body::set_prev_pos(Vector3 pos) {
    prev_pos = pos;
}

Vector3 mf_vehicle_body::get_prev_pos(void) {
    return prev_pos;
}

void mf_vehicle_body::set_z_vel(real_t velocity) {
    z_vel = velocity;
}

real_t mf_vehicle_body::get_z_vel(void) {
    return z_vel;
}

void mf_vehicle_body::set_x_vel(real_t velocity) {
    x_vel = velocity;
}

real_t mf_vehicle_body::get_x_vel(void) {
    return x_vel;
}

