
#include <algorithm>
#include <core/class_db.h>
#include <scene/resources/cylinder_shape.h>

#include "vehicle.h"
#include "wheel.h"
#include "math.h"

mf_vehicle_wheel::mf_vehicle_wheel(void) {
}

void mf_vehicle_wheel::init(void) {
    if (body == nullptr) {
        OS::get_singleton()->print("wheel has no body\n");
        return;
    }

    OS::get_singleton()->print("Found wheel: %ls\n", String(get_name()).c_str());

    real_t nominal_load = body->get_weight() * real_t(0.25);
    wheel_moment = real_t(0.5) * wheel_mass * Math::pow(tire_radius, real_t(2));
    set_cast_to(Vector3(0, -1, 0) * (spring_length + tire_radius));

    Ref<CylinderShape> cylinder = collider->get_shape();

    cylinder->set_radius(tire_radius - real_t(0.2));
    cylinder->set_height(tire_width);

    peak_sa = lateral_force->get_point_position(1).x;
    peak_sr = longitudinal_force->get_point_position(1).x;

    if (tire_formula_to_use == SIMPLE_PACEJKA) {
        peak_sa = get_peak_pacejka(nominal_load, tire_stiffness, pacejka_C_lat, mu, pacejka_E);
        peak_sr = get_peak_pacejka(nominal_load, tire_stiffness, pacejka_C_long, mu, pacejka_E);
    }
}

mf_vehicle_wheel::~mf_vehicle_wheel(void) {
}

real_t mf_vehicle_wheel::get_peak_pacejka(real_t yload, real_t tire_stiff, real_t C, real_t friction_coeff, real_t E) {
    bool done = false;
    Array unsorted_points = Array();
    Array sorted_points = Array();

    for (int i = 0; i <= 100; i++) {
        unsorted_points.append(pacejka(i * real_t(0.01), tire_stiff, C, friction_coeff, E, yload));
        sorted_points.append(unsorted_points[i]);
        if (i == 99) {
            done = true;
        }
    }

    if (done == true) {
        sorted_points.sort();

        for (int slip = 0; slip <= 100; slip++) {
            if (unsorted_points[slip] == sorted_points[99])
                return slip * real_t(0.01);
        }
    }
    return real_t(0);
}

real_t mf_vehicle_wheel::pacejka(real_t slip, real_t B, real_t C, real_t D, real_t E, real_t yforce) {
    return yforce * D * Math::sin(C * Math::atan(B * slip - E * (B * slip - Math::atan(B * slip - Math::atan(B * slip)))));
}

void mf_vehicle_wheel::process(real_t delta) {
    wheel_mesh->rotate_x(Math::wrapf((double)-spin * (double)delta, (double)0, Math_TAU));
    if (z_vel > 2.0)
        calc_tire_wear(delta, y_force);
}

void mf_vehicle_wheel::physics_process(real_t delta) {
    wheel_body->get_transform().origin = Vector3(0, -1, 0) * spring_curr_length;
}

void mf_vehicle_wheel::calc_tire_wear(real_t delta, real_t yload) {
    real_t larger_slip = std::max(Math::abs(slip_vec.x), Math::abs(slip_vec.y));
    tire_wear += larger_slip * mu * delta * real_t(0.01) * yload * real_t(0.0001) / tire_stiffness;
    tire_wear = std::clamp((double)tire_wear, (double)0, (double)1);
}

real_t mf_vehicle_wheel::calc_rolling_resistance(real_t yload, real_t speed) {
    real_t spd_factor = std::clamp(Math::abs(speed) / real_t(44), real_t(0), real_t(1));
    real_t crr = rol_res_surface_mul * tire_rr_vel_curve->interpolate_baked(spd_factor);
    return crr * yload;
}

real_t mf_vehicle_wheel::apply_forces(real_t opposite_comp, real_t delta, String surface) {
    // local forward velocity
    local_vel = get_global_transform().get_basis().xform_inv((get_global_transform().origin - prev_pos) / delta);
    z_vel = -local_vel.z;
    Vector2 planar_vect = Vector2(local_vel.x, local_vel.z).normalized();
    prev_pos = get_global_transform().origin.normalized();

    // suspension
    if (is_colliding())
        spring_curr_length = get_collision_point().distance_to(get_global_transform().origin) - tire_radius;
    else
        spring_curr_length = spring_length;

    real_t compress = real_t(1) - spring_curr_length / spring_length;
    y_force = spring_stiffness * compress * spring_length;

    if ((compress - prev_compress) >= 0)
        y_force += (bump + wheel_mass) * (compress - prev_compress) * spring_length / delta;
    else
        y_force += rebound * (compress - prev_compress) * spring_length / delta;

    y_force = std::max(real_t(0), y_force);
    prev_compress = compress;

    rolling_resistance = calc_rolling_resistance(y_force, z_vel);

    slip_vec.x = Math::asin(std::clamp(-planar_vect.x, real_t(-1), real_t(1)));
    slip_vec.y = real_t(0.0);

    // slip
    if (z_vel != 0)
        slip_vec.y = (z_vel - spin * tire_radius) / Math::abs(z_vel);
    else {
        if (spin == 0)
            slip_vec.y = real_t(0);
        else
            slip_vec.y = real_t(0.01) * spin;
    }

    /*
     * Spin and net torque
     * real_t net_torque = force_vec.y * tire_radius;
     * if (spin < 5 && braketorque > Math::abs(net_torque))
     *     spin = 0
     * else {
     *     net_torque -= (braketorque + rolling_resistance) * sign(spin);
     *     spin += delta * net_torque / wheel_moment;
     * }
     */

    real_t slip_ratio = slip_vec.y;
    real_t slip_angle = slip_vec.x;

    real_t normalized_sr = slip_ratio / peak_sr;
    real_t normalized_sa = slip_angle / peak_sa;
    real_t resultant_slip = Math::sqrt(Math::pow(normalized_sr, real_t(2) + Math::pow(normalized_sa, real_t(2))));

    real_t sr_modified = resultant_slip * peak_sr;
    real_t sa_modified = resultant_slip * peak_sa;

    real_t x_force = real_t(0);
    real_t z_force = real_t(0);

    if (tire_formula_to_use == CURVE_BASED_FORMULA) {
        x_force = tire_force_vol2(Math::abs(sa_modified), y_force, lateral_force) * signum(slip_vec.x);
        z_force = tire_force_vol2(Math::abs(sr_modified), y_force, longitudinal_force) * signum(slip_vec.y);
    } else if (tire_formula_to_use == SIMPLE_PACEJKA) {
        x_force = pacejka(Math::abs(sa_modified), tire_stiffness, pacejka_C_lat, mu, pacejka_E, y_force) * signum(slip_vec.x);
        z_force = pacejka(Math::abs(sr_modified), tire_stiffness, pacejka_C_long, mu, pacejka_E, y_force) * signum(slip_vec.y);
    }

    if (resultant_slip != 0) {
        force_vec.x = x_force * Math::abs(normalized_sa / resultant_slip);
        force_vec.y = z_force * Math::abs(normalized_sr / resultant_slip);
    } else {
        x_force = real_t(0);
        z_force = real_t(0);

        force_vec.x = x_force;
        force_vec.y = z_force;
    }

    if (tire_formula_to_use == BRUSH_TIRE_FORMULA) {
        force_vec = brush_formula(slip_vec, y_force);    
    }

    if (is_colliding()) {
        Vector3 contact = get_collision_point() - body->get_global_transform().origin;
        Vector3 normal = get_collision_normal();
        real_t wear_mu = tire_wear_mu_curve->interpolate_baked(tire_wear);

        if (surface.empty() == false) {
            if (surface == "Tarmac") {
                mu = real_t(1.0) * tire_mu * wear_mu;
                rol_res_surface_mul = real_t(0.01);
            } else if (surface == "Grass") {
                mu = real_t(0.55) * tire_mu * wear_mu;
                rol_res_surface_mul = real_t(0.025);
            } else if (surface == "Gravel") {
                mu = real_t(0.6) * tire_mu * wear_mu;
                rol_res_surface_mul = real_t(0.03);
            } else if (surface == "Snow") {
                mu = real_t(0.4) * tire_mu * wear_mu;
                rol_res_surface_mul = real_t(0.035);
            }
        } else
            mu = real_t(1) * tire_mu * wear_mu;

        body->add_force(normal * y_force, contact);
        body->add_force(get_global_transform().basis.elements[0] * force_vec.x, contact); // basis.x
        body->add_force(get_global_transform().basis.elements[2] * force_vec.y, contact); // basis.z

        if (compress != 0) {
            compress = real_t(1) - (spring_curr_length / spring_length);
            y_force += anti_roll * (compress - opposite_comp);
        }
        return compress;
    } else
        spin -= signum(spin) * delta * real_t(2) / wheel_moment;

    return real_t(0);
}

real_t mf_vehicle_wheel::apply_torque(real_t drive, real_t drive_inertia, real_t brake_torque, real_t delta) {
    real_t prev_spin = spin;
    real_t net_torque = force_vec.y * tire_radius;
    net_torque += drive;
    if (spin < 5 && brake_torque > Math::abs(net_torque)) {
        spin = 0;
    } else {
        net_torque -= (brake_torque + rolling_resistance) * signum(spin);
        spin += delta * net_torque / (wheel_moment + drive_inertia);
    }

    if (drive * delta == 0)
        return 0.5;

    return (spin - prev_spin) * (wheel_moment + drive_inertia) / (drive * delta);
}

void mf_vehicle_wheel::apply_solid_axle_spin(real_t axlespin) {
    spin = axlespin;
}

real_t mf_vehicle_wheel::tire_force_vol2(real_t slip, real_t normal_load, Ref<Curve> tire_curve) {
    real_t friction = normal_load * mu;
    return tire_curve->interpolate_baked(Math::abs(slip)) * friction * signum(slip);
}

Vector2 mf_vehicle_wheel::brush_formula(Vector2 slip, real_t yforce) {
    real_t stiffness = real_t(500000) * tire_stiffness * Math::pow(brush_contact_patch, real_t(2));
    real_t friction = mu * yforce;
    real_t deflect = Math::sqrt(Math::pow(stiffness * slip.y, real_t(2)) + Math::pow(stiffness * Math::tan(slip.x), real_t(2)));

    if (deflect == 0)
        return Vector2(0, 0);
    else {
        Vector2 vect = Vector2(0, 0);

        if (deflect <= real_t(0.5) * friction * (real_t(1) - slip.y)) {
            vect.y = stiffness * -slip.y / (real_t(1) - slip.y);
            vect.x = stiffness * Math::tan(slip.x) / (real_t(1) - slip.y);
        } else {
            real_t brushy = (real_t(1) - friction * (real_t(1) - slip_vec.y) / (real_t(4) * deflect)) / deflect;
            vect.y = -friction * stiffness * -slip.y * brushy;
            vect.x = friction * stiffness * Math::tan(slip.x) * brushy;
        }
        return vect;
    }
    return Vector2(0, 0);
}

String mf_vehicle_wheel::get_configuration_warning() const {
    String warning = Spatial::get_configuration_warning();
    if (!Object::cast_to<mf_vehicle_body>(get_parent())) {
        if (warning != String()) {
            warning += "\n\n";
        }
        warning+= TTR("mf_vehicle_wheel serves to provide a wheel/tire system to mf_vehicle_body.  Please use it as a child of a mf_vehicle_body.");
    }
    return warning;
}

void mf_vehicle_wheel::_notification(int p_what) {
    switch(p_what) {
        case NOTIFICATION_ENTER_TREE:
            {
                mf_vehicle_body *cb = Object::cast_to<mf_vehicle_body>(get_parent());

                if (!cb)
                    break;

                body = cb;
                cb->wheel_data.push_back(this);
                cb->wheel_list[get_name()] = cb->wheel_data.size() -1;
                if (get_name() == String("wheel_front_left"))
                    cb->set_wheel_radius(tire_radius);

                if (!collider) {
                    Node *collider_node = get_node(NodePath("wheel_body/CollisionShape"));
                    if (collider_node != NULL)
                        //collider = collider_node->cast_to<CollisionShape>();
                        collider = Object::cast_to<CollisionShape>(collider_node);
                }

                if (wheel_body == nullptr) {
                    Node *kinematic_node = get_node(NodePath("wheel_body"));
                    if (kinematic_node == nullptr)
                        wheel_body = Object::cast_to<KinematicBody>(kinematic_node);
                }

                if (wheel_mesh == nullptr) {
                    Node *mesh_instance = get_node(NodePath("wheel_body/wheel_mesh"));
                    if (mesh_instance == nullptr)
                        wheel_mesh = Object::cast_to<MeshInstance>(mesh_instance);
                }
            }
        break;

        case NOTIFICATION_EXIT_TREE:
            {
                if (body == nullptr)
                    break;

                body->wheel_data.erase(this);
                body = nullptr;
            }
        break;
    }
}

void mf_vehicle_wheel::steer(real_t input, real_t max_steer) {
    rotate_y(max_steer * (input + (real_t(1) - Math::cos(input * real_t(0.5) * Math_PI)) * ackermann));
}

void mf_vehicle_wheel::_bind_methods(void) {
    ClassDB::bind_method(D_METHOD("init"), &mf_vehicle_wheel::init);
    ClassDB::bind_method(D_METHOD("process", "delta"), &mf_vehicle_wheel::process);
    ClassDB::bind_method(D_METHOD("physics_process", "delta"), &mf_vehicle_wheel::physics_process);
    ClassDB::bind_method(D_METHOD("get_peak_pacejka", "yload", "tire_stiff", "C", "friction_coeff", "E"), &mf_vehicle_wheel::get_peak_pacejka);
    ClassDB::bind_method(D_METHOD("calc_tire_wear", "delta", "yload"), &mf_vehicle_wheel::calc_tire_wear);
    // ClassDB::bind_method(D_METHOD("") &mf_vehicle_wheel::);
    ClassDB::bind_method(D_METHOD("calc_rolling_resistance", "yload", "speed"), &mf_vehicle_wheel::calc_rolling_resistance);
    ClassDB::bind_method(D_METHOD("apply_forces", "opposite_comp", "delta", "surface"), &mf_vehicle_wheel::apply_forces);
    ClassDB::bind_method(D_METHOD("apply_torque", "drive", "drive_inertia", "brake_torque", "delta"), &mf_vehicle_wheel::apply_torque);
    ClassDB::bind_method(D_METHOD("apply_solid_axle_spin", "axle_spin"), &mf_vehicle_wheel::apply_solid_axle_spin);
    ClassDB::bind_method(D_METHOD("tire_force_vol2", "slip", "normal_load", "tire_curve"), &mf_vehicle_wheel::tire_force_vol2);
    ClassDB::bind_method(D_METHOD("pacejka", "slip", "B", "C", "D", "E", "yforce"), &mf_vehicle_wheel::pacejka);
    ClassDB::bind_method(D_METHOD("brush_formula", "slip", "yforce"), &mf_vehicle_wheel::brush_formula);
    ClassDB::bind_method(D_METHOD("steer", "input", "max_steer"), &mf_vehicle_wheel::steer);

    // fuck you setters and getters.  eat a dick.
    ClassDB::bind_method(D_METHOD("set_tire_formula_to_use", "formula"), &mf_vehicle_wheel::set_tire_formula_to_use);
    ClassDB::bind_method(D_METHOD("get_tire_formula_to_use"), &mf_vehicle_wheel::get_tire_formula_to_use);
    ClassDB::bind_method(D_METHOD("set_spring_length", "length"), &mf_vehicle_wheel::set_spring_length);
    ClassDB::bind_method(D_METHOD("get_spring_length"), &mf_vehicle_wheel::get_spring_length);
    ClassDB::bind_method(D_METHOD("set_spring_stiffness", "stiffness"), &mf_vehicle_wheel::set_spring_stiffness);
    ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &mf_vehicle_wheel::get_spring_stiffness);
    ClassDB::bind_method(D_METHOD("set_bump", "amount"), &mf_vehicle_wheel::set_bump);
    ClassDB::bind_method(D_METHOD("get_bump"), &mf_vehicle_wheel::get_bump);
    ClassDB::bind_method(D_METHOD("set_rebound", "amount"), &mf_vehicle_wheel::set_rebound);
    ClassDB::bind_method(D_METHOD("get_rebound"), &mf_vehicle_wheel::get_rebound);;
    ClassDB::bind_method(D_METHOD("set_anti_roll", "amount"), &mf_vehicle_wheel::set_anti_roll);
    ClassDB::bind_method(D_METHOD("get_anti_roll"), &mf_vehicle_wheel::get_anti_roll);
    ClassDB::bind_method(D_METHOD("set_wheel_mass", "mass"), &mf_vehicle_wheel::set_wheel_mass);
    ClassDB::bind_method(D_METHOD("get_wheel_mass"), &mf_vehicle_wheel::get_wheel_mass);
    ClassDB::bind_method(D_METHOD("set_tire_radius", "radius"), &mf_vehicle_wheel::set_tire_radius);
    ClassDB::bind_method(D_METHOD("get_tire_radius"), &mf_vehicle_wheel::get_tire_radius);
    ClassDB::bind_method(D_METHOD("set_tire_width", "width"), &mf_vehicle_wheel::set_tire_width);
    ClassDB::bind_method(D_METHOD("get_tire_width"), &mf_vehicle_wheel::get_tire_width);
    ClassDB::bind_method(D_METHOD("set_ackermann", "amount"), &mf_vehicle_wheel::set_ackermann);
    ClassDB::bind_method(D_METHOD("get_ackermann"), &mf_vehicle_wheel::get_ackermann);
    ClassDB::bind_method(D_METHOD("set_tire_mu", "amount"), &mf_vehicle_wheel::set_tire_mu);
    ClassDB::bind_method(D_METHOD("get_tire_mu"), &mf_vehicle_wheel::get_tire_mu);
    ClassDB::bind_method(D_METHOD("set_tire_wear_mu_curve", "curve"), &mf_vehicle_wheel::set_tire_wear_mu_curve);
    ClassDB::bind_method(D_METHOD("get_tire_wear_mu_curve"), &mf_vehicle_wheel::get_tire_wear_mu_curve);
    ClassDB::bind_method(D_METHOD("set_tire_rr_vel_curve", "curve"), &mf_vehicle_wheel::set_tire_rr_vel_curve);
    ClassDB::bind_method(D_METHOD("get_tire_rr_vel_curve"), &mf_vehicle_wheel::get_tire_rr_vel_curve);
    ClassDB::bind_method(D_METHOD("set_lateral_force", "curve"), &mf_vehicle_wheel::set_lateral_force);
    ClassDB::bind_method(D_METHOD("get_lateral_force"), &mf_vehicle_wheel::get_lateral_force);
    ClassDB::bind_method(D_METHOD("set_longitudinal_force", "curve"), &mf_vehicle_wheel::set_longitudinal_force);
    ClassDB::bind_method(D_METHOD("get_longitudinal_force"), &mf_vehicle_wheel::get_longitudinal_force);
    ClassDB::bind_method(D_METHOD("set_brush_contact_patch", "patch"), &mf_vehicle_wheel::set_brush_contact_patch);
    ClassDB::bind_method(D_METHOD("get_brush_contact_patch"), &mf_vehicle_wheel::get_brush_contact_patch);
    ClassDB::bind_method(D_METHOD("set_tire_stiffness", "stiffness"), &mf_vehicle_wheel::set_tire_stiffness);
    ClassDB::bind_method(D_METHOD("get_tire_stiffness"), &mf_vehicle_wheel::get_tire_stiffness);
    ClassDB::bind_method(D_METHOD("set_pacejka_C_long", "c_long"), &mf_vehicle_wheel::set_pacejka_C_long);
    ClassDB::bind_method(D_METHOD("get_pacejka_C_long"), &mf_vehicle_wheel::get_pacejka_C_long);
    ClassDB::bind_method(D_METHOD("set_pacejka_C_lat", "latitude"), &mf_vehicle_wheel::set_pacejka_C_lat);
    ClassDB::bind_method(D_METHOD("get_pacejka_C_lat"), &mf_vehicle_wheel::get_pacejka_C_lat);
    ClassDB::bind_method(D_METHOD("set_pacejka_E", "e"), &mf_vehicle_wheel::set_pacejka_E);
    ClassDB::bind_method(D_METHOD("get_pacejka_E"), &mf_vehicle_wheel::get_pacejka_E);
    ClassDB::bind_method(D_METHOD("set_peak_sr", "slip_ratio"), &mf_vehicle_wheel::set_peak_sr);
    ClassDB::bind_method(D_METHOD("get_peak_sr"), &mf_vehicle_wheel::get_peak_sr);
    ClassDB::bind_method(D_METHOD("set_peak_sa", "slip_angle"), &mf_vehicle_wheel::set_peak_sa);
    ClassDB::bind_method(D_METHOD("get_peak_sa"), &mf_vehicle_wheel::get_peak_sa);
    ClassDB::bind_method(D_METHOD("set_tire_wear", "wear"), &mf_vehicle_wheel::set_tire_wear);
    ClassDB::bind_method(D_METHOD("get_tire_wear"), &mf_vehicle_wheel::get_tire_wear);
    ClassDB::bind_method(D_METHOD("set_mu", "wheel_mu"), &mf_vehicle_wheel::set_mu);
    ClassDB::bind_method(D_METHOD("get_mu"), &mf_vehicle_wheel::get_mu);
    ClassDB::bind_method(D_METHOD("set_y_force", "force"), &mf_vehicle_wheel::set_y_force);
    ClassDB::bind_method(D_METHOD("get_y_force"), &mf_vehicle_wheel::get_y_force);
    ClassDB::bind_method(D_METHOD("set_wheel_moment", "moment"), &mf_vehicle_wheel::set_wheel_moment);
    ClassDB::bind_method(D_METHOD("get_wheel_moment"), &mf_vehicle_wheel::get_wheel_moment);
    ClassDB::bind_method(D_METHOD("set_spin", "amount"), &mf_vehicle_wheel::set_spin);
    ClassDB::bind_method(D_METHOD("get_spin"), &mf_vehicle_wheel::get_spin);
    ClassDB::bind_method(D_METHOD("set_z_vel", "velocity"), &mf_vehicle_wheel::set_z_vel);
    ClassDB::bind_method(D_METHOD("get_z_vel"), &mf_vehicle_wheel::get_z_vel);
    ClassDB::bind_method(D_METHOD("set_local_vel", "velocity"), &mf_vehicle_wheel::set_local_vel);
    ClassDB::bind_method(D_METHOD("get_local_vel"), &mf_vehicle_wheel::get_local_vel);
    ClassDB::bind_method(D_METHOD("set_rolling_resistance", "resistance"), &mf_vehicle_wheel::set_rolling_resistance);
    ClassDB::bind_method(D_METHOD("get_rolling_resistance"), &mf_vehicle_wheel::get_rolling_resistance);
    ClassDB::bind_method(D_METHOD("set_rol_res_surface_mul", "amount"), &mf_vehicle_wheel::set_rol_res_surface_mul);
    ClassDB::bind_method(D_METHOD("get_rol_res_surface_mul"), &mf_vehicle_wheel::get_rol_res_surface_mul);
    ClassDB::bind_method(D_METHOD("set_force_vec", "force"), &mf_vehicle_wheel::set_force_vec);
    ClassDB::bind_method(D_METHOD("get_force_vec"), &mf_vehicle_wheel::get_force_vec);
    ClassDB::bind_method(D_METHOD("set_slip_vec", "vec"), &mf_vehicle_wheel::set_slip_vec);
    ClassDB::bind_method(D_METHOD("get_slip_vec"), &mf_vehicle_wheel::get_slip_vec);
    ClassDB::bind_method(D_METHOD("set_prev_pos", "pos"), &mf_vehicle_wheel::set_prev_pos);
    ClassDB::bind_method(D_METHOD("get_prev_pos"), &mf_vehicle_wheel::get_prev_pos);
    ClassDB::bind_method(D_METHOD("set_prev_compress", "amount"), &mf_vehicle_wheel::set_prev_compress);
    ClassDB::bind_method(D_METHOD("get_prev_compress"), &mf_vehicle_wheel::get_prev_compress);
    ClassDB::bind_method(D_METHOD("set_spring_curr_length", "length"), &mf_vehicle_wheel::set_spring_curr_length);
    ClassDB::bind_method(D_METHOD("get_spring_curr_length"), &mf_vehicle_wheel::get_spring_curr_length);

    // Properties
    // ADD_PROPERTY(PropertyInfo(Variant::, ""), "set_", "get_");                                                                                                                                                     
    // ADD_PROPERTYI(PropertyInfo(Variant::OBJECT, "anim_offset_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_param_curve", "get_param_curve", PARAM_ANIM_OFFSET);
    // ADD_PROPERTY(PropertyInfo(Variant::, "", PROPERTY_HINT_ENUM, "1,2,3"), "set_", "get_");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "tire_formula_to_use", PROPERTY_HINT_ENUM, "Simple Pacejka,Brush Tire Formula,Curve Based Formula"), "set_tire_formula_to_use", "get_tire_formula_to_use");
    ADD_PROPERTY_DEFAULT("tire_formula_to_use", CURVE_BASED_FORMULA);
    ADD_GROUP("Suspension", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "spring_length"), "set_spring_length", "get_spring_length");
    ADD_PROPERTY_DEFAULT("spring_length", 0.2);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "spring_stiffness"), "set_spring_stiffness", "get_spring_stiffness");
    ADD_PROPERTY_DEFAULT("spring_stiffness", 20000);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "bump"), "set_bump", "get_bump");
    ADD_PROPERTY_DEFAULT("bump", 5000);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "rebound"), "set_rebound", "get_rebound");
    ADD_PROPERTY_DEFAULT("rebound", 3000);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "anti_roll"), "set_anti_roll", "get_anti_roll");
    ADD_PROPERTY_DEFAULT("anti_roll", 0);

    ADD_GROUP("Tire Properties", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "wheel_mass"), "set_wheel_mass", "get_wheel_mass");
    ADD_PROPERTY_DEFAULT("wheel_mass", 15);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "tire_radius"), "set_tire_radius", "get_tire_radius");
    ADD_PROPERTY_DEFAULT("tire_radius", 0.3);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "tire_width"), "set_tire_width", "get_tire_width");
    ADD_PROPERTY_DEFAULT("tire_width", 0.2);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "ackermann"), "set_ackermann", "get_ackermann");
    ADD_PROPERTY_DEFAULT("ackermann", 0.15);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "tire_mu"), "set_tire_mu", "get_tire_mu");
    ADD_PROPERTY_DEFAULT("tire_mu", 0.9);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "tire_wear_mu_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_tire_wear_mu_curve", "get_tire_wear_mu_curve");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "tire_rr_vel_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_tire_rr_vel_curve", "get_tire_rr_vel_curve");

    ADD_GROUP("Curve Tire Formula", "");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lateral_force", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_lateral_force", "get_lateral_force");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "longitudinal_force", PROPERTY_HINT_RESOURCE_TYPE, "Curve"), "set_longitudinal_force", "get_longitudinal_force");

    ADD_GROUP("Brush Tire Formula", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "brush_contact_patch"), "set_brush_contact_patch", "get_brush_contact_patch");
    ADD_PROPERTY_DEFAULT("brush_contact_patch", 0.2);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "tire_stiffness"), "set_tire_stiffness", "get_tire_stiffness");
    ADD_PROPERTY_DEFAULT("tire_stiffness", 10);

    ADD_GROUP("Pacejka Tire Formula", "");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "pacejka_C_long"), "set_pacejka_C_long", "get_pacejka_C_long");
    ADD_PROPERTY_DEFAULT("pacejka_C_long", 1.65);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "pacejka_C_lat"), "set_pacejka_C_lat", "get_pacejka_C_lat");
    ADD_PROPERTY_DEFAULT("pacejka_C_lat", 1.35);
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "pacejka_E"), "set_pacejka_E", "get_pacejka_E");
    ADD_PROPERTY_DEFAULT("pacejka_E", 0);

    BIND_ENUM_CONSTANT(SIMPLE_PACEJKA);
    BIND_ENUM_CONSTANT(BRUSH_TIRE_FORMULA);
    BIND_ENUM_CONSTANT(CURVE_BASED_FORMULA);
}

void mf_vehicle_wheel::set_tire_formula_to_use(TIRE_FORMULAS formula) {
    tire_formula_to_use = formula;
}

int mf_vehicle_wheel::get_tire_formula_to_use(void) {
    return tire_formula_to_use;
}

// fuck my life...
void mf_vehicle_wheel::set_spring_length(real_t length) {
    spring_length = length;
}

real_t mf_vehicle_wheel::get_spring_length(void) {
    return spring_length;
}

void mf_vehicle_wheel::set_spring_stiffness(real_t stiffness) {
    spring_stiffness = stiffness;
}

real_t mf_vehicle_wheel::get_spring_stiffness(void) {
    return spring_stiffness;
}

void mf_vehicle_wheel::set_bump(real_t amount) {
    bump = amount;
}

real_t mf_vehicle_wheel::get_bump(void) {
    return bump;
}

void mf_vehicle_wheel::set_rebound(real_t amount) {
    rebound = amount;
}

real_t mf_vehicle_wheel::get_rebound(void) {
    return rebound;
}

void mf_vehicle_wheel::set_anti_roll(real_t amount) {
    anti_roll = amount;
}

real_t mf_vehicle_wheel::get_anti_roll(void) {
    return anti_roll;
}

void mf_vehicle_wheel::set_wheel_mass(real_t mass) {
    wheel_mass = mass;
}

real_t mf_vehicle_wheel::get_wheel_mass(void) {
    return wheel_mass;
}

void mf_vehicle_wheel::set_tire_radius(real_t radius) {
    tire_radius = radius;
    update_gizmo();
}

real_t mf_vehicle_wheel::get_tire_radius(void) {
    return tire_radius;
}

void mf_vehicle_wheel::set_tire_width(real_t width) {
    tire_width = width;
}

real_t mf_vehicle_wheel::get_tire_width(void) {
    return tire_width;
}

void mf_vehicle_wheel::set_ackermann(real_t amount) {
    ackermann = amount;
}

real_t mf_vehicle_wheel::get_ackermann(void) {
    return ackermann;
}

void mf_vehicle_wheel::set_tire_mu(real_t amount) {
    tire_mu = amount;
}

real_t mf_vehicle_wheel::get_tire_mu(void) {
    return tire_mu;
}

void mf_vehicle_wheel::set_tire_wear_mu_curve(Ref<Curve> curve) {
    tire_wear_mu_curve = curve;
}

Ref<Curve> mf_vehicle_wheel::get_tire_wear_mu_curve(void) {
    return tire_wear_mu_curve;
}

void mf_vehicle_wheel::set_tire_rr_vel_curve(Ref<Curve> curve) {
    tire_rr_vel_curve = curve;
}

Ref<Curve> mf_vehicle_wheel::get_tire_rr_vel_curve(void) {
    return tire_rr_vel_curve;
}

void mf_vehicle_wheel::set_lateral_force(Ref<Curve> curve) {
    lateral_force = curve;
}

Ref<Curve> mf_vehicle_wheel::get_lateral_force(void) {
    return lateral_force;
}

void mf_vehicle_wheel::set_longitudinal_force(Ref<Curve> curve) {
    longitudinal_force = curve;
}

Ref<Curve> mf_vehicle_wheel::get_longitudinal_force(void) {
    return longitudinal_force;
}

void mf_vehicle_wheel::set_brush_contact_patch(real_t patch) {
    brush_contact_patch = patch;
}

real_t mf_vehicle_wheel::get_brush_contact_patch(void) {
    return brush_contact_patch;
}

void mf_vehicle_wheel::set_tire_stiffness(real_t stiffness) {
    tire_stiffness = stiffness;
}

real_t mf_vehicle_wheel::get_tire_stiffness(void) {
    return tire_stiffness;
}

void mf_vehicle_wheel::set_pacejka_C_long(real_t c_long) {
    pacejka_C_long = c_long;
}

real_t mf_vehicle_wheel::get_pacejka_C_long(void) {
    return pacejka_C_long;
}

void mf_vehicle_wheel::set_pacejka_C_lat(real_t latitude) {
    pacejka_C_lat = latitude;
}

real_t mf_vehicle_wheel::get_pacejka_C_lat(void) {
    return pacejka_C_lat;
}

void mf_vehicle_wheel::set_pacejka_E(real_t e) {
    pacejka_E = e;
}

real_t mf_vehicle_wheel::get_pacejka_E(void) {
    return pacejka_E;
}

void mf_vehicle_wheel::set_peak_sr(real_t slip_ratio) {
    peak_sr = slip_ratio;
}

real_t mf_vehicle_wheel::get_peak_sr(void) {
    return peak_sr;
}

void mf_vehicle_wheel::set_peak_sa(real_t slip_angle) {
    peak_sa = slip_angle;
}

real_t mf_vehicle_wheel::get_peak_sa(void) {
    return peak_sa;
}

void mf_vehicle_wheel::set_tire_wear(real_t wear) {
    tire_wear = wear;
}

real_t mf_vehicle_wheel::get_tire_wear(void) {
    return tire_wear;
}

void mf_vehicle_wheel::set_mu(real_t wheel_mu) {
    mu = wheel_mu;
}

real_t mf_vehicle_wheel::get_mu(void) {
    return mu;
}

void mf_vehicle_wheel::set_y_force(real_t force) {
    y_force = force;
}

real_t mf_vehicle_wheel::get_y_force(void) {
    return y_force;
}

void mf_vehicle_wheel::set_wheel_moment(real_t moment) {
    wheel_moment = moment;
}

real_t mf_vehicle_wheel::get_wheel_moment(void) {
    return wheel_moment;
}

void mf_vehicle_wheel::set_spin(real_t amount) {
    spin = amount;
}

real_t mf_vehicle_wheel::get_spin(void) {
    return spin;
}

void mf_vehicle_wheel::set_z_vel(real_t velocity) {
    z_vel = velocity;
}

real_t mf_vehicle_wheel::get_z_vel(void) {
    return z_vel;
}

void mf_vehicle_wheel::set_local_vel(Vector3 velocity) {
    local_vel = velocity;
}

Vector3 mf_vehicle_wheel::get_local_vel(void) {
    return local_vel;
}

void mf_vehicle_wheel::set_rolling_resistance(real_t resistance) {
    rolling_resistance = resistance;
}

real_t mf_vehicle_wheel::get_rolling_resistance(void) {
    return rolling_resistance;
}

void mf_vehicle_wheel::set_rol_res_surface_mul(real_t amount) {
    rol_res_surface_mul = amount;
}

real_t mf_vehicle_wheel::get_rol_res_surface_mul(void) {
    return rol_res_surface_mul;
}

void mf_vehicle_wheel::set_force_vec(Vector2 force) {
    force_vec = force;
}

Vector2 mf_vehicle_wheel::get_force_vec(void) {
    return force_vec;
}

void mf_vehicle_wheel::set_slip_vec(Vector2 vec) {
    slip_vec = vec;
}

Vector2 mf_vehicle_wheel::get_slip_vec(void) {
    return slip_vec;
}

void mf_vehicle_wheel::set_prev_pos(Vector3 pos) {
    prev_pos = pos;
}

Vector3 mf_vehicle_wheel::get_prev_pos(void) {
    return prev_pos;
}

void mf_vehicle_wheel::set_prev_compress(real_t amount) {
    prev_compress = amount;
}

real_t mf_vehicle_wheel::get_prev_compress(void) {
    return prev_compress;
}

void mf_vehicle_wheel::set_spring_curr_length(real_t length) {
    spring_curr_length = length;
}

real_t mf_vehicle_wheel::get_spring_curr_length(void) {
    return spring_curr_length;
}

