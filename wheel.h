#ifndef WHEEL_H
#define WHEEL_H

#include <core/method_bind_ext.gen.inc>
#include <scene/3d/ray_cast.h>
#include <scene/3d/physics_body.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/collision_shape.h>

class mf_vehicle_body;

class mf_vehicle_wheel:public RayCast {
    GDCLASS(mf_vehicle_wheel, RayCast);

    public:
        mf_vehicle_wheel(void);
        ~mf_vehicle_wheel(void);

        // Tire formula
        enum TIRE_FORMULAS {
            SIMPLE_PACEJKA,
            BRUSH_TIRE_FORMULA,
            CURVE_BASED_FORMULA
        };

        // Naming these the same as the functions they get called im.
        void process(real_t delta);
        void physics_process(real_t delta);

        real_t get_peak_pacejka(real_t yload, real_t tire_stiff, real_t C, real_t friction_coeff, real_t E);
        void calc_tire_wear(real_t delta, real_t yload);
        real_t calc_rolling_resistance(real_t yload, real_t speed);
        real_t apply_forces(real_t opposite_comp, real_t delta, String surface);
        real_t apply_torque(real_t drive, real_t drive_inertia, real_t brake_torque, real_t delta);
        void apply_solid_axle_spin(real_t axle_spin);
        real_t tire_force_vol2(real_t slip, real_t normal_load, Ref<Curve> tire_curve);
        real_t pacejka(real_t slip, real_t B, real_t C, real_t D, real_t E, real_t yforce);
        Vector2 brush_formula(Vector2 slip, real_t yforce);
        void steer(real_t input, real_t max_steer);

//    protected:
        TIRE_FORMULAS tire_formula_to_use = CURVE_BASED_FORMULA;

        // Suspension
        real_t spring_length = real_t(0.2);
        real_t spring_stiffness = real_t(15000);
        real_t bump = real_t(5000);
        real_t rebound = real_t(3000);
        real_t anti_roll = real_t(0.0);

        // Tire
        real_t wheel_mass = real_t(15);
        real_t tire_radius = real_t(0.3);
        real_t tire_width = real_t(0.2);
        real_t ackermann = real_t(0.15);
        real_t tire_mu = real_t(0.9);
        Ref<Curve> tire_wear_mu_curve;
        Ref<Curve> tire_rr_vel_curve;

        // curve tire formula
        Ref<Curve> lateral_force;
        Ref<Curve> longitudinal_force;

        // brush tire formula
        real_t brush_contact_patch = real_t(0.2);
        real_t tire_stiffness = real_t(10);

        // pacejka tire formula
        real_t pacejka_C_long = real_t(1.65);
        real_t pacejka_C_lat = real_t(1.35);
        real_t pacejka_E = real_t(0);

        real_t peak_sr = real_t(0.10); // slip ratio
        real_t peak_sa = real_t(0.10); // slip angle

        real_t tire_wear = real_t(0);

        real_t mu = real_t(1);
        real_t y_force = real_t(0);

        real_t wheel_moment = real_t(0);
        real_t spin = real_t(0);
        real_t z_vel = real_t(0);
        Vector3 local_vel = Vector3(0, 0, 0); // this might actually be a Vector3, we'll find out...

        real_t rolling_resistance = real_t(0);
        real_t rol_res_surface_mul = real_t(0.02);

        Vector2 force_vec = Vector2(0, 0);
        Vector2 slip_vec = Vector2(0, 0);
        Vector3 prev_pos = Vector3(0, 0, 0);

        real_t prev_compress = real_t(0);
        real_t spring_curr_length = real_t(spring_length);

        mf_vehicle_body *body = nullptr;
        CollisionShape collider;
        KinematicBody wheel_body;
        MeshInstance wheel_mesh;

    protected:
        String get_configuration_warning() const;
        void _notification(int p_what);
        static void _bind_methods(void);

    // Trying to keep the main bulk of the class clean.
    public:
        void set_tire_formula_to_use(TIRE_FORMULAS formula);
        int get_tire_formula_to_use(void);
        void set_spring_length(real_t length);
        real_t get_spring_length(void);
        void set_spring_stiffness(real_t stiffness);
        real_t get_spring_stiffness(void);
        void set_bump(real_t amount);
        real_t get_bump(void);
        void set_rebound(real_t amount);
        real_t get_rebound(void);
        void set_anti_roll(real_t amount);
        real_t get_anti_roll(void);
        void set_wheel_mass(real_t mass);
        real_t get_wheel_mass(void);
        void set_tire_radius(real_t radius);
        real_t get_tire_radius(void);
        void set_tire_width(real_t width);
        real_t get_tire_width(void);
        void set_ackermann(real_t amount);
        real_t get_ackermann(void);
        void set_tire_mu(real_t amount);
        real_t get_tire_mu(void);
        void set_tire_wear_mu_curve(Ref<Curve> curve);
        Ref<Curve> get_tire_wear_mu_curve(void);
        void set_tire_rr_vel_curve(Ref<Curve> curve);
        Ref<Curve> get_tire_rr_vel_curve(void);
        void set_lateral_force(Ref<Curve> curve);
        Ref<Curve> get_lateral_force(void);
        void set_longitudinal_force(Ref<Curve> curve);
        Ref<Curve> get_longitudinal_force(void);
        void set_brush_contact_patch(real_t patch);
        real_t get_brush_contact_patch(void);
        void set_tire_stiffness(real_t stiffness);
        real_t get_tire_stiffness(void);
        void set_pacejka_C_long(real_t c_long);
        real_t get_pacejka_C_long(void);
        void set_pacejka_C_lat(real_t latitude);
        real_t get_pacejka_C_lat(void);
        void set_pacejka_E(real_t e);
        real_t get_pacejka_E(void);
        void set_peak_sr(real_t slip_ratio);
        real_t get_peak_sr(void);
        void set_peak_sa(real_t slip_angle);
        real_t get_peak_sa(void);
        void set_tire_wear(real_t wear);
        real_t get_tire_wear(void);
        void set_mu(real_t wheel_mu);
        real_t get_mu(void);
        void set_y_force(real_t force);
        real_t get_y_force(void);
        void set_wheel_moment(real_t moment);
        real_t get_wheel_moment(void);
        void set_spin(real_t amount);
        real_t get_spin(void);
        void set_z_vel(real_t velocity);
        real_t get_z_vel(void);
        void set_local_vel(Vector3 velocity);
        Vector3 get_local_vel(void);
        void set_rolling_resistance(real_t resistance);
        real_t get_rolling_resistance(void);
        void set_rol_res_surface_mul(real_t amount);
        real_t get_rol_res_surface_mul(void);
        void set_force_vec(Vector2 force);
        Vector2 get_force_vec(void);
        void set_slip_vec(Vector2 vec);
        Vector2 get_slip_vec(void);
        void set_prev_pos(Vector3 pos);
        Vector3 get_prev_pos(void);
        void set_prev_compress(real_t amount);
        real_t get_prev_compress(void);
        void set_spring_curr_length(real_t length);
        real_t get_spring_curr_length(void);
};

VARIANT_ENUM_CAST(mf_vehicle_wheel::TIRE_FORMULAS);

#endif // WHEEL_H

