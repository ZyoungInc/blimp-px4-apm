#pragma once

#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

class Plane;

/*
 * Minimal low-energy controller for the demonstration airship.
 *
 * This intentionally uses GPS ground velocity instead of an airspeed
 * estimate.  It is only intended for the calm-weather demonstration setup
 * documented in docs/AIRSHIP_*.md.
 */
class AirshipController {
public:
    explicit AirshipController(Plane &plane_ref);

    static const AP_Param::GroupInfo var_info[];

    // AIR_ENABLE is deliberately sampled once during startup.  Changing the
    // parameter in flight must never swap between the Plane and airship
    // control/output paths without running their normal initialisation.
    void latch_enable_at_boot();
    bool enable_requested() const { return _enable.get() != 0; }
    bool enabled() const { return _enable_latched && _enabled_at_boot; }
    bool active() const { return enabled() && _active; }

    void enter_poshold();
    void enter_rtl();
    void enter_auto_target(const Location &target);
    void leave();

    void request_rtl() { _rtl_requested = true; }
    bool rtl_requested() const { return _rtl_requested; }
    void clear_rtl_request() { _rtl_requested = false; }

    // Called from the automatic mode update loop.
    void update();

    // Called from the servo output loop in pilot-controlled modes.
    void manual_tilt_update();
    void reset_to_vertical();

    bool verify_waypoint(const Location &target) const;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;
    bool arming_configuration_valid() const;

    float throttle_pct() const { return _throttle_pct; }
    float rudder_cd() const { return _rudder_cd; }
    float tilt_deg() const { return _tilt_deg; }
    float tilt_servo_cd() const;

private:
    enum class TargetType : uint8_t {
        NONE,
        POSHOLD,
        RTL,
        AUTO_WAYPOINT,
    };

    enum class State : uint8_t {
        INACTIVE,
        HOLD,
        RECOVER,
        BRAKE,
        NO_POSITION,
    };

    Plane &_plane;

    AP_Int8 _enable;
    AP_Float _radius_inner_m;
    AP_Float _radius_outer_m;
    AP_Float _alt_band_m;
    AP_Float _return_speed_ms;
    AP_Int8 _return_throttle_pct;
    AP_Int8 _climb_throttle_pct;
    AP_Float _tilt_forward_max_deg;
    AP_Float _tilt_backward_max_deg;
    AP_Float _tilt_rate_dps;
    AP_Float _yaw_p_pct_deg;
    AP_Int8 _yaw_max_pct;
    AP_Int8 _manual_channel;

    TargetType _target_type = TargetType::NONE;
    State _state = State::INACTIVE;
    State _resume_state = State::HOLD;
    bool _active = false;
    bool _enable_latched = false;
    bool _enabled_at_boot = false;
    bool _center_valid = false;
    bool _target_alt_valid = false;
    bool _climb_active = false;
    bool _rtl_requested = false;

    Location _center;
    int32_t _target_alt_cm = 0;
    int32_t _desired_heading_cd = 0;
    int32_t _last_hold_heading_cd = 0;

    Vector2f _filtered_drift_ne_ms;
    float _throttle_pct = 0.0f;
    float _rudder_cd = 0.0f;
    float _tilt_deg = 0.0f;
    float _tilt_target_deg = 0.0f;

    uint32_t _last_update_ms = 0;
    uint32_t _last_tilt_update_ms = 0;
    uint32_t _boundary_since_ms = 0;
    uint32_t _position_lost_ms = 0;
    uint32_t _position_valid_ms = 0;

    bool position_is_valid() const;
    bool vertical_position_is_valid() const;
    void initialise(TargetType type);
    void set_state(State state);
    void update_no_position(float dt);
    void update_hold(const Vector2f &ground_velocity_ne_ms, float dt);
    void update_recovery(const Vector2f &ground_velocity_ne_ms, float distance_m, float dt);
    float vertical_demand_pct();
    void apply_vector(float horizontal_pct, float vertical_pct);
    void update_rudder(int32_t desired_heading_cd, float max_pct, float deadband_deg);
    void slew_tilt(float target_deg);
    void reset_integrators_if_slow(float ground_speed_ms);
    void recover_center_if_needed();
    const char *state_name(State state) const;
};
