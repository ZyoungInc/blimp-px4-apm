#include "Plane.h"

#include "airship_controller.h"

namespace {
constexpr uint32_t RADIUS_DWELL_MS = 2000;
constexpr uint32_t POSITION_RECOVERY_MS = 3000;
constexpr uint32_t NO_POSITION_DESCENT_MS = 10000;
constexpr float ALTITUDE_HYSTERESIS_M = 5.0f;
constexpr float ARRIVAL_DECEL_MSS = 0.3f;
constexpr float MAX_BRAKE_RATIO = 0.6f;
constexpr float WIND_FILTER_TC_S = 15.0f;
constexpr float WIND_MIN_SPEED_MS = 0.3f;
constexpr float HOLD_HEADING_DEADBAND_DEG = 25.0f;
constexpr float HOLD_YAW_MAX_PCT = 20.0f;
constexpr float BRAKE_HEADING_LIMIT_DEG = 35.0f;
constexpr float FULL_THRUST_HEADING_ERROR_DEG = 35.0f;
constexpr float LIMITED_THRUST_HEADING_ERROR_DEG = 70.0f;
constexpr float LIMITED_THRUST_RATIO = 0.4f;
constexpr float INTEGRATOR_RESET_SPEED_MS = 1.0f;
constexpr float MANUAL_TILT_DEADBAND = 0.03f;
constexpr float BRAKE_ENTER_THR_PCT = -1.0f;
constexpr float BRAKE_EXIT_THR_PCT = -0.25f;
}

const AP_Param::GroupInfo AirshipController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Airship controller enable
    // @Description: Enables the demonstration airship controller and cross-tail mixer. Reboot after changing.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AirshipController, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: R_IN
    // @DisplayName: Airship inner radius
    // @Description: Radius inside which horizontal recovery stops.
    // @Units: m
    // @Range: 2 100
    // @User: Standard
    AP_GROUPINFO("R_IN", 2, AirshipController, _radius_inner_m, 10.0f),

    // @Param: R_OUT
    // @DisplayName: Airship outer radius
    // @Description: Radius beyond which low-energy position hold starts recovery.
    // @Units: m
    // @Range: 5 500
    // @User: Standard
    AP_GROUPINFO("R_OUT", 3, AirshipController, _radius_outer_m, 35.0f),

    // @Param: ALT_BAND
    // @DisplayName: Airship altitude half-band
    // @Description: Allowed altitude error above and below the target before vertical thrust is requested.
    // @Units: m
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("ALT_BAND", 4, AirshipController, _alt_band_m, 20.0f),

    // @Param: RET_SPD
    // @DisplayName: Airship return ground speed
    // @Description: Target inward GPS ground speed during recovery. This is not airspeed.
    // @Units: m/s
    // @Range: 0.5 10
    // @User: Standard
    AP_GROUPINFO("RET_SPD", 5, AirshipController, _return_speed_ms, 2.5f),

    // @Param: RET_THR
    // @DisplayName: Airship maximum return throttle
    // @Description: Maximum horizontal throttle demand used for return and waypoint flight.
    // @Units: %
    // @Range: 5 100
    // @User: Standard
    AP_GROUPINFO("RET_THR", 6, AirshipController, _return_throttle_pct, 35),

    // @Param: CLIMB_THR
    // @DisplayName: Airship climb throttle
    // @Description: Vertical throttle demand used below the altitude deadband.
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("CLIMB_THR", 7, AirshipController, _climb_throttle_pct, 25),

    // @Param: TILT_FMAX
    // @DisplayName: Airship forward tilt endpoint angle
    // @Description: Measured physical forward angle at SERVO3_MAX. Zero degrees is vertical.
    // @Units: deg
    // @Range: 1 90
    // @User: Standard
    AP_GROUPINFO("TILT_FMAX", 8, AirshipController, _tilt_forward_max_deg, 90.0f),

    // @Param: TILT_BMAX
    // @DisplayName: Airship backward tilt endpoint angle
    // @Description: Measured physical backward angle magnitude at SERVO3_MIN. Zero degrees is vertical.
    // @Units: deg
    // @Range: 1 90
    // @User: Standard
    AP_GROUPINFO("TILT_BMAX", 9, AirshipController, _tilt_backward_max_deg, 45.0f),

    // @Param: TILT_RATE
    // @DisplayName: Airship tilt slew rate
    // @Description: Maximum physical tilt movement rate.
    // @Units: deg/s
    // @Range: 1 90
    // @User: Standard
    AP_GROUPINFO("TILT_RATE", 10, AirshipController, _tilt_rate_dps, 10.0f),

    // @Param: YAW_P
    // @DisplayName: Airship heading proportional gain
    // @Description: Rudder percentage commanded per degree of heading error.
    // @Range: 0.05 5
    // @User: Standard
    AP_GROUPINFO("YAW_P", 11, AirshipController, _yaw_p_pct_deg, 1.0f),

    // @Param: YAW_MAX
    // @DisplayName: Airship maximum rudder
    // @Description: Maximum rudder demand from automatic heading control.
    // @Units: %
    // @Range: 5 100
    // @User: Standard
    AP_GROUPINFO("YAW_MAX", 12, AirshipController, _yaw_max_pct, 50),

    // @Param: MAN_CH
    // @DisplayName: Airship manual tilt channel
    // @Description: One-based RC input channel used for common tilt in MANUAL and FBWA.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("MAN_CH", 13, AirshipController, _manual_channel, 7),

    AP_GROUPEND
};

void AirshipController::latch_enable_at_boot()
{
    if (_enable_latched) {
        return;
    }
    _enabled_at_boot = (_enable.get() != 0);
    _enable_latched = true;
}

AirshipController::AirshipController(Plane &plane_ref) :
    _plane(plane_ref)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AirshipController::initialise(const TargetType type)
{
    _target_type = type;
    _active = true;
    _resume_state = type == TargetType::POSHOLD ? State::HOLD : State::RECOVER;
    _center_valid = false;
    _target_alt_valid = false;
    _climb_active = false;
    _filtered_drift_ne_ms.zero();
    _boundary_since_ms = 0;
    _position_lost_ms = 0;
    _position_valid_ms = 0;
    _last_update_ms = AP_HAL::millis();
    _desired_heading_cd = _plane.ahrs.yaw_sensor;
    _last_hold_heading_cd = _desired_heading_cd;
    _throttle_pct = 0.0f;
    _rudder_cd = 0.0f;
}

void AirshipController::enter_poshold()
{
    initialise(TargetType::POSHOLD);
    _rtl_requested = false;
    _center = _plane.current_loc;
    _target_alt_cm = _plane.current_loc.alt;
    _target_alt_valid = vertical_position_is_valid();
    _center_valid = position_is_valid();
    set_state(_center_valid ? State::HOLD : State::NO_POSITION);
    if (!_center_valid) {
        _position_lost_ms = AP_HAL::millis();
    }
}

void AirshipController::enter_rtl()
{
    initialise(TargetType::RTL);
    _rtl_requested = false;
    recover_center_if_needed();

    if (!_center_valid || !position_is_valid()) {
        _position_lost_ms = AP_HAL::millis();
        set_state(State::NO_POSITION);
        return;
    }

    const float distance_m = _plane.current_loc.get_distance(_center);
    set_state(distance_m <= _radius_inner_m ? State::HOLD : State::RECOVER);
}

void AirshipController::enter_auto_target(const Location &target)
{
    initialise(TargetType::AUTO_WAYPOINT);
    if (target.lat == 0 && target.lng == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airship: waypoint position unavailable");
        request_rtl();
        _position_lost_ms = AP_HAL::millis();
        set_state(State::NO_POSITION);
        return;
    }
    _center = target;
    _target_alt_valid = target.get_alt_cm(Location::AltFrame::ABSOLUTE, _target_alt_cm);
    if (!_target_alt_valid) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airship: waypoint altitude unavailable");
        request_rtl();
    }
    _center_valid = !target.is_zero();
    if (!_center_valid || !position_is_valid()) {
        _position_lost_ms = AP_HAL::millis();
        set_state(State::NO_POSITION);
    } else {
        set_state(State::RECOVER);
    }
}

void AirshipController::leave()
{
    _active = false;
    _target_type = TargetType::NONE;
    _throttle_pct = 0.0f;
    _rudder_cd = 0.0f;
    _tilt_target_deg = 0.0f;
    _rtl_requested = false;
    set_state(State::INACTIVE);
}

bool AirshipController::position_is_valid() const
{
    if (!_plane.have_position) {
        return false;
    }

    nav_filter_status status{};
    if (!_plane.ahrs.get_filter_status(status)) {
        return false;
    }

    return status.flags.horiz_pos_abs &&
           status.flags.horiz_vel &&
           !status.flags.gps_glitching &&
           !status.flags.dead_reckoning;
}

bool AirshipController::vertical_position_is_valid() const
{
    nav_filter_status status{};
    return _plane.ahrs.get_filter_status(status) && status.flags.vert_pos;
}

void AirshipController::recover_center_if_needed()
{
    switch (_target_type) {
    case TargetType::POSHOLD:
        if (_center_valid) {
            break;
        }
        _center = _plane.current_loc;
        if (!_target_alt_valid && vertical_position_is_valid()) {
            _target_alt_cm = _plane.current_loc.alt;
            _target_alt_valid = true;
        }
        _center_valid = true;
        break;
    case TargetType::RTL:
        if (AP::ahrs().home_is_set()) {
            if (!_center_valid) {
                // Capture Home once.  Rally points and later Home changes are
                // intentionally ignored for this RTL instance.
                _center = _plane.home;
                _center_valid = true;
            }
            if (!_target_alt_valid && vertical_position_is_valid()) {
                _target_alt_cm = MAX(_plane.current_loc.alt,
                                     _plane.get_RTL_altitude_cm());
                _target_alt_valid = true;
                _center.set_alt_cm(_target_alt_cm, Location::AltFrame::ABSOLUTE);
                _plane.next_WP_loc = _center;
            }
        }
        break;
    case TargetType::AUTO_WAYPOINT:
        if (!_center_valid) {
            _center_valid = !_center.is_zero();
        }
        break;
    case TargetType::NONE:
        break;
    }
}

void AirshipController::update()
{
    if (!active()) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - _last_update_ms) * 0.001f;
    _last_update_ms = now_ms;
    dt = constrain_float(dt, 0.001f, 0.1f);

    _plane.nav_roll_cd = 0;
    _plane.nav_pitch_cd = 0;

    const bool position_valid = position_is_valid();
    if (!position_valid) {
        _position_valid_ms = 0;
        if (_state != State::NO_POSITION) {
            _resume_state = _state;
            _position_lost_ms = now_ms;
            _position_valid_ms = 0;
            set_state(State::NO_POSITION);
        }
        update_no_position(dt);
        return;
    }

    // This also locks the RTL altitude if vertical position becomes valid
    // after horizontal position/Home did.
    recover_center_if_needed();

    if (_state == State::NO_POSITION) {
        if (!_center_valid) {
            _position_valid_ms = 0;
            update_no_position(dt);
            return;
        }
        if (_position_valid_ms == 0) {
            _position_valid_ms = now_ms;
        }
        if (now_ms - _position_valid_ms < POSITION_RECOVERY_MS) {
            update_no_position(dt);
            return;
        }
        _position_lost_ms = 0;
        _position_valid_ms = 0;
        if (_target_type == TargetType::AUTO_WAYPOINT) {
            set_state(State::RECOVER);
        } else {
            set_state(_resume_state == State::INACTIVE ? State::HOLD : _resume_state);
        }
    }

    if (_target_type == TargetType::POSHOLD &&
        !_target_alt_valid && vertical_position_is_valid()) {
        _target_alt_cm = _plane.current_loc.alt;
        _target_alt_valid = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airship: altitude hold restored");
    }

    const Vector2f ground_velocity_ne_ms = _plane.ahrs.groundspeed_vector();
    const float ground_speed_ms = ground_velocity_ne_ms.length();
    reset_integrators_if_slow(ground_speed_ms);

    const float distance_m = _plane.current_loc.get_distance(_center);
    if (_target_type != TargetType::AUTO_WAYPOINT) {
        if (_state == State::HOLD) {
            if (distance_m > _radius_outer_m) {
                if (_boundary_since_ms == 0) {
                    _boundary_since_ms = now_ms;
                } else if (now_ms - _boundary_since_ms >= RADIUS_DWELL_MS) {
                    _boundary_since_ms = 0;
                    set_state(State::RECOVER);
                }
            } else {
                _boundary_since_ms = 0;
            }
        } else if (_state == State::RECOVER || _state == State::BRAKE) {
            if (distance_m <= _radius_inner_m) {
                // Stop horizontal thrust immediately; dwell only confirms HOLD.
                if (_boundary_since_ms == 0) {
                    _boundary_since_ms = now_ms;
                } else if (now_ms - _boundary_since_ms >= RADIUS_DWELL_MS) {
                    _boundary_since_ms = 0;
                    set_state(State::HOLD);
                }
            } else {
                _boundary_since_ms = 0;
            }
        }
    }

    if (_state == State::HOLD) {
        update_hold(ground_velocity_ne_ms, dt);
    } else {
        update_recovery(ground_velocity_ne_ms, distance_m, dt);
    }
}

void AirshipController::update_no_position(const float dt)
{
    (void)dt;
    _rudder_cd = 0.0f;
    reset_integrators_if_slow(0.0f);

    const bool rc_lost = !rc().has_valid_input();
    if (rc_lost && _position_lost_ms != 0 &&
        AP_HAL::millis() - _position_lost_ms >= NO_POSITION_DESCENT_MS) {
        _throttle_pct = 0.0f;
        slew_tilt(0.0f);
        return;
    }

    const float vertical_pct = vertical_position_is_valid() ? vertical_demand_pct() : 0.0f;
    apply_vector(0.0f, vertical_pct);
}

void AirshipController::update_hold(const Vector2f &ground_velocity_ne_ms, const float dt)
{
    const float alpha = dt / (WIND_FILTER_TC_S + dt);
    _filtered_drift_ne_ms += (ground_velocity_ne_ms - _filtered_drift_ne_ms) * alpha;

    if (_filtered_drift_ne_ms.length() >= WIND_MIN_SPEED_MS) {
        const Vector2f into_wind = -_filtered_drift_ne_ms;
        _last_hold_heading_cd = wrap_360_cd(degrees(into_wind.angle()) * 100.0f);
    }
    _desired_heading_cd = _last_hold_heading_cd;
    update_rudder(_desired_heading_cd,
                  MIN(float(_yaw_max_pct), HOLD_YAW_MAX_PCT),
                  HOLD_HEADING_DEADBAND_DEG);
    apply_vector(0.0f, vertical_demand_pct());
}

void AirshipController::update_recovery(const Vector2f &ground_velocity_ne_ms,
                                        const float distance_m,
                                        const float dt)
{
    (void)dt;
    const float stop_radius_m = (_target_type == TargetType::AUTO_WAYPOINT) ?
                                MAX(float(_plane.g.waypoint_radius), 1.0f) :
                                float(_radius_inner_m);

    if (distance_m <= stop_radius_m && _target_type != TargetType::AUTO_WAYPOINT) {
        update_rudder(_desired_heading_cd, float(_yaw_max_pct), 0.0f);
        apply_vector(0.0f, vertical_demand_pct());
        return;
    }

    _desired_heading_cd = _plane.current_loc.get_bearing_to(_center);
    const float bearing_rad = radians(_desired_heading_cd * 0.01f);
    const Vector2f toward_target(cosf(bearing_rad), sinf(bearing_rad));
    const float inward_speed_ms = ground_velocity_ne_ms * toward_target;
    const float remaining_m = MAX(distance_m - stop_radius_m, 0.0f);
    const float desired_speed_ms = MIN(float(_return_speed_ms),
                                       safe_sqrt(2.0f * ARRIVAL_DECEL_MSS * remaining_m));
    const float speed_scale = MAX(float(_return_speed_ms), 0.1f);
    float horizontal_pct = float(_return_throttle_pct) *
                           constrain_float((desired_speed_ms - inward_speed_ms) / speed_scale,
                                           -MAX_BRAKE_RATIO, 1.0f);

    const float heading_error_deg = fabsf(wrap_180_cd(_desired_heading_cd - _plane.ahrs.yaw_sensor)) * 0.01f;
    if (horizontal_pct < 0.0f && heading_error_deg > BRAKE_HEADING_LIMIT_DEG) {
        horizontal_pct = 0.0f;
    } else if (horizontal_pct > 0.0f && heading_error_deg > FULL_THRUST_HEADING_ERROR_DEG) {
        float thrust_ratio = LIMITED_THRUST_RATIO;
        if (heading_error_deg < LIMITED_THRUST_HEADING_ERROR_DEG) {
            const float blend = (LIMITED_THRUST_HEADING_ERROR_DEG - heading_error_deg) /
                                (LIMITED_THRUST_HEADING_ERROR_DEG - FULL_THRUST_HEADING_ERROR_DEG);
            thrust_ratio = LIMITED_THRUST_RATIO + blend * (1.0f - LIMITED_THRUST_RATIO);
        }
        horizontal_pct *= thrust_ratio;
    }

    if (_state == State::BRAKE) {
        if (horizontal_pct > BRAKE_EXIT_THR_PCT) {
            set_state(State::RECOVER);
        }
    } else if (horizontal_pct < BRAKE_ENTER_THR_PCT) {
        set_state(State::BRAKE);
    }
    update_rudder(_desired_heading_cd, float(_yaw_max_pct), 0.0f);
    apply_vector(horizontal_pct, vertical_demand_pct());
}

float AirshipController::vertical_demand_pct()
{
    if (!_target_alt_valid || !vertical_position_is_valid()) {
        return 0.0f;
    }
    const float altitude_error_m = (_target_alt_cm - _plane.current_loc.alt) * 0.01f;
    const float band_m = MAX(float(_alt_band_m), 1.0f);
    const float release_m = MAX(band_m - ALTITUDE_HYSTERESIS_M, 0.0f);

    if (_climb_active) {
        if (altitude_error_m <= release_m) {
            _climb_active = false;
        }
    } else if (altitude_error_m > band_m) {
        _climb_active = true;
    }

    return _climb_active ? constrain_float(float(_climb_throttle_pct), 0.0f, 100.0f) : 0.0f;
}

void AirshipController::apply_vector(const float horizontal_pct, const float vertical_pct)
{
    const float horizontal = constrain_float(horizontal_pct, -100.0f, 100.0f);
    const float vertical = constrain_float(vertical_pct, 0.0f, 100.0f);
    _throttle_pct = constrain_float(safe_sqrt(sq(horizontal) + sq(vertical)), 0.0f, 100.0f);

    float desired_tilt_deg = 0.0f;
    if (!is_zero(horizontal) || !is_zero(vertical)) {
        desired_tilt_deg = degrees(atan2f(horizontal, vertical));
    }
    desired_tilt_deg = constrain_float(desired_tilt_deg,
                                       -MAX(float(_tilt_backward_max_deg), 0.0f),
                                       MAX(float(_tilt_forward_max_deg), 0.0f));
    slew_tilt(desired_tilt_deg);
}

void AirshipController::update_rudder(const int32_t desired_heading_cd,
                                      const float max_pct,
                                      const float deadband_deg)
{
    float error_deg = wrap_180_cd(desired_heading_cd - _plane.ahrs.yaw_sensor) * 0.01f;
    if (fabsf(error_deg) <= deadband_deg) {
        _rudder_cd = 0.0f;
        return;
    }

    if (deadband_deg > 0.0f) {
        error_deg -= copysignf(deadband_deg, error_deg);
    }
    const float rudder_pct = constrain_float(error_deg * float(_yaw_p_pct_deg), -max_pct, max_pct);
    _rudder_cd = rudder_pct * (SERVO_MAX * 0.01f);
}

void AirshipController::slew_tilt(const float target_deg)
{
    _tilt_target_deg = constrain_float(target_deg,
                                       -MAX(float(_tilt_backward_max_deg), 0.0f),
                                       MAX(float(_tilt_forward_max_deg), 0.0f));
    const uint32_t now_ms = AP_HAL::millis();
    float dt = _last_tilt_update_ms == 0 ? 0.02f : (now_ms - _last_tilt_update_ms) * 0.001f;
    _last_tilt_update_ms = now_ms;
    dt = constrain_float(dt, 0.001f, 0.1f);
    const float max_change_deg = MAX(float(_tilt_rate_dps), 0.0f) * dt;
    _tilt_deg += constrain_float(_tilt_target_deg - _tilt_deg, -max_change_deg, max_change_deg);
}

void AirshipController::manual_tilt_update()
{
    if (!enabled()) {
        return;
    }

    const int8_t channel_number = _manual_channel.get();
    RC_Channel *channel = channel_number >= 1 && channel_number <= 16 ?
                          rc().channel(channel_number - 1) : nullptr;
    if (channel == nullptr || !rc().has_valid_input()) {
        slew_tilt(0.0f);
        return;
    }

    float input;
    if (!channel->norm_input_ignore_trim(input)) {
        slew_tilt(0.0f);
        return;
    }
    input = constrain_float(input, -1.0f, 1.0f);
    const float input_abs = fabsf(input);
    if (input_abs <= MANUAL_TILT_DEADBAND) {
        input = 0.0f;
    } else {
        input = copysignf((input_abs - MANUAL_TILT_DEADBAND) /
                          (1.0f - MANUAL_TILT_DEADBAND), input);
    }
    const float target_deg = input >= 0.0f ?
                             input * float(_tilt_forward_max_deg) :
                             input * float(_tilt_backward_max_deg);
    slew_tilt(target_deg);
}

void AirshipController::reset_to_vertical()
{
    _throttle_pct = 0.0f;
    _rudder_cd = 0.0f;
    slew_tilt(0.0f);
}

float AirshipController::tilt_servo_cd() const
{
    if (_tilt_deg >= 0.0f) {
        return 4500.0f * _tilt_deg / MAX(float(_tilt_forward_max_deg), 1.0f);
    }
    return 4500.0f * _tilt_deg / MAX(float(_tilt_backward_max_deg), 1.0f);
}

bool AirshipController::verify_waypoint(const Location &target) const
{
    if (!active() || _target_type != TargetType::AUTO_WAYPOINT ||
        _state == State::NO_POSITION || !_center_valid || !position_is_valid()) {
        return false;
    }
    const float acceptance_radius_m = MAX(float(_plane.g.waypoint_radius), 1.0f);
    return _plane.current_loc.get_distance(target) <= acceptance_radius_m;
}

bool AirshipController::arming_configuration_valid() const
{
    const AP_Arming::Required required = _plane.arming.arming_required();
    return required == AP_Arming::Required::YES_MIN_PWM ||
           required == AP_Arming::Required::YES_ZERO_PWM;
}

void AirshipController::reset_integrators_if_slow(const float ground_speed_ms)
{
    if (ground_speed_ms >= INTEGRATOR_RESET_SPEED_MS) {
        return;
    }
    _plane.rollController.reset_I();
    _plane.pitchController.reset_I();
    _plane.yawController.reset_I();
}

void AirshipController::set_state(const State state)
{
    if (_state == state) {
        return;
    }
    _state = state;
    GCS_SEND_TEXT(state == State::NO_POSITION ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                  "Airship: %s", state_name(state));
}

const char *AirshipController::state_name(const State state) const
{
    switch (state) {
    case State::INACTIVE:    return "inactive";
    case State::HOLD:        return "hold";
    case State::RECOVER:     return "recover";
    case State::BRAKE:       return "brake";
    case State::NO_POSITION: return "no position";
    }
    return "unknown";
}

bool AirshipController::pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const
{
    if (_enable_latched && ((_enable.get() != 0) != _enabled_at_boot)) {
        hal.util->snprintf(failure_msg, failure_msg_len,
                           "reboot after AIR_ENABLE change");
        return false;
    }

    if (!enabled()) {
        return true;
    }

#if HAL_QUADPLANE_ENABLED
    if (_plane.quadplane.enabled()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set Q_ENABLE 0");
        return false;
    }
#endif

    if (_plane.control_mode != &_plane.mode_manual &&
        _plane.control_mode != &_plane.mode_fbwa) {
        hal.util->snprintf(failure_msg, failure_msg_len, "arm in MANUAL or FBWA");
        return false;
    }

    if (!arming_configuration_valid()) {
        hal.util->snprintf(failure_msg, failure_msg_len,
                           "set ARMING_REQUIRE 1 or 2");
        return false;
    }

    if (_plane.aparm.throttle_min.get() != 0) {
        // A positive THR_MIN would turn HOLD and NO_POSITION's commanded zero
        // back into motor thrust in Plane::apply_throttle_limits().
        hal.util->snprintf(failure_msg, failure_msg_len, "set THR_MIN 0");
        return false;
    }
    if (_plane.g.use_reverse_thrust.get() != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set USE_REV_THRUST 0");
        return false;
    }
    if (_plane.aparm.throttle_max.get() <= 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set THR_MAX above 0");
        return false;
    }

    if (_radius_inner_m <= 0.0f || _radius_outer_m <= _radius_inner_m) {
        hal.util->snprintf(failure_msg, failure_msg_len, "require 0 < AIR_R_IN < AIR_R_OUT");
        return false;
    }
    if (_alt_band_m <= 0.0f || _return_speed_ms <= 0.0f ||
        _return_throttle_pct < 5 || _return_throttle_pct > 100 ||
        _climb_throttle_pct < 0 || _climb_throttle_pct > 100) {
        hal.util->snprintf(failure_msg, failure_msg_len, "invalid AIR speed/throttle");
        return false;
    }
    if (_tilt_forward_max_deg <= 0.0f || _tilt_forward_max_deg > 90.0f ||
        _tilt_backward_max_deg <= 0.0f || _tilt_backward_max_deg > 90.0f ||
        _tilt_rate_dps <= 0.0f || _tilt_rate_dps > 90.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "invalid AIR tilt parameters");
        return false;
    }
    if (_manual_channel < 1 || _manual_channel > 16) {
        hal.util->snprintf(failure_msg, failure_msg_len, "invalid AIR_MAN_CH");
        return false;
    }
    const uint8_t manual_channel = uint8_t(_manual_channel.get());
    if (manual_channel == _plane.channel_roll->ch() ||
        manual_channel == _plane.channel_pitch->ch() ||
        manual_channel == _plane.channel_throttle->ch() ||
        manual_channel == _plane.channel_rudder->ch() ||
        manual_channel == uint8_t(rc().flight_mode_channel_number())) {
        hal.util->snprintf(failure_msg, failure_msg_len,
                           "AIR_MAN_CH conflicts with flight control");
        return false;
    }
    const RC_Channel *rtl_channel = rc().find_channel_for_option(RC_Channel::AUX_FUNC::RTL);
    if (rtl_channel != nullptr && manual_channel == rtl_channel->ch()) {
        hal.util->snprintf(failure_msg, failure_msg_len,
                           "AIR_MAN_CH conflicts with RTL switch");
        return false;
    }
    if (_yaw_p_pct_deg <= 0.0f || _yaw_p_pct_deg > 5.0f ||
        _yaw_max_pct < 5 || _yaw_max_pct > 100) {
        hal.util->snprintf(failure_msg, failure_msg_len, "invalid AIR yaw parameters");
        return false;
    }
    if (_plane.g2.manual_rc_mask.get() != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set MANUAL_RCMASK 0");
        return false;
    }
    if (!is_equal(float(_plane.g.mixing_gain), 0.5f) ||
        _plane.g.mixing_offset.get() != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len,
                           "set MIXING_GAIN 0.5 and OFFSET 0");
        return false;
    }
    if (_plane.g2.rudd_dt_gain.get() != 5) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set RUDD_DT_GAIN 5");
        return false;
    }
#if HAL_WITH_IO_MCU
    if (_plane.g.override_channel.get() != 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "set OVERRIDE_CHAN 0");
        return false;
    }
#endif

    static const SRV_Channel::Function expected_functions[] = {
        SRV_Channel::k_throttleLeft,
        SRV_Channel::k_throttleRight,
        SRV_Channel::k_tiltMotorLeft,
        SRV_Channel::k_elevon_left,
        SRV_Channel::k_vtail_right,
        SRV_Channel::k_elevon_right,
        SRV_Channel::k_vtail_left,
        SRV_Channel::k_none,
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(expected_functions); i++) {
        const SRV_Channel *channel = SRV_Channels::srv_channel(i);
        if (channel == nullptr || channel->get_function() != expected_functions[i]) {
            hal.util->snprintf(failure_msg, failure_msg_len,
                               "set SERVO%u_FUNCTION %u", unsigned(i + 1),
                               unsigned(expected_functions[i]));
            return false;
        }
    }

    for (uint8_t i = ARRAY_SIZE(expected_functions); i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel *channel = SRV_Channels::srv_channel(i);
        if (channel == nullptr) {
            continue;
        }
        const SRV_Channel::Function function = channel->get_function();
        if (function == SRV_Channel::k_throttle ||
            function == SRV_Channel::k_throttleLeft ||
            function == SRV_Channel::k_throttleRight ||
            function == SRV_Channel::k_tiltMotorLeft ||
            function == SRV_Channel::k_elevon_left ||
            function == SRV_Channel::k_elevon_right ||
            function == SRV_Channel::k_vtail_left ||
            function == SRV_Channel::k_vtail_right) {
            hal.util->snprintf(failure_msg, failure_msg_len,
                               "remove duplicate AIR function on SERVO%u",
                               unsigned(i + 1));
            return false;
        }
    }
    return true;
}
