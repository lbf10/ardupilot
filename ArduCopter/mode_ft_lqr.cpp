#include "Copter.h"

#if MODE_FT_LQR_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */

#ifndef FT_LQR_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define FT_LQR_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define FT_LQR_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define FT_LQR_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f ft_lqr_pos_target_cm;       // position target (used by posvel controller only)
static Vector3f ft_lqr_vel_target_cms;      // velocity target (used by velocity controller and posvel controller)
static uint32_t posvel_update_time_ms;      // system time of last target update to posvel controller (i.e. position and velocity update)
static uint32_t vel_update_time_ms;         // system time of last target update to velocity controller

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float yaw_rate_cds;
    float climb_rate_cms;
    bool use_yaw_rate;
} static ft_lqr_angle_state;

struct Ft_Lqr_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} ft_lqr_limit;

// ft_lqr_init - initialise guided controller
bool ModeFT_LQR::init(bool ignore_checks)
{
    // start in position control mode
    pos_control_start();

    copter.multicontrol.init();
    return true;
}


// do_user_takeoff_start - initialises waypoint controller to implement take-off
bool ModeFT_LQR::do_user_takeoff_start(float final_alt_above_home)
{
    ft_lqr_mode = Guided_TakeOff;

    // initialise wpnav destination
    Location target_loc = copter.current_loc;
    target_loc.set_alt_cm(final_alt_above_home, Location::AltFrame::ABOVE_HOME);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

    return true;
}

// initialise guided mode's position controller
void ModeFT_LQR::pos_control_start()
{
    // set to position control mode
    ft_lqr_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// initialise guided mode's velocity controller
void ModeFT_LQR::vel_control_start()
{
    // set ft_lqr_mode to velocity controller
    ft_lqr_mode = Guided_Velocity;

    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control->init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
void ModeFT_LQR::posvel_control_start()
{
    // set ft_lqr_mode to velocity controller
    ft_lqr_mode = Guided_PosVel;

    pos_control->init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control->set_xy_target(curr_pos.x, curr_pos.y);
    pos_control->set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

bool ModeFT_LQR::is_taking_off() const
{
    return ft_lqr_mode == Guided_TakeOff;
}

// initialise guided mode's angle controller
void ModeFT_LQR::angle_control_start()
{
    // set ft_lqr_mode to velocity controller
    ft_lqr_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    ft_lqr_angle_state.update_time_ms = millis();
    ft_lqr_angle_state.roll_cd = ahrs.roll_sensor;
    ft_lqr_angle_state.pitch_cd = ahrs.pitch_sensor;
    ft_lqr_angle_state.yaw_cd = ahrs.yaw_sensor;
    ft_lqr_angle_state.climb_rate_cms = 0.0f;
    ft_lqr_angle_state.yaw_rate_cds = 0.0f;
    ft_lqr_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// ft_lqr_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeFT_LQR::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
    // ensure we are in position control mode
    if (ft_lqr_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, terrain_alt);

    // log target
    copter.Log_Write_GuidedTarget(ft_lqr_mode, destination, Vector3f());
    return true;
}

bool ModeFT_LQR::get_wp(Location& destination)
{
    if (ft_lqr_mode != Guided_WP) {
        return false;
    }
    return wp_nav->get_oa_wp_destination(destination);
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool ModeFT_LQR::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // ensure we are in position control mode
    if (ft_lqr_mode != Guided_WP) {
        pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // log target
    copter.Log_Write_GuidedTarget(ft_lqr_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// ft_lqr_set_velocity - sets guided mode's target velocity
void ModeFT_LQR::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (ft_lqr_mode != Guided_Velocity) {
        vel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // record velocity target
    ft_lqr_vel_target_cms = velocity;
    vel_update_time_ms = millis();

    // log target
    if (log_request) {
        copter.Log_Write_GuidedTarget(ft_lqr_mode, Vector3f(), velocity);
    }
}

// set guided mode posvel target
bool ModeFT_LQR::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    // check we are in velocity control mode
    if (ft_lqr_mode != Guided_PosVel) {
        posvel_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    posvel_update_time_ms = millis();
    ft_lqr_pos_target_cm = destination;
    ft_lqr_vel_target_cms = velocity;

    copter.pos_control->set_pos_target(ft_lqr_pos_target_cm);

    // log target
    copter.Log_Write_GuidedTarget(ft_lqr_mode, destination, velocity);
    return true;
}

// set guided mode angle target
void ModeFT_LQR::set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (ft_lqr_mode != Guided_Angle) {
        angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(ft_lqr_angle_state.roll_cd, ft_lqr_angle_state.pitch_cd, ft_lqr_angle_state.yaw_cd);
    ft_lqr_angle_state.roll_cd = ToDeg(ft_lqr_angle_state.roll_cd) * 100.0f;
    ft_lqr_angle_state.pitch_cd = ToDeg(ft_lqr_angle_state.pitch_cd) * 100.0f;
    ft_lqr_angle_state.yaw_cd = wrap_180_cd(ToDeg(ft_lqr_angle_state.yaw_cd) * 100.0f);
    ft_lqr_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    ft_lqr_angle_state.use_yaw_rate = use_yaw_rate;

    ft_lqr_angle_state.climb_rate_cms = climb_rate_cms;
    ft_lqr_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !copter.ap.auto_armed && (ft_lqr_angle_state.climb_rate_cms > 0.0f)) {
        copter.set_auto_armed(true);
    }

    // log target
    copter.Log_Write_GuidedTarget(ft_lqr_mode,
                           Vector3f(ft_lqr_angle_state.roll_cd, ft_lqr_angle_state.pitch_cd, ft_lqr_angle_state.yaw_cd),
                           Vector3f(0.0f, 0.0f, ft_lqr_angle_state.climb_rate_cms));
}

// ft_lqr_run - runs the guided controller
// should be called at 100hz or more
void ModeFT_LQR::run()
{
    if(copter.polyNav.isRunning()){
        ft_lqr_mode = Guided_WP;
    }
    
    // call the correct auto controller
    switch (ft_lqr_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case Guided_WP:
        // run position controller       
        {   
            if(copter.polyNav.isRunning()){
                bool returnValue = true;
                // Update polynomial trajectory
                if(!copter.polyNav.updateTrajectory()){returnValue=false;};

                // Update inertial and desired states
                if(!copter.multicontrol.updateStates(copter.polyNav.getDesiredState())){returnValue=false;};

                // Update position control
                if(!copter.multicontrol.positionControl()){returnValue=false;};

                // Calculate attitude reference
                if(!copter.multicontrol.attitudeReference()){returnValue=false;};

                // Update attitude control
                if(!copter.multicontrol.attitudeFTLQRControl()){returnValue=false;};

                // Update control allocation
                if(!copter.multicontrol.controlAllocation()){returnValue=false;};

                if(returnValue){
                    // assign values to motors

                    // cork now, so that all channel outputs happen at once
                    SRV_Channels::cork();

                    float battVoltage = copter.battery.voltage();
                    //float *rotVoltages = copter.multicontrol.desiredRotorVoltages();
                    float *rotSpeeds = copter.multicontrol.desiredRotorSpeeds();
                    float throttle[NUMBER_OF_ROTORS];
                    uint16_t pwm[NUMBER_OF_ROTORS];
                    for(int it=0;it<NUMBER_OF_ROTORS;it++){
                        throttle[it] = abs(rotSpeeds[it]);
                        throttle[it] = 8.327766884267e-7*throttle[it]*throttle[it]+0.0005023351541*throttle[it]+0.1911812180;
                        //throttle[it] = -1.0249277445252e-6*throttle[it]*throttle[it]+0.002658375700044*throttle[it]-0.413078665036508;
                        pwm[it] = (uint16_t) 1000*throttle[it]+1000;

                        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(it);
                        SRV_Channels::set_output_pwm(function, pwm[it]);
                    }
                    
                    copter.multicontrol.dumpfile << " battVoltage:," << battVoltage << " , " << 
                                                    " PWM:," << 
                                                    pwm[0] << " , " <<
                                                    pwm[1] << " , " <<
                                                    pwm[2] << " , " <<
                                                    pwm[3] << " , " <<
                                                    pwm[4] << " , " <<
                                                    pwm[5] << " , " <<
                                                    pwm[6] << " , " <<
                                                    pwm[7] << " , " <<
                                                    " throttle:," << 
                                                    throttle[0] << " , " <<
                                                    throttle[1] << " , " <<
                                                    throttle[2] << " , " <<
                                                    throttle[3] << " , " <<
                                                    throttle[4] << " , " <<
                                                    throttle[5] << " , " <<
                                                    throttle[6] << " , " <<
                                                    throttle[7] << " , " <<
                                                    " rotSpeeds:," << 
                                                    rotSpeeds[0] << " , " <<
                                                    rotSpeeds[1] << " , " <<
                                                    rotSpeeds[2] << " , " <<
                                                    rotSpeeds[3] << " , " <<
                                                    rotSpeeds[4] << " , " <<
                                                    rotSpeeds[5] << " , " <<
                                                    rotSpeeds[6] << " , " <<
                                                    rotSpeeds[7] << " , " <<
                                                    " _ftLQR.gainRotorSpeeds:," << 
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(0) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(1) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(2) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(3) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(4) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(5) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(6) << " , " <<
                                                    copter.multicontrol._ftLQR.gainRotorSpeeds(7) << " , " <<                              
                                                    " time step:," <<  copter.multicontrol.measuredTimeStep() << " , " << 
                                                    " currentPosition:," << 
                                                    copter.multicontrol._currentPosition.x() << " , " << 
                                                    copter.multicontrol._currentPosition.y() << " , " << 
                                                    copter.multicontrol._currentPosition.z() << " , " << 
                                                    " desiredPosition:," << 
                                                    copter.multicontrol._desiredPosition.x() << " , " << 
                                                    copter.multicontrol._desiredPosition.y() << " , " << 
                                                    copter.multicontrol._desiredPosition.z() << " , " << 
                                                    " desiredForce:," << 
                                                    copter.multicontrol._desiredForce.x() << " , " << 
                                                    copter.multicontrol._desiredForce.y() << " , " << 
                                                    copter.multicontrol._desiredForce.z() << " , " << 
                                                    " desiredTorque:," << 
                                                    copter.multicontrol._desiredTorque.x() << " , " << 
                                                    copter.multicontrol._desiredTorque.y() << " , " << 
                                                    copter.multicontrol._desiredTorque.z() << " , " << 
                                                    " desiredAttitude:," << 
                                                    copter.multicontrol._desiredAttitude.w() << " , " <<
                                                    copter.multicontrol._desiredAttitude.x() << " , " << 
                                                    copter.multicontrol._desiredAttitude.y() << " , " << 
                                                    copter.multicontrol._desiredAttitude.z() << " , " << 
                                                    " velFilter.desiredAngularVelocity:," << 
                                                    copter.multicontrol._velFilter.desiredAngularVelocity.x() << " , " <<
                                                    copter.multicontrol._velFilter.desiredAngularVelocity.y() << " , " << 
                                                    copter.multicontrol._velFilter.desiredAngularVelocity.z() << " , " <<  
                                                    " velFilter.desiredAngularAcceleration:," << 
                                                    copter.multicontrol._velFilter.desiredAngularAcceleration.x() << " , " <<
                                                    copter.multicontrol._velFilter.desiredAngularAcceleration.y() << " , " << 
                                                    copter.multicontrol._velFilter.desiredAngularAcceleration.z() << " , " <<  
                                                    " velFilter.desiredGainAngularAcceleration:," << 
                                                    copter.multicontrol._ftLQR.desiredGainAngularAcceleration.x() << " , " <<
                                                    copter.multicontrol._ftLQR.desiredGainAngularAcceleration.y() << " , " << 
                                                    copter.multicontrol._ftLQR.desiredGainAngularAcceleration.z() << " , " <<  
                                                    " currentAttitude (quat ENU):," << 
                                                    copter.multicontrol._currentAttitude.w() << " , " <<
                                                    copter.multicontrol._currentAttitude.x() << " , " << 
                                                    copter.multicontrol._currentAttitude.y() << " , " << 
                                                    copter.multicontrol._currentAttitude.z() << " , " <<  
                                                    " currentAttitude (ahrs euler NED):," << 
                                                    copter.ahrs.get_roll() << " , " <<
                                                    copter.ahrs.get_pitch() << " , " << 
                                                    copter.ahrs.get_yaw() << " , " << 
                                                    " currentAngularVelocity:," << 
                                                    copter.multicontrol._currentAngularVelocity.x() << " , " << 
                                                    copter.multicontrol._currentAngularVelocity.y() << " , " << 
                                                    copter.multicontrol._currentAngularVelocity.z() << " , " <<  
                                                    " qe:, " << 
                                                    copter.multicontrol.auxQuaternion.w() << " , " <<
                                                    copter.multicontrol.auxQuaternion.x() << " , " << 
                                                    copter.multicontrol.auxQuaternion.y() << " , " << 
                                                    copter.multicontrol.auxQuaternion.z() << std::endl;

                    // push all channels
                    SRV_Channels::push();
                }
            }
            else{
                // Update inertial states
                if(!copter.multicontrol.updateStates(copter.polyNav.getDesiredState())){};
                pos_control_run();
            }
            break;
        }      
        /*{
            pos_control_run();
            break;
        }*/

    case Guided_Velocity:
        // run velocity controller
        vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        angle_control_run();
        break;
    }
 }

// ft_lqr_takeoff_run - takeoff in guided mode
//      called by ft_lqr_run at 100hz or more
void ModeFT_LQR::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();

        // switch to position control mode but maintain current target
        const Vector3f& target = wp_nav->get_wp_destination();
        set_destination(target);
    }
}

// ft_lqr_pos_control_run - runs the guided position controller
// called from ft_lqr_run
void ModeFT_LQR::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// ft_lqr_vel_control_run - runs the guided velocity controller
// called from ft_lqr_run
void ModeFT_LQR::vel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > FT_LQR_POSVEL_TIMEOUT_MS) {
        if (!pos_control->get_desired_velocity().is_zero()) {
            set_desired_velocity_with_accel_and_fence_limits(Vector3f(0.0f, 0.0f, 0.0f));
        }
        if (auto_yaw.mode() == AUTO_YAW_RATE) {
            auto_yaw.set_rate(0.0f);
        }
    } else {
        set_desired_velocity_with_accel_and_fence_limits(ft_lqr_vel_target_cms);
    }

    // call velocity controller which includes z axis controller
    pos_control->update_vel_controller_xyz();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.yaw(), true);
    }
}

// ft_lqr_posvel_control_run - runs the guided spline controller
// called from ft_lqr_run
void ModeFT_LQR::posvel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > FT_LQR_POSVEL_TIMEOUT_MS) {
        ft_lqr_vel_target_cms.zero();
        if (auto_yaw.mode() == AUTO_YAW_RATE) {
            auto_yaw.set_rate(0.0f);
        }
    }

    // calculate dt
    float dt = pos_control->time_since_last_xy_update();

    // sanity check dt
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // advance position target using velocity target
    ft_lqr_pos_target_cm += ft_lqr_vel_target_cms * dt;

    // send position and velocity targets to position controller
    pos_control->set_pos_target(ft_lqr_pos_target_cm);
    pos_control->set_desired_velocity_xy(ft_lqr_vel_target_cms.x, ft_lqr_vel_target_cms.y);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position-velocity controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.yaw(), true);
    }
}

// ft_lqr_angle_control_run - runs the guided angle controller
// called from ft_lqr_run
void ModeFT_LQR::angle_control_run()
{
    // constrain desired lean angles
    float roll_in = ft_lqr_angle_state.roll_cd;
    float pitch_in = ft_lqr_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), copter.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(ft_lqr_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(ft_lqr_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(ft_lqr_angle_state.climb_rate_cms, -fabsf(wp_nav->get_default_speed_down()), wp_nav->get_default_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - ft_lqr_angle_state.update_time_ms > FT_LQR_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && (ft_lqr_angle_state.climb_rate_cms <= 0.0f))) {
        make_safe_spool_down();
        return;
    }

    // TODO: use get_alt_hold_state
    // landed with positive desired climb rate, takeoff
    if (copter.ap.land_complete && (ft_lqr_angle_state.climb_rate_cms > 0.0f)) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
            set_throttle_takeoff();
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    if (ft_lqr_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
}

// helper function to update position controller's desired velocity while respecting acceleration limits
void ModeFT_LQR::set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des)
{
    // get current desired velocity
    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    // get change in desired velocity
    Vector3f vel_delta = vel_des - curr_vel_des;

    // limit xy change
    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_max_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    // limit z change
    float vel_delta_z_max = G_Dt * pos_control->get_max_accel_z();
    curr_vel_des.z += constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);

#if AC_AVOID_ENABLED
    // limit the velocity to prevent fence violations
    copter.avoid.adjust_velocity(pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy(), curr_vel_des, G_Dt);
    // get avoidance adjusted climb rate
    curr_vel_des.z = get_avoidance_adjusted_climbrate(curr_vel_des.z);
#endif

    // update position controller with new target
    pos_control->set_desired_velocity(curr_vel_des);
}

// helper function to set yaw state and targets
void ModeFT_LQR::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

// Guided Limit code

// ft_lqr_limit_clear - clear/turn off guided limits
void ModeFT_LQR::limit_clear()
{
    ft_lqr_limit.timeout_ms = 0;
    ft_lqr_limit.alt_min_cm = 0.0f;
    ft_lqr_limit.alt_max_cm = 0.0f;
    ft_lqr_limit.horiz_max_cm = 0.0f;
}

// ft_lqr_limit_set - set guided timeout and movement limits
void ModeFT_LQR::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    ft_lqr_limit.timeout_ms = timeout_ms;
    ft_lqr_limit.alt_min_cm = alt_min_cm;
    ft_lqr_limit.alt_max_cm = alt_max_cm;
    ft_lqr_limit.horiz_max_cm = horiz_max_cm;
}

// ft_lqr_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_ft_lqr_start function
void ModeFT_LQR::limit_init_time_and_pos()
{
    // initialise start time
    ft_lqr_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    ft_lqr_limit.start_pos = inertial_nav.get_position();
}

// ft_lqr_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_FT_LQR_ENABLE mission command
bool ModeFT_LQR::limit_check()
{
    // check if we have passed the timeout
    if ((ft_lqr_limit.timeout_ms > 0) && (millis() - ft_lqr_limit.start_time >= ft_lqr_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(ft_lqr_limit.alt_min_cm) && (curr_pos.z < ft_lqr_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(ft_lqr_limit.alt_max_cm) && (curr_pos.z > ft_lqr_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (ft_lqr_limit.horiz_max_cm > 0.0f) {
        float horiz_move = get_horizontal_distance_cm(ft_lqr_limit.start_pos, curr_pos);
        if (horiz_move > ft_lqr_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}


uint32_t ModeFT_LQR::wp_distance() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_distance_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_distance_to_target();
        break;
    default:
        return 0;
    }
}

int32_t ModeFT_LQR::wp_bearing() const
{
    switch(mode()) {
    case Guided_WP:
        return wp_nav->get_wp_bearing_to_destination();
        break;
    case Guided_PosVel:
        return pos_control->get_bearing_to_target();
        break;
    default:
        return 0;
    }
}

float ModeFT_LQR::crosstrack_error() const
{
    if (mode() == Guided_WP) {
        return wp_nav->crosstrack_error();
    } else {
        return 0;
    }
}

#endif
