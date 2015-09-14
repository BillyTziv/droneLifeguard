/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

static uint8_t auto_disarming_counter;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = g.rc_4.control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            // run pre-arm-checks and display failures
            pre_arm_checks(true);
            if(ap.pre_arm_check && arm_checks(true,false)) {
                if (!init_arm_motors()) {
                    // reset arming counter if arming fail
                    arming_counter = 0;
                    AP_Notify::flags.arming_failed = true;
                }
            }else{
                // reset arming counter if pre-arm checks fail
                arming_counter = 0;
                AP_Notify::flags.arming_failed = true;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarming_counter = 0;
        }

    // full left
    }else if (tmp < -4000) {
        if (!manual_flight_mode(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        AP_Notify::flags.arming_failed = false;
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
static void auto_disarm_check()
{
    // exit immediately if we are already disarmed or throttle is not zero
    if (!motors.armed() || !ap.throttle_zero) {
        auto_disarming_counter = 0;
        return;
    }

    // allow auto disarm in manual flight modes or Loiter/AltHold if we're landed
    if (manual_flight_mode(control_mode) || ap.land_complete) {
        auto_disarming_counter++;

        if(auto_disarming_counter >= AUTO_DISARMING_DELAY) {
            init_disarm_motors();
            auto_disarming_counter = 0;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false in the unlikely case that arming fails (because of a gyro calibration failure)
static bool init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    // Reset home position
    // -------------------
    if (ap.home_is_set) {
        init_home();
        calc_distance_and_bearing();
    }

    if(did_ground_start == false) {
        startup_ground(true);
        // final check that gyros calibrated successfully
        if (((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) && !ins.gyro_calibrated_ok_all()) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Gyro calibration failed"));
            AP_Notify::flags.armed = false;
            failsafe_enable();
            return false;
        }
        did_ground_start = true;
    }

    // fast baro calibration to reset ground pressure
    init_barometer(false);

    // reset inertial nav alt to zero
    inertial_nav.set_altitude(0.0f);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    ahrs.set_armed(true);

    // set hover throttle
    motors.set_mid_throttle(g.throttle_mid);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif