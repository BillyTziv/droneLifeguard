/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE_ENABLED == ENABLED

/*
 * control_autotune.pde - init and run calls for autotune flight mode
 *
 * Instructions:
 *      1) Set up one flight mode switch position to be AltHold.
 *      2) Set the Ch7 Opt or Ch8 Opt to AutoTune to allow you to turn the auto tuning on/off with the ch7 or ch8 switch.
 *      3) Ensure the ch7 or ch8 switch is in the LOW position.
 *      4) Wait for a calm day and go to a large open area.
 *      5) Take off and put the vehicle into AltHold mode at a comfortable altitude.
 *      6) Set the ch7/ch8 switch to the HIGH position to engage auto tuning:
 *          a) You will see it twitch about 20 degrees left and right for a few minutes, then it will repeat forward and back.
 *          b) Use the roll and pitch stick at any time to reposition the copter if it drifts away (it will use the original PID gains during repositioning and between tests).
 *             When you release the sticks it will continue auto tuning where it left off.
 *          c) Move the ch7/ch8 switch into the LOW position at any time to abandon the autotuning and return to the origin PIDs.
 *          d) Make sure that you do not have any trim set on your transmitter or the autotune may not get the signal that the sticks are centered.
 *      7) When the tune completes the vehicle will change back to the original PID gains.
 *      8) Put the ch7/ch8 switch into the LOW position then back to the HIGH position to test the tuned PID gains.
 *      9) Put the ch7/ch8 switch into the LOW position to fly using the original PID gains.
 *      10) If you are happy with the autotuned PID gains, leave the ch7/ch8 switch in the HIGH position, land and disarm to save the PIDs permanently.
 *          If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the original PIDs. The gains will not be saved when you disarm
 *
 * What it's doing during each "twitch":
 *      a) invokes 90 deg/sec rate request
 *      b) records maximum "forward" roll rate and bounce back rate
 *      c) when copter reaches 20 degrees or 1 second has passed, it commands level
 *      d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
 *      e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
 *      f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
 *      g) increases rate P until the max rotate rate becomes greater than the requeste rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 * Notes: AUTOTUNE should not be set-up as a flight mode, it should be invoked only from the ch7/ch8 switch.
 *
 */

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500    // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS    500    // timeout for tuning mode's testing step
#define AUTOTUNE_TARGET_ANGLE_CD           2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_CDS           9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_LEVEL_ANGLE_CD             300    // angle which qualifies as level
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250    // time we require the copter to be level
#define AUTOTUNE_AGGRESSIVENESS            0.1f    // tuning for 10% overshoot
#define AUTOTUNE_RD_STEP                0.0005f    // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                 0.005f    // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                   0.5f    // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_SP_BACKOFF               0.75f    // Stab P gains are reduced to 75% of their maximum value discovered during tuning
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f    // I is set 10x smaller than P during testing
#define AUTOTUNE_RP_RATIO_FINAL            1.0f    // I is set 1x P after testing
#define AUTOTUNE_RD_MIN                  0.002f    // minimum Rate D value
#define AUTOTUNE_RD_MAX                  0.020f    // maximum Rate D value
#define AUTOTUNE_RP_MIN                   0.01f    // minimum Rate P value
#define AUTOTUNE_RP_MAX                   0.35f    // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f    // maximum Stab P value
#define AUTOTUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3

// autotune modes (high level states)
enum AutoTuneTuneMode {
    AUTOTUNE_MODE_UNINITIALISED = 0,          // autotune has never been run
    AUTOTUNE_MODE_TUNING = 1,               // autotune is testing gains
    AUTOTUNE_MODE_SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
    AUTOTUNE_MODE_FAILED = 3,               // tuning has failed, user is flying on original gains
};

// steps performed while in the tuning mode
enum AutoTuneStepType {
    AUTOTUNE_STEP_WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
    AUTOTUNE_STEP_TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
    AUTOTUNE_STEP_UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTOTUNE_AXIS_ROLL = 0,                 // roll axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_PITCH = 1                 // pitch axis is being tuned (either angle or rate)
};

// mini steps performed while in Tuning mode, Testing step
enum AutoTuneTuneType {
    AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
    AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
    AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
    AUTOTUNE_TYPE_SP_UP = 3                 // angle P is being tuned up
};

// autotune_state_struct - hold state flags
struct autotune_state_struct {
    AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t             pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType    axis                : 1;    // see AutoTuneAxisType for which things can be tuned
    uint8_t             positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType    tune_type           : 2;    // see AutoTuneTuneType
} autotune_state;
