// -*- c -*-
//
// barndoor.ino: arduino code for an astrophotography barndoor mount
//
// Copyright (C) 2014-2015 Daniel P. Berrange
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// This code is used to drive the circuit described at
//
//  http://fstop138.berrange.com/2014/01/building-an-barn-door-mount-part-1-arduino-stepper-motor-control/
//
// Based on the maths calculations documented at
//
//  http://fstop138.berrange.com/2014/01/building-an-barn-door-mount-part-2-calculating-mount-movements/
//
// The code assumes an **isosceles** drive barn door mount design.
//
// Other barndoor drive designs will require different mathematical
// formulas to correct errors

#include <Fsm.h>
#include <FastAccelStepper.h>
#include <avdweb_Switch.h>

#include "defines.h"
#include "events.h"
#include "settings.h"
#include "usb_control.h"

#define PREFER_SINE

// We don't want to send debug over the serial port by default since
// it seriously slows down the main loop causing tracking errors
#define DEBUG

// Constants to set based on hardware construction specs
//
// Assuming you followed the blog linked above, these few variables
// should be the only things that you need to change in general
//
// static const float INITIAL_OPENING = 2.1;       // Initial opening of barn doors when switched on 
//                                                 // (distance between two pivot points in cm)
// static const float MAXIMUM_OPENING = 16.5;      // Maximum distance to allow barn doors to open (30 deg == 2 hours)

// Nothing below this line should require changing unless your barndoor
// is not an Isoceles mount, or you changed the electrical circuit design

#define TICKS_PER_MS        (F_CPU / 1000L)

// Derived constants
static const long MOTOR_USTEPS_ACCELERATION = (MOTOR_ACCELERATION * MICRO_STEPS);
static const long MOTOR_MAX_USTEPS_SPEED = F_CPU / (MOTOR_MAX_SPEED * MICRO_STEPS);

static const float USTEPS_PER_ROTATION = 360.0 / STEP_SIZE_DEG * MICRO_STEPS; // usteps per rod rotation
static const float USTEPS_PER_CM = THREADS_PER_CM * USTEPS_PER_ROTATION;
static const long MOTOR_SLOW_SPEED_USTEPS = F_CPU / (MOTOR_SLOW_SPEED * USTEPS_PER_CM);
static const long MOTOR_VERY_SLOW_SPEED_USTEPS = F_CPU / (MOTOR_VERY_SLOW_SPEED * USTEPS_PER_CM);

// Assuming:
// d = thread length (between two pivot points)
// l1 = lower arm length (from hinge axis to lower pivot point axis)
// l2 = upper arm length (from hinge axis to upper pivot point axis)
// theta = angle between arms (between two pivot points)
//
// then:
// d^2 = l1^2 + l2^2 - 2 * l1 * l2 * cos(theta) = a * cos(theta) + b
// where:
// a = -2 * l1 * l2
// b = l1^2 + l2^2
// 
// Alternate calculation (more accurate because cos() has limited accuracy near 0)
// Assuming: 
// theta = 2*phi
// then:
// d^2 = l1^2 + l2^2 - 2 * l1 * l2 * cos(2*phi) 
//     = l1^2 + l2^2 - 2 * l1 * l2 * (1-2*sin^2(phi))
//     = l1^2 + l2^2 - 2 * l1 * l2 + 4 * l1 * l2 * sin^2(phi)
//     = (l1 - l2) + 4 * l1 * l2 * sin^2(phi)
//     = a * sin^2(theta/2) + b
// where:
// a = 4 * l1 * l2
// b = (l1 - l2)^2

// Standard constants
static const float SIDE_REAL_SECS = 86164.0905; // time in seconds for 1 rotation of earth


#ifdef PREFER_SINE
static const float ALPHA = 4 * BASE_LEN_LOWER_CM * BASE_LEN_UPPER_CM;
static const float BETA = (BASE_LEN_LOWER_CM - BASE_LEN_UPPER_CM) * (BASE_LEN_LOWER_CM - BASE_LEN_UPPER_CM);

static const float LAMBDA = PI / (1000.0 * SIDE_REAL_SECS);         // ms to angle factor
static const float ALPHA2 = ALPHA * USTEPS_PER_CM * USTEPS_PER_CM;
static const float BETA2 = BETA * USTEPS_PER_CM * USTEPS_PER_CM;

#else
static const float ALPHA = -2 * BASE_LEN_LOWER_CM * BASE_LEN_UPPER_CM;
static const float BETA = BASE_LEN_LOWER_CM * BASE_LEN_LOWER_CM + BASE_LEN_UPPER_CM * BASE_LEN_UPPER_CM;

static const float LAMBDA = 2 * PI / (1000.0 * SIDE_REAL_SECS); // ms to angle factor
#endif

// Setup motor class with parameters targetting an DRV8825 board

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motor = engine.stepperConnectToPin(PIN_OUT_STEP);

Switch StartButton(PIN_IN_START);
Switch StopButton(PIN_IN_STOP);
Switch RewindButton(PIN_IN_REWIND);
Switch StartLimit(PIN_IN_START_LIMIT, INPUT_PULLUP, false, 10, 300, 250, 5);
Switch EndLimit(PIN_IN_END_LIMIT, INPUT_PULLUP, false, 10, 300, 250, 5);

// Forward declaration
extern Fsm barndoor;

#ifdef USB_CONTROL
UsbControl usbControl;
#endif


#ifdef PREFER_SINE
// Given time offset from the 100% closed position, figure out
// the total number of steps required to achieve that
long time_to_usteps(long ms)
{
    return (long)(sqrt(ALPHA2 * square(sin(ms * LAMBDA)) + BETA2));
}

// Given total number of steps from 100% closed position, figure out
// the corresponding total tracking time in ms
long usteps_to_time(long usteps)
{
    return (long)(
      asin(sqrt((square((float)usteps) - BETA2) / ALPHA2)) *
      1000.0 * SIDE_REAL_SECS / PI);
}
#else

// Given time offset from the 100% closed position, figure out
// the total number of steps required to achieve that
long time_to_usteps(long ms)
{
    return (long)(USTEPS_PER_ROTATION *
                  THREADS_PER_CM * 
                  sqrt(ALPHA * cos(ms * LAMBDA) + BETA));
                  //sqrt(ALPHA * cos(ms / 1000.0 * 2 * PI / SIDE_REAL_SECS) + BETA));

}

// Given total number of steps from 100% closed position, figure out
// the corresponding total tracking time in ms
long usteps_to_time(long usteps)
{
    return (long)(
      acos((square(usteps / (USTEPS_PER_ROTATION * THREADS_PER_CM)) - BETA) / ALPHA) *
      1000.0 * SIDE_REAL_SECS / (2 * PI));
}

#endif

// Given an angle, figure out the usteps required to get to
// that point.
long angle_to_usteps(float angle)
{
    return time_to_usteps(SIDE_REAL_SECS * 1000.0 / 360.0 * angle);
}

long distance_to_usteps(float distance)
{
    return distance * USTEPS_PER_CM;
}

// These variables are initialized when the motor switches
// from stopped to running, so we know our starting conditions


// If the barn door doesn't go to 100% closed, this records
// the inital offset we started from for INITIAL_ANGLE
static long offsetPositionUSteps;
// The maximum we're willing to open the mount to avoid the
// doors falling open and smashing the camera. Safety first :-)
static long maximumPositionUSteps;
// Total motor steps since 100% closed, at the time the
// motor started running
static long startPositionUSteps;
// Total tracking time associated with total motor steps
static long startPositionMs;
// Wall clock time at the point the motor switched from
// stopped to running.
// static long startWallClockMs;


// These variables are used while running to calculate our
// constantly changing targets

// The wall clock time where we need to next calculate tracking
// rate / target
// static long targetWallClockMs;
// Total tracking time associated with our target point
static long targetPositionMs;
// Total motor steps associated with target point
static long targetPositionUSteps;

// base step time in ticks
static long stepTime;
// The following are used in the line algorithm
// used to distribute the remaining ticks between steps
// number of steps in the plan
static long deltaSteps;
// number of extra ticks to distribute
static long deltaTicks;
// the cumulative error multiplied by 2*deltaSteps
static long ticksError;

// static float minSpeed;
// static float maxSpeed;

// The logical motor position which takes into account the
// fact that we have an initial opening angle
long motor_position()
{
    return motor->getCurrentPosition() + offsetPositionUSteps;
}

long motor_position(long position)
{
    return position + offsetPositionUSteps;
}

long motor_position_at_queue_end()
{
    return motor->getPositionAfterCommandsCompleted() + offsetPositionUSteps;
}


void motor_moveTo(long position)
{
    position -= offsetPositionUSteps;
    if(motor->targetPos() != position)
    {
#ifdef DEBUG
        Serial.print("# motor current target pos: ");
        Serial.print(motor->targetPos());
        Serial.print(", new target pos: ");
        Serial.print(position);
        Serial.print("\n");
#endif
        motor->moveTo(position);
    }
}

void motor_stop()
{
    motor->moveTo(motor->getPositionAfterCommandsCompleted());

}

// This is called whenever the motor is switch from stopped to running.
//
// It reads the current motor position which lets us see how many steps
// have run since the device was first turned on in the 100% closed
// position. From that we then figure out the total tracking time that
// corresponds to our open angle. This is then used by plan_tracking()
// to figure out subsequent deltas
void start_tracking(void)
{
    startPositionUSteps = motor_position_at_queue_end();
    startPositionMs = usteps_to_time(startPositionUSteps);
    targetPositionMs = startPositionMs;
    targetPositionUSteps = startPositionUSteps;
    // startWallClockMs = millis();
    // targetWallClockMs = startWallClockMs;

    // minSpeed = 0;
    // maxSpeed = 0;

#ifdef DEBUG
    Serial.print("# Enter sidereal\n");
    Serial.print("# offset pos usteps: ");
    Serial.print(offsetPositionUSteps);
    Serial.print(", start pos usteps: ");
    Serial.print(startPositionUSteps);
    Serial.print(", start pos ms: ");
    Serial.print(startPositionMs);
    // Serial.print(", start wclk ms: ");
    // Serial.print(startWallClockMs);
    Serial.print("\n\n");
#endif
}


// This is called when we need to figure out a new target position
//
// The tangent errors are small enough that over a short period,
// we can assume constant linear motion will give constant angular
// motion.
//
// So we set our target values to what we expect them all to be
// 15 seconds  in the future
void plan_tracking()
{
    long current_position = motor_position();
    //targetWallClockMs = targetWallClockMs + PLANAHEAD_TIME;
    //long newTargetPositionMs = startPositionMs + (targetWallClockMs - startWallClockMs);
    long newTargetPositionMs = targetPositionMs + PlanAheadTrueTimeMs;
    long newTargetPositionUSteps = time_to_usteps(newTargetPositionMs);
    deltaSteps = newTargetPositionUSteps - targetPositionUSteps;
    deltaTicks = TICKS_PER_MS * PlanAheadSysTimeMs;
    stepTime = deltaTicks / deltaSteps;
    deltaTicks -= stepTime * deltaSteps;
    ticksError = 0;
    
    targetPositionMs = newTargetPositionMs;
    targetPositionUSteps = newTargetPositionUSteps;

#ifdef DEBUG
    Serial.print("# current pos usteps: ");
    Serial.print(current_position);
    Serial.print("/");
    Serial.print(motor_position_at_queue_end());
    Serial.print(", target pos usteps: ");
    Serial.print(targetPositionUSteps);
    Serial.print(", target pos ms: ");
    Serial.print(targetPositionMs);
    Serial.print("\n");
#endif
    // minSpeed = 200;
    // maxSpeed = 0;
}


// This is called on every iteration of the main loop
//
// It looks at our target steps and target wall clock time and
// figures out the rate of steps required to get to the target
// in the remaining wall clock time. This applies the constant
// linear motion expected by  plan_tracking()
//
// By re-calculating rate of steps on every iteration, we are
// self-correcting if we are not invoked by the arduino at a
// constant rate
void apply_tracking()
{
 
#ifdef DEBUG32
    Serial.print("# Target ");
    Serial.print(targetPositionUSteps);
    Serial.print("  curr ");
    Serial.print(motor_position());
    Serial.print("  left");
    Serial.print(stepsLeft);
    Serial.print("\n");
#endif

    struct stepper_command_s cmd = {
        .ticks = 0, .steps = 1, .count_up = true};

    if (motor_position_at_queue_end() >= maximumPositionUSteps) {
        if (!motor->isRunning()) {
            barndoor.trigger(EVENT_END_SWITCH);
        }
    } else {
        // Add queue entry if necessary
        if(!motor->isQueueFull()) {

            if(motor_position_at_queue_end() >= targetPositionUSteps) {
                plan_tracking();
            }

            // Line drawing algorithm used to distribute extra ticks evenly between steps
            long ticks = stepTime;
            ticksError += 2 * deltaTicks;
            if(ticksError > deltaSteps) {
                ticksError -= 2 * deltaSteps;
                ticks++;
            }
            cmd.ticks = ticks;
            motor->addQueueEntry(&cmd);
        }
    }
}


// Called when switching from stopped to running
// in sidereal tracking mode
void state_sidereal_enter(void)
{
    start_tracking();
    plan_tracking();
}


// Called on every tick, when running in sidereal
// tracking mode
//
// XXX we don't currently use the direction switch
// in sidereal mode. Could use it for sidereal vs lunar
// tracking rate perhaps ?
void state_sidereal_update(void)
{
    apply_tracking();
}

void state_sidereal_exit(void)
{
    // nada
    motor_stop();
}

void state_highspeed_enter(void)
{
    motor_stop();
    delay(10);

#ifdef DEBUG
    Serial.print("# Enter highspeed\n");
#endif
}


// Called on every iteration when in non-tracking highspeed
// forward/back mode. Will automatically step when it
// hits the 100% closed position to avoid straining
// the motor
void state_highspeed_update(void)
{
    // pinInDirection is a 2-position switch for choosing direction
    // of motion
    if (digitalRead(PIN_IN_DIRECTION)) {
        if (motor_position() >= maximumPositionUSteps) {
            //motor->addQueueStepperStop();
            barndoor.trigger(EVENT_END_SWITCH);
        } else {
            motor_moveTo(maximumPositionUSteps);
        }
    } else {
        if (motor_position() <= offsetPositionUSteps) {
            //motor->addQueueStepperStop();
            barndoor.trigger(EVENT_START_SWITCH);
        } else {
            motor_moveTo(offsetPositionUSteps);
        }
    }
}

void state_highspeed_exit(void)
{
    // nada
    motor_stop();
}

void state_off_enter(void)
{
#ifdef DEBUG
    Serial.print("# Enter off\n");
#endif
    motor_stop();
}

void state_off_update(void)
{
}

void state_off_exit(void)
{
}


void auto_home_motor(void) {

    #ifdef DEBUG
        Serial.print("# Auto homing...");
    #endif
    // Move motor to home position until Start limit switch is activated
        
    StartLimit.poll();
    motor->enableOutputs();
    {
        struct stepper_command_s cmd = {
            .ticks = MOTOR_SLOW_SPEED_USTEPS, .steps = 1, .count_up = false};

        while(!StartLimit.on()) {
            if (!motor->isQueueFull()) motor->addQueueEntry(&cmd);
            StartLimit.poll();
        }
    }

    #ifdef DEBUG
        Serial.print("backing up...");
    #endif
    {
        // Back up slowly until Start limit switch is released
        struct stepper_command_s cmd = {
            .ticks = MOTOR_VERY_SLOW_SPEED_USTEPS, .steps = 1, .count_up = true};

        while(StartLimit.on()) {
            if (motor->isQueueEmpty()) motor->addQueueEntry(&cmd);
            StartLimit.poll();
        }
     }

    // Stop motor. It is now homed
    motor->setCurrentPosition(0);
    #ifdef DEBUG
        Serial.print("done.\n");
    #endif
}

void poll_switches(void) {
    StartButton.poll();
    StopButton.poll();
    RewindButton.poll();
    StartLimit.poll();
    EndLimit.poll();
}

void init_vars() {
    offsetPositionUSteps = distance_to_usteps(InitialOpening);
    maximumPositionUSteps = distance_to_usteps(MaximumOpening);
}

// A finite state machine with 3 states - sidereal, highspeed and off
static State stateSidereal(state_sidereal_enter, state_sidereal_update, state_sidereal_exit);
static State stateHighspeed(state_highspeed_enter, state_highspeed_update, state_highspeed_exit);
static State stateOff(state_off_enter, state_off_update, state_off_exit);
Fsm barndoor(&stateOff);



// Global initialization when first turned off
void setup(void)
{
    readSettings();
    engine.init();
    motor->setDirectionPin(PIN_OUT_DIRECTION);
    motor->setEnablePin(PIN_OUT_ENABLE);
    motor->setAutoEnable(true);
    motor->setAcceleration(MOTOR_USTEPS_ACCELERATION);
    motor->setSpeed(MOTOR_MAX_USTEPS_SPEED);

    barndoor.add_transition(&stateOff, &stateSidereal, EVENT_START_BUTTON, NULL);
    barndoor.add_transition(&stateOff, &stateHighspeed, EVENT_REWIND_BUTTON, NULL);
    barndoor.add_transition(&stateSidereal, &stateOff, EVENT_STOP_BUTTON, NULL);
    barndoor.add_transition(&stateSidereal, &stateOff, EVENT_END_SWITCH, NULL);
    barndoor.add_transition(&stateSidereal, &stateHighspeed, EVENT_REWIND_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateSidereal, EVENT_START_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, EVENT_STOP_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, EVENT_START_SWITCH, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, EVENT_END_SWITCH, NULL);

    barndoor.add_transition(&stateOff, &stateOff, EVENT_AUTO_HOME, auto_home_motor);

    init_vars();    
    auto_home_motor();
#if defined(DEBUG) || defined(USB_CONTROL)
//#ifdef DEBUG 
    Serial.begin(115200);
#endif
}

void loop(void)
{
    long endTime = millis() + 8;
    poll_switches();
    // pinInSidereal/pinInHighspeed are two poles of a 3-position
    // switch, that let us choose between sidereal tracking,
    // stopped and highspeed mode
    if (StartButton.pushed()) {
        barndoor.trigger(EVENT_START_BUTTON);
    }
    if (RewindButton.pushed()) {
        barndoor.trigger(EVENT_REWIND_BUTTON);
    }
    if (StopButton.pushed()) {
        barndoor.trigger(EVENT_STOP_BUTTON);
    }
    if(StopButton.longPress()) {
        barndoor.trigger(EVENT_AUTO_HOME);
    }
    
    if(StartLimit.pushed()) {
        barndoor.trigger(EVENT_START_SWITCH);
    }
    if (EndLimit.pushed()) {
         barndoor.trigger(EVENT_END_SWITCH);
    }

    barndoor.run_machine();

    #ifdef USB_CONTROL
        usbControl.loop(endTime - millis());
    #endif
}


//
// Local variables:
//  c-indent-level: 4
//  c-basic-offset: 4
//  indent-tabs-mode: nil
// End:
//



