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

// http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>
#include <avdweb_Switch.h>

#define PREFER_SINE

// We don't want to send debug over the serial port by default since
// it seriously slows down the main loop causing tracking errors
//#define DEBUG

// Constants to set based on hardware construction specs
//
// Assuming you followed the blog linked above, these few variables
// should be the only things that you need to change in general
//
static const float STEP_SIZE_DEG = 1.8;  // Degrees rotation per step
static const float MICRO_STEPS = 32;      // Number of microsteps per step
static const float THREADS_PER_CM = 8;   // Number of threads in rod per cm of length
static const float BASE_LEN_UPPER_CM = 28.1;    // Length from hinge to center of rod in cm
static const float BASE_LEN_LOWER_CM = 28;      // Length from hinge to center of rod in cm
static const float INITIAL_OPENING = 2.1;       // Initial opening of barn doors when switched on 
                                                // (distance between two pivot points in cm)
static const float MAXIMUM_OPENING = 16.5;      // Maximum distance to allow barn doors to open (30 deg == 2 hours)

#define MOTOR_ACCELERATION  1000L    // steps per sec^2 - need to test this
#define MOTOR_MAX_SPEED     2500L    // steps per sec
#define MOTOR_SLOW_SPEED        0.05     // cm/sec
#define MOTOR_VERY_SLOW_SPEED   0.01     // cm/sec

// Nothing below this line should require changing unless your barndoor
// is not an Isoceles mount, or you changed the electrical circuit design

// Constants to set based on electronic construction specs
static const int pinOutStep = 3;      // Arduino digital pin connected to EasyDriver step
static const int pinOutDirection = 2; // Arduino digital pin connected to EasyDriver direction
static const int pinOutEnable = 4; // Arduino digital pin connected to EasyDriver enable

static const int pinInStart = A0;  
static const int pinInStop = A1; 
static const int pinInRewind = A2; 
static const int pinInDirection = A3; 
#ifdef PRO_MINI
static const int pinInStartLimit = A6;
static const int pinInEndLimit = A7;
#else
static const int pinInStartLimit = A4;
static const int pinInEndLimit = A5;
#endif

#define PLANAHEAD_TIME      15000L

// Derived constants
static const long MOTOR_USTEPS_ACCELERATION = MOTOR_ACCELERATION * MICRO_STEPS;
static const long MOTOR_MAX_USTEPS_SPEED = MOTOR_MAX_SPEED * MICRO_STEPS;

static const float USTEPS_PER_ROTATION = 360.0 / STEP_SIZE_DEG * MICRO_STEPS; // usteps per rod rotation
static const float USTEPS_PER_CM = THREADS_PER_CM * USTEPS_PER_ROTATION;
static const long MOTOR_SLOW_SPEED_USTEPS = (long)(MOTOR_SLOW_SPEED * USTEPS_PER_CM);
static const long MOTOR_VERY_SLOW_SPEED_USTEPS = (long)(MOTOR_VERY_SLOW_SPEED * USTEPS_PER_CM);

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

// Events
#define START_BUTTON        1
#define REWIND_BUTTON       2
#define STOP_BUTTON         3
#define END_SWITCH          4
#define START_SWITCH        5

// Setup motor class with parameters targetting an EasyDriver board
static AccelStepper motor(AccelStepper::DRIVER,
                          pinOutStep,
                          pinOutDirection);

Switch StartButton(pinInStart);
Switch StopButton(pinInStop);
Switch RewindButton(pinInRewind);
Switch StartLimit(pinInStartLimit, INPUT_PULLUP, false, 10, 300, 250, 5);
Switch EndLimit(pinInEndLimit, INPUT_PULLUP, false, 10, 300, 250, 5);

// Forward declaration
extern Fsm barndoor;

#ifdef PREFER_SINE
// Given time offset from the 100% closed position, figure out
// the total number of steps required to achieve that
long time_to_usteps(long ms)
{
    return (long)(sqrt(ALPHA2 * square(sin(ms * LAMBDA)) + BETA2));

    // return (long)(USTEPS_PER_ROTATION *
    //               THREADS_PER_CM * 
    //               sqrt(ALPHA * square(sin(ms * LAMBDA)) + BETA));
}

// Given total number of steps from 100% closed position, figure out
// the corresponding total tracking time in seconds
long usteps_to_time(long usteps)
{
    return (long)(
      asin(sqrt((square(usteps) - BETA2) / ALPHA2)) *
      1000.0 * SIDE_REAL_SECS / PI);
    // return (long)(
    //   asin(sqrt((square(usteps / USTEPS_PER_CM) - BETA) / ALPHA)) *
    //   1000.0 * SIDE_REAL_SECS / PI);
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
// the corresponding total tracking time in seconds
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
static long startWallClockMs;


// These variables are used while running to calculate our
// constantly changing targets

// The wall clock time where we need to next calculate tracking
// rate / target
static long targetWallClockMs;
// Total tracking time associated with our target point
static long targetPositionMs;
// Total motor steps associated with target point
static long targetPositionUSteps;



// The logical motor position which takes into account the
// fact that we have an initial opening angle
long motor_position()
{
    return motor.currentPosition() + offsetPositionUSteps;
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
    startPositionUSteps = motor_position();
    startPositionMs = usteps_to_time(startPositionUSteps);
    startWallClockMs = millis();
    targetWallClockMs = startWallClockMs;

#ifdef DEBUG
    Serial.print("Enter sidereal\n");
    Serial.print("offset pos usteps: ");
    Serial.print(offsetPositionUSteps);
    Serial.print(", start pos usteps: ");
    Serial.print(startPositionUSteps);
    Serial.print(", start pos ms: ");
    Serial.print(startPositionMs);
    Serial.print(", start wclk ms: ");
    Serial.print(startWallClockMs);
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
void plan_tracking(void)
{
    targetWallClockMs = targetWallClockMs + PLANAHEAD_TIME;
    targetPositionMs = startPositionMs + (targetWallClockMs - startWallClockMs);
    targetPositionUSteps = time_to_usteps(targetPositionMs);

#ifdef DEBUG
    Serial.print("current pos usteps: ");
    Serial.print(motor_position());
    Serial.print(", target pos usteps: ");
    Serial.print(targetPositionUSteps);
    Serial.print(", target pos ms: ");
    Serial.print(targetPositionMs);
    Serial.print(", target wclk ms: ");
    Serial.print(targetWallClockMs);
    Serial.print("\n");
#endif
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
void apply_tracking(long currentWallClockMs)
{
    long timeLeft = targetWallClockMs - currentWallClockMs;
    long stepsLeft = targetPositionUSteps - motor_position();
    float stepsPerSec = 1000.0 * (float)stepsLeft / (float)timeLeft;

#ifdef DEBUG32
    Serial.print("Target ");
    Serial.print(targetPositionUSteps);
    Serial.print("  curr ");
    Serial.print(motor_position());
    Serial.print("  left");
    Serial.print(stepsLeft);
    Serial.print("\n");
#endif

    if (motor_position() >= maximumPositionUSteps) {
        motor.stop();
    } else {
        motor.setSpeed(stepsPerSec);
        motor.runSpeed();
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
    long currentWallClockMs = millis();

    if (currentWallClockMs >= targetWallClockMs) {
        plan_tracking();
    }

    apply_tracking(currentWallClockMs);
}

void state_sidereal_exit(void)
{
    // nada
}

void state_highspeed_enter(void)
{
#ifdef DEBUG
    Serial.print("Enter highspeed\n");
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
    if (digitalRead(pinInDirection)) {
        if (motor_position() >= maximumPositionUSteps) {
            motor.stop();
            barndoor.trigger(END_SWITCH);
        } else {
            motor.moveTo(maximumPositionUSteps);

            motor.run();
        }
    } else {
        if (motor.currentPosition() <= 0) {
            motor.stop();
            barndoor.trigger(START_SWITCH);
        } else {
            motor.moveTo(0);

            motor.run();
        }
    }
}

void state_highspeed_exit(void)
{
    // nada
}

void state_off_enter(void)
{
#ifdef DEBUG
    Serial.print("Enter off\n");
#endif
    motor.stop();
}

void state_off_update(void)
{
    if(motor.isRunning()) {
        motor.run();
    } else  {
        motor.disableOutputs(); // save power by disabling outputs
    }
}

void state_off_exit(void)
{
    motor.enableOutputs();
}


void auto_home_motor(void) {

    #ifdef DEBUG
        Serial.print("Auto homing...");
    #endif
    // Move motor to home position until Start limit switch is activated
    motor.setSpeed(-MOTOR_SLOW_SPEED_USTEPS);
    StartLimit.poll();
    motor.enableOutputs();
    while(!StartLimit.on()) {
        motor.runSpeed();
        StartLimit.poll();
    }

    #ifdef DEBUG
        Serial.print("backing up...");
    #endif
    // Back up slowly until Start limit switch is released
    motor.setSpeed(MOTOR_VERY_SLOW_SPEED_USTEPS);
    while(StartLimit.on()) {
        motor.runSpeed();
        StartLimit.poll();
    }

    // Stop motor. It is now homed
    motor.stop();
    motor.setCurrentPosition(0);
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


// A finite state machine with 3 states - sidereal, highspeed and off
static State stateSidereal(state_sidereal_enter, state_sidereal_update, state_sidereal_exit);
static State stateHighspeed(state_highspeed_enter, state_highspeed_update, state_highspeed_exit);
static State stateOff(state_off_enter, state_off_update, state_off_exit);
Fsm barndoor(&stateOff);

// Global initialization when first turned off
void setup(void)
{
    motor.setEnablePin(pinOutEnable);
    motor.setPinsInverted(true, false, true);
    motor.setAcceleration(MOTOR_USTEPS_ACCELERATION);
    motor.setMaxSpeed(MOTOR_MAX_USTEPS_SPEED);
    
    // offsetPositionUSteps = angle_to_usteps(INITIAL_ANGLE);
    // maximumPositionUSteps = angle_to_usteps(MAXIMUM_ANGLE);
    offsetPositionUSteps = distance_to_usteps(INITIAL_OPENING);
    maximumPositionUSteps = distance_to_usteps(MAXIMUM_OPENING);

    barndoor.add_transition(&stateOff, &stateSidereal, START_BUTTON, NULL);
    barndoor.add_transition(&stateOff, &stateHighspeed, REWIND_BUTTON, NULL);
    barndoor.add_transition(&stateSidereal, &stateOff, STOP_BUTTON, NULL);
    barndoor.add_transition(&stateSidereal, &stateOff, END_SWITCH, NULL);
    barndoor.add_transition(&stateSidereal, &stateHighspeed, REWIND_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateSidereal, START_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, STOP_BUTTON, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, START_SWITCH, NULL);
    barndoor.add_transition(&stateHighspeed, &stateOff, END_SWITCH, NULL);

    auto_home_motor();
#ifdef DEBUG
    Serial.begin(115200);
#endif
}

void loop(void)
{
    poll_switches();
    // pinInSidereal/pinInHighspeed are two poles of a 3-position
    // switch, that let us choose between sidereal tracking,
    // stopped and highspeed mode
    if (StartButton.pushed()) {
        barndoor.trigger(START_BUTTON);
    }
    if (RewindButton.pushed()) {
        barndoor.trigger(REWIND_BUTTON);
    }
    if (StopButton.pushed()) {
        barndoor.trigger(STOP_BUTTON);
    }
    
    if(StartLimit.pushed()) {
        barndoor.trigger(START_SWITCH);
    }
    if (EndLimit.pushed()) {
         barndoor.trigger(END_SWITCH);
    }
    barndoor.run_machine();
}


//
// Local variables:
//  c-indent-level: 4
//  c-basic-offset: 4
//  indent-tabs-mode: nil
// End:
//



