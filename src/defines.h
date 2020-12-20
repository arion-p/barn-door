

// Constants to set based on hardware construction specs
// Barn door geometry
static const float STEP_SIZE_DEG = 1.8;         // Degrees rotation per step
static const float MICRO_STEPS = 32;            // Number of microsteps per step
static const float THREADS_PER_CM = 8;          // Number of threads in rod per cm of length
static const float BASE_LEN_UPPER_CM = 28.1;    // Length from hinge to center of rod in cm
static const float BASE_LEN_LOWER_CM = 28;      // Length from hinge to center of rod in cm

// Motor specs
#define MOTOR_ACCELERATION  1000L       // steps per sec^2 - need to test this
#define MOTOR_MAX_SPEED     4000L       // steps per sec
#define MOTOR_SLOW_SPEED        0.05    // cm/sec
#define MOTOR_VERY_SLOW_SPEED   0.01    // cm/sec

// Constants to set based on electronic construction specs
// Note that step pin MUST be connected to pin 9 since
// this is the output of timer 1 comparator (OC1A)
#define PIN_OUT_STEP        9   // Arduino digital pin connected to DRV8825 step
#define PIN_OUT_DIRECTION   5   // Arduino digital pin connected to DRV8825 direction
#define PIN_OUT_ENABLE      6   // Arduino digital pin connected to DRV8825 enable

#define PIN_IN_START        A0
#define PIN_IN_STOP         A1
#define PIN_IN_REWIND       A2
#define PIN_IN_DIRECTION    A3

#ifdef PRO_MINI
#define PIN_IN_START_LIMIT  A6
#define PIN_IN_END_LIMIT    A7
#else
#define PIN_IN_START_LIMIT  A4
#define PIN_IN_END_LIMIT    A5
#endif

// Timing
#define DEFAULT_PLANAHEAD_TIME      15000L
