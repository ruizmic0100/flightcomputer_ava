/*
Author: [Your Name]
Description: Flight Controller Prototype
This code represents a prototype flight controller for a small fixed-wing aircraft. It simulates sensor readings and includes basic flight control logic for demonstration purposes. The flight controller is designed to provide real-time flight data, log flight information, and handle low battery situations.

Potential Uses:
- Educational tool for learning about flight control systems.
- Basis for developing an actual flight controller for small RC aircraft.
- Research and development in the field of UAVs (Unmanned Aerial Vehicles).

Future Updates:
- Integration with actual sensors for accurate data.
- Implementation of sophisticated flight control algorithms.
- Real-time communication with a ground control station.
- Enhanced safety features and autonomous flight capabilities.

Original Creation Date: September 21st, Thursday
Completion Date: TDB
*/

// Required STD libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Required pico libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Multicore functionality flags
#define FLAG_VALUE 123

// Uncomment only one receiver communication type.
#define USE_PWM_RX
// #define USE_PPM_RX

// Uncomment only one IMU
#define USE_MPU6050_I2C // Default
// #define USE_BMI088_I2C // Board-equipped

// Uncomment only one pressure sensor
// #define USE_BMP390 // Board-equipped

// Uncomment only one digital magnentic sensor
// #define USE_LIS2MDLTR // Board-equipped

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
// #define ACCEl_4G
// #define ACCEl_8G
// #define ACCEl_16G

// Optional libraries
#if defined USE_MPU6050_I2C
    #include "../MPU6050/MPU6050.h"
    // MPU6050 mpu6050;
#elif defined USE_BMI088_I2C
    #include "src/BMI088/BMI088.h"
    // BMI088 bmi088;
#else
    #error No MPU defined...
#endif

// Setup gyro and accel full scale value selection and scale factor
#if defined USE_MPU6050_I2C
    #define GYRO_FS_SEL_250     MPU6050_GYRO_FS_250
    #define GYRO_FS_SEL_500     MPU6050_GYRO_FS_500
    #define GYRO_FS_SEL_1000    MPU6050_GYRO_FS_1000
    #define GYRO_FS_SEL_2000    MPU6050_GYRO_FS_2000
    #define ACCEL_FS_SEL_2      MPU6050_ACCEL_FS_2
    #define ACCEL_FS_SEL_4      MPU6050_ACCEL_FS_4
    #define ACCEL_FS_SEL_8      MPU6050_ACCEL_FS_8
    #define ACCEL_FS_SEL_16     MPU6050_ACCEL_FS_16
// TODO(MSR): do this for the BMI088
#endif

#if defined GYRO_250DPS
    #define GYRO_SCALE GYRO_FS_SEL_250
    #define GYRO_SCALE_FACTOR 131.0
// TODO(MSR) define the rest of the scaling selections
#endif

// ---------------------------------- User-Specified Variables --------------------------------------

// Radio failsafe values for every channel in the even that the bad receiver data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; // throttle
unsigned long channel_2_fs = 1500; // aileron
unsigned long channel_3_fs = 1500; // elevator
unsigned long channel_4_fs = 1500; // rudder
unsigned long channel_5_fs = 2000; // auxillery1
unsigned long channel_6_fs = 2000; // auxillery2

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04; // Madgwick filter parameter
float B_accel = 0.14;    // Accelerometer LP filter parameter, (MPU6050 default: 0.14. TODO(MSR): Figure out what BMI088's is)
float B_gyro = 0.1;      // Gyro LP filter parameter, (MPU6050 default: 0.1, TODO(MSR): Figure out what BMI088's is)
float B_mag = 1.0;       // Magnetometer LP filter parameter

// TODO(MSR): Magnetometer calibration parameters 

// IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

// Controller parameters (take note of defaults before modifying!):
float i_limit = 25.0;       // Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;       // Max roll angle i degrees for roll mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 30.0;      // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;       // Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;      // Roll P-gain - angle mode
float Ki_roll_angle = 0.3;      // Roll I-gain - angle mdoe
float Kd_roll_angle = 0.05;     // Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;        // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;     // Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;     //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;    // Pitch D-gain - angle mode (has no affect on controlANGLE2)
float B_loop_pitch = 0.0;       // Pitch damping term for controlANGLE2(), lower is mroe damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;      // Roll P-gain - rate mode
float Ki_roll_rate = 0.2;       // Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;    // Roll D-gain - rate mode (be careful when increasing to high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;     // Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;      // Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002;   // Pitch D-gain - rate mode (be careful when increasing to high, motors will begin to overheat!)

float Kp_yaw = 0.3;     // Yaw P-gain
float Ki_yaw = 0.05;    // Yaw I-gain
float Kd_yaw = 0.00015; // Yaw D-gain (be careful when increaseing to high, motors will begin to overhead)


// ---------------------------------- Declare Pins --------------------------------------

// TODO(MSR): figure this out
// NOTE: Certain pins are reerved for onboard stuff
// NOTE: for SBUS and PPM special pins are used for that
const int ch1Pin = 15; //throttle



// ---------------------------------- Declare Global Variables --------------------------------------

// General stuff:
float dt;
absolute_time_t current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

// Radio Communication stuff:
unsigned long channel_1_pwm;
unsigned long channel_1_pwm_prev;

// IMU stuff:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; // Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Normalized desired state:
float throttle_desired, roll_desired, yaw_desired;
float roll_passthru, pitch_passthru, yaw_passthru;

// Controller:
float error_roll, error_roll_prev, roll_desired_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_desired_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

// Mixer:
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_comand_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_comand_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

// Flight status
bool armedFly = false;


// define constants and variables
const float MIN_BATTERY_LEVEL = 11.0;  // minimum safe battery level
const int MAX_LOG_ENTRIES = 1000;      // maximum number of data log entries

struct FlightData {
    float altitude;
    float pitch;
    float yaw;
    float roll;
    float batteryLevel;
    // add more data fields as needed
};

struct LogEntry {
    FlightData data;
    uint32_t timestamp;
};

FlightData currentFlightData;
LogEntry flightLog[MAX_LOG_ENTRIES];
int logIndex = 0;

void togglePinTest(const uint testPin) {
    gpio_put(testPin, 1);
    sleep_ms(500);
    gpio_put(testPin, 0);
    sleep_ms(500);
}

void setupBlink() {
    gpio_put(5, 1);
    sleep_ms(100);
    gpio_put(5, 0);
    sleep_ms(100);
    gpio_put(5, 1);
    sleep_ms(100);
    gpio_put(5, 0);
    sleep_ms(100);
    gpio_put(5, 1);
    sleep_ms(100);
    gpio_put(5, 0);
    sleep_ms(100);
}

// ------------------------ timer logic ---------------------------

    /// \tag::timer_example[]
    volatile bool timer_fired = false;

int64_t alarm_callback(alarm_id_t id, void* user_data) {
    printf("Timer %d fired!\n", (int) id);
    timer_fired = true;
    //Can return a value here in us to fire in the future
    return 0;
}

bool repeating_timer_callback(struct repeating_timer *t) {
    printf("Repeat at %lld\n", time_us_64());
    return true;
}

// ------------------------ timer logic ---------------------------

void IMUinit() {
    printf("IMUinit() started");
}

void radioSetup() {
    printf("radioSetup() started");
}

void armedStatus() {
    printf("armedStatus() started");
}


// function to initialize the flight controller
void initializeFlightController() {
    // implement initialization code here
    printf("initializeFlightController() started");
    sleep_ms(500); // Sleep for an arbituary amount of time to help stuff settle on boot up.

    // Simple GPIO test
    gpio_init(5);
    gpio_set_dir(5, GPIO_OUT);

    // Initialize all the pins
    gpio_init(26);
    gpio_init(27);
    gpio_init(28);
    gpio_init(29);
    gpio_init(12); // SDA
    gpio_init(13); // SCL


    // NOTE(MSR): might need to set their dir...
    gpio_set_dir(26, GPIO_OUT);
    gpio_set_dir(27, GPIO_OUT);
    gpio_set_dir(28, GPIO_OUT);
    sleep_ms(5); // Wait a bit for configs to set

    radioSetup();

    // Set radio channels ot default (safe) values before entering main loop
    channel_1_pwm = channel_1_fs;

    // Initialize IMU communication
    IMUinit();
    sleep_ms(5); // Wait a bit for configs to set


    // Get the IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powering up.
    // Calculate_IMU_error(); // Calibration parameters printed to serial monitor. Paste these in the user specififed variables section, then comment this out until further use.
    
    // If using Magnometer, uncomment for one time magnetomer calibration(may need to repeat for new locations)
    // calibrateMagnetometer(); // Generates magnetometer error ands cale factors to be pasted in user-specified variables section.

    // Arm servo channels

    sleep_ms(5);

    // CalibrateESCs(); // PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps.
    // Code will not proceed past here if this function is uncommented!

    // Indicate setup is complete by 3 quick blinks.
    setupBlink();
}

// function to read sensors and update flight data
void updateFlightData() {
    // printf("updateFlightData()");
    // // simulate sensor readings for demonstration
    // currentFlightData.altitude += 0.1;
    // currentFlightData.pitch += 1.0;
    // currentFlightData.yaw += 0.5;
    // currentFlightData.roll += 0.8;
    // currentFlightData.batteryLevel -= 0.01;

    // // check for low battery level
    // if (currentFlightData.batteryLevel < MIN_BATTERY_LEVEL) {
    //     // implement low battery action
    //     printf("low battery! land immediately.\n");
    // }
}

// function to log flight data
void logFlightData() {
    // if (logIndex < MAX_LOG_ENTRIES) {
    //     flightLog[logIndex].data = currentFlightData;
    //     flightLog[logIndex].timestamp = 0;
    //     logIndex++;
    // }
}


void core1_entry() {
    multicore_fifo_push_blocking(FLAG_VALUE);
 
    uint32_t g = multicore_fifo_pop_blocking();
 
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("Its all gone well on core 1!");
 
    while (1)
        tight_loop_contents();
        printf("Core 1 running!");
}
int main() {

    stdio_init_all();

    // printf("Core 0 running!\n");

    // multicore_launch_core1(core1_entry);
    // Wait for it to start up
    // uint32_t g = multicore_fifo_pop_blocking();
    // if (g != FLAG_VALUE)
    //     printf("Hmm, that's not right on core 0!\n");
    // else {
    //     multicore_fifo_push_blocking(FLAG_VALUE);
    //     printf("It's all gone well on core 0!");
    // }

    // initialize the flight controller after cores have started up.
    initializeFlightController();

    // // TODO(MSR): Test this.
    // // Watchdog timer
    // add_alarm_in_ms(2000, alarm_callback, NULL, false);

    // while (!timer_fired) {
    //     tight_loop_contents();
    // }

    // // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
    // // If the delay is negative (see below) then the next call t the callback will be exactly 500ms after the
    // // start of the call to the last callback.
    // struct repeating_timer timer;
    // add_repeating_timer_ms(500, reapeating_timer_callback, NULL, &timer);
    // sleep_ms(3000);
    // bool cancelled = cancel_repeating_timer(&timer);
    // printf("cancelled timer...%d", cancelled);
    // sleep_ms(2000);

    // // Negative delay so means we will cal repeating_timer_callback, and call it again
    // // 500ms later regardless of how long the callback took to execute
    // add_repeating_timer_ms(-500, repeating_timer_callback, NULL, &timer);
    // sleep_ms(3000);
    // cancelled = cancel_reapeating_timer(&timer);
    // printf("cancelled timer...%d", cancelled);
    // sleep_ms(2000);


    // main loop on core 0
    while (1) {

        // Keep track of what time it is and how much time has elapsed since the last loop
        current_time = get_absolute_time();
        printf("current_time = %llu\n", current_time/1000000);

        togglePinTest(5); // Indicate that we are in the main loop with short blinks every 0.5 seconds

        // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
        // printRadioData();     // Prints radio pwm values (expected: 1000 to 2000)
        // printDesiredState();  // Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
        // printGyroData();      // Prints filtered gyro data drirect from IMU (expected: ~ -250 to 250, 0 at rest)
        // printAccelData();     // Prints filtered accelerometer data direct from IMU (expected ~ -2 to 2; x, y, 0 when level, z 1 when level)
        // printMagData();       // Prints filtered magnetometer data direct from IMU (exptected: ~ -300 to 300)
        // printRollPitchYaw();  // Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
        // printPIDoutput();     // Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
        // printMotorCommands(); // Prints the values being written to the motors (expected: 120 to 250)
        // printServoCommands(); // Prints the values being written to the servos (expected: 0 to 180)
        // printLoopRate();      // Prints the time between loops in microseconds (expected: microseconds between loop iterations)

        // get arming status
        armedStatus(); // Check if the throttle cut is off and throttle is low

        // Get vehicle state
        // getIMUdata(); // Pulls raw gyro, acccelerometer, and magnetometer data from IMU and LP filters t remove noise
        // Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)



        // delay for a short time (simulating real-time operation)
        sleep_ms(100);  // sleep for 100ms
    }

    return 0;
}