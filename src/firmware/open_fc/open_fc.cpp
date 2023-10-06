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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FLAG_VALUE 123

//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
// function to initialize the flight controller
void initializeFlightController() {
    // implement initialization code here
    printf("initializeFlightController()");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// function to read sensors and update flight data
void updateFlightData() {
    printf("updateFlightData()");
    // simulate sensor readings for demonstration
    currentFlightData.altitude += 0.1;
    currentFlightData.pitch += 1.0;
    currentFlightData.yaw += 0.5;
    currentFlightData.roll += 0.8;
    currentFlightData.batteryLevel -= 0.01;

    // check for low battery level
    if (currentFlightData.batteryLevel < MIN_BATTERY_LEVEL) {
        // implement low battery action
        printf("low battery! land immediately.\n");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// function to log flight data
void logFlightData() {
    if (logIndex < MAX_LOG_ENTRIES) {
        flightLog[logIndex].data = currentFlightData;
        flightLog[logIndex].timestamp = 0;
        logIndex++;
    }
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

    printf("Core 0 running!\n");

    multicore_launch_core1(core1_entry);
    // Wait for it to start up
    uint32_t g = multicore_fifo_pop_blocking();
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!");
    }

    // initialize the flight controller after cores have started up.
    initializeFlightController();

    // main loop on core 0
    while (1) {
        // update flight data
        updateFlightData();

        // log flight data
        logFlightData();

        // implement other flight control logic here

        // delay for a short time (simulating real-time operation)
        sleep_ms(100);  // sleep for 100ms
    }

    return 0;
}