#include "globals.h"
#include <sys/types.h>

// ==================== Barrier Crossing Detection Constants ====================
// State machine for crossing parking zone barrier using IMU pitch angle
// Barrier is a raised platform ~2-3 inches high creating a climbing ramp

const double PITCH_CLIMB_THRESHOLD = 15.0; // Degrees robot tilts up when climbing (front wheels on ramp)
const double PITCH_LEVEL_THRESHOLD = 5.0;  // Degrees considered "flat" (within sensor noise)
const double CROSSING_TIMEOUT = 3000;      // Milliseconds to abort if stuck (safety timeout = 3 seconds)
const double DRIVE_SPEED =  90;            // Motor voltage (0-127) for forward power during crossing
const double HEADING_KP = 2.0;             // Proportional gain for heading correction (prevents veering)

// ============================= Barrier Crossing State Machine ============================= //
// State flow: FLAT → CLIMBING (pitch > 15°) → DROPPING (pitch < 5°) → LANDED (pitch ≈ 0°)
// Uses IMU pitch to detect climbing ramp, crossing barrier, and landing on platform
void crossBarrier() {
    double targetHeading = imu.get_heading();  // Lock heading at start (drive straight)
    
    // ==================== State Flags ====================
    bool hasClimbed = false;  // True when robot has tilted up (front wheels on ramp)
    bool hasDropped = false;  // True when robot levels out (on top of barrier)
    
    // ==================== Safety Timeout ====================
    uint32_t startTime = pros::millis();  // Start timer for timeout
    while (true) {
        u_int32_t currentTime = pros::millis();
        
        // Timeout: if crossing takes > 3 seconds, something is wrong
        if (pros::millis() - startTime > CROSSING_TIMEOUT) {
            chassis.tank(-127, -127);  // Back up at full power
            pros::delay(300);           // For 300ms
            chassis.tank(0, 0);         // Stop
            break;  // Exit crossing routine
        }

        // ==================== Sensor Readings ====================
        double currentPitch = imu.get_pitch();      // Tilt angle: positive = front up, negative = front down
        double currentHeading = imu.get_heading();  // Robot orientation (0-360°)
        
        // ==================== Heading Correction ====================
        // Keep robot driving straight using proportional control
        double error = targetHeading - currentHeading;  // Heading error
        
        // Wrap error to [-180, 180] for shortest turn
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double turnOffset = error * HEADING_KP;  // Proportional correction: larger error = stronger turn

        // ==================== Drive Forward with Heading Correction ====================
        chassis.tank(DRIVE_SPEED + turnOffset,  // Left side: add correction (turn right if error positive)
                     DRIVE_SPEED - turnOffset); // Right side: subtract correction
        
        // ==================== State 1: CLIMBING ====================
        // Detect when robot starts climbing ramp (pitch > 15°)
        if (!hasClimbed && currentPitch > PITCH_CLIMB_THRESHOLD) {
            hasClimbed = true;  // Front wheels are on ramp
            std::cout << "Status: CLIMBING" << std::endl;
        }

        // ==================== State 2: DROPPING ====================
        // Detect when robot reaches top and starts leveling out (pitch < 5°)
        if (hasClimbed && !hasDropped && currentPitch < PITCH_LEVEL_THRESHOLD) {
            hasDropped = true;  // Robot is on top of barrier
            std::cout << "Status: DROPPING" << std::endl;
        }

        // ==================== State 3: LANDED ====================
        // Detect when robot is fully on platform (pitch ≈ 0°, flat surface)
        if (hasClimbed && hasDropped && std::abs(currentPitch) < PITCH_LEVEL_THRESHOLD) {
            std::cout << "Status: LANDED" << std::endl;
            
            pros::delay(200);    // Coast forward slightly to clear barrier edge
            chassis.tank(0, 0);  // Stop motors
            break;  // Exit crossing routine (success)
        }
        
        // ==================== Loop Timing ====================
        pros::Task::delay_until(&currentTime, 10);  // Run at 100Hz (10ms loop time)
    }
}