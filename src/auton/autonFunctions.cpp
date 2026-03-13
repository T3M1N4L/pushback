#include "main.h"
#include "globals.h" 
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "MCL.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void storage(int voltage = 127)
{
    intakeMotor.move(voltage);
    topMotor.move(-5);

}

void intake(int voltage = 127)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}

void outtake(int voltage = 127)
{
    intakeMotor.move(-voltage);
    topMotor.move(-voltage);
}

void score_bottomgoal(int voltage = 127)
{
    intakeMotor.move(-voltage);
    topMotor.move(-voltage);
}

void score_longgoal(int voltage = 127)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}


void score_longgoal_auton(int voltage)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}



void intake_stop()
{
    intakeMotor.move(0);
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    topMotor.move(0);
}

void score_midgoal(int voltage = 90)
{
    pulldown.retract();
    wing.extend();
    intakeMotor.move(voltage);
    topMotor.move(-20);
}



void resting_state()
{
    intake_stop();
    tongue.retract();
    wing.retract();
    pulldown.extend();
}

void pulldown_state(bool state)
{
    if (state) {
        pulldown.extend();
    } else {
        pulldown.retract();
    }
}

void tongue_state(bool state)
{
    if(state && !tongue.is_extended())
    {
        tongue.extend();
    }
    else if(tongue.is_extended())
    {
        tongue.retract();
    }
    else {
        std::cout << "tongue mech state unchanged\n" << std::endl;
    }
}


// ============================= Odometry Reset Functions ============================= //

void reset_odometry()
{
    // Placeholder for future odometry reset functionality
}

// ==================== MCL Position Reset ====================
// Instantly snap odometry to MCL position estimate (bypasses fusion filter)
// Useful after robot reaches known field feature or MCL has converged
void MCL_reset(bool x = true, bool y = true)
{
    MCL::particle_mutex.take();  // Lock particle data for thread-safe read
    // Selectively use MCL position or keep current odometry per axis
    float X = x ? MCL::X : chassis.getPose().x;  // Use MCL X if x=true, else keep odom X
    float Y = y ? MCL::Y : chassis.getPose().y;  // Use MCL Y if y=true, else keep odom Y
    MCL::particle_mutex.give();  // Unlock particle data
    
    // Directly set chassis position (heading always from odometry/IMU)
    chassis.setPose(X, Y, chassis.getPose().theta);
}

// ============================= Fused Odometry System ============================= //
// Combines high-frequency odometry with robust MCL position estimates
// Uses complementary filter (exponential smoothing) to gradually correct drift

pros::Task* fusionTask = nullptr;  // Background task handle for fusion loop

// ==================== Fusion Loop (100 Hz) ====================
// Complementary filter: fuses odometry (continuous) with MCL (50 Hz sensor-based)
// Formula: new_pos = odom_pos + (MCL_pos - odom_pos) × GAIN
// This creates smooth corrections without jerky motion from discrete MCL updates
void fusion_loop_fn(void* ignore) {
    const uint32_t LOOP_DELAY_MS = 10;  // 100 Hz update rate (10ms period)
    uint32_t start_time = pros::millis();  // Track loop timing

    // ==================== Fusion Gain (5% per frame) ====================
    // Higher gain = faster correction but jerkier motion
    // Lower gain = smoother but slower drift correction
    // 0.05 chosen empirically: reaches 63% correction in ~200ms (20 frames)
    const float MCL_GAIN = 0.05f;  // 5% correction per 10ms frame

    while (true) {
        // ==================== Read Current Positions ====================
        // Get high-frequency odometry estimate (from encoders + IMU)
        lemlib::Pose odomPose = chassis.getPose(true);
        
        // Get MCL position estimate (thread-safe read from particle filter)
        MCL::particle_mutex.take();  // Lock for atomic read
        float target_x = MCL::X;     // MCL's weighted average X position
        float target_y = MCL::Y;     // MCL's weighted average Y position
        MCL::particle_mutex.give();  // Unlock

        // ==================== Complementary Filter ====================
        // Exponential smoothing: gradually move odometry toward MCL estimate
        // Error = (MCL position - odometry position)
        // Correction = Error × Gain (5% of error per frame)
        float new_x = odomPose.x + (target_x - odomPose.x) * MCL_GAIN;  // X correction
        float new_y = odomPose.y + (target_y - odomPose.y) * MCL_GAIN;  // Y correction
        
        // Update chassis pose with fused position (heading unchanged - trust IMU)
        chassis.setPose(new_x, new_y, odomPose.theta);

        // ==================== Loop Timing ====================
        // Maintain precise 10ms period (100 Hz)
        pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
    }
}

// ==================== Enable/Disable Fused Odometry ====================
// Spawns or kills background fusion task
// Call enable_fused_odometry(true) after starting MCL in autonomous
void enable_fused_odometry(bool enable) {
    if (enable) {
        // Start fusion task if not already running
        if (fusionTask == nullptr) {
            fusionTask = new pros::Task(fusion_loop_fn, NULL, "MCL_Fusion");  // Create 100 Hz background task
        }
    } else {
        // Stop fusion task and clean up
        if (fusionTask != nullptr) {
            fusionTask->remove();  // Terminate task
            delete fusionTask;     // Free memory
            fusionTask = nullptr;  // Reset pointer
        }
    }
}

void relativeMotion(float expected_x, float expected_y, float expected_theta, float distance, int timeout_ms, bool forw = true, float earlyExit = 0)
{
    lemlib::Pose targetPose(
        expected_x + distance * std::sin(lemlib::degToRad(expected_theta)),
        expected_y + distance * std::cos(lemlib::degToRad(expected_theta)),
        expected_theta
    );

    chassis.moveToPoint(targetPose.x, targetPose.y, timeout_ms, {.forwards = forw, .earlyExitRange = earlyExit});
}


