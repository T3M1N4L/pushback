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
    topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    topMotor.move(0);

}

void intake(int voltage)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}

void outtake(int voltage)
{
    intakeMotor.move(-voltage);
    topMotor.move(-voltage);
}

void score_bottomgoal(int voltage)
{
    intakeMotor.move(-voltage);
    topMotor.move(-voltage);
}

void score_longgoal(int voltage, Color allianceColor)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}

void score_longgoal_auton(int voltage, Color allianceColor)
{
    intakeMotor.move(voltage);
    topMotor.move(voltage);
}

void intake_to_basket()
{
    intakeMotor.move(127);
    topMotor.move(127);
}

void intake_stop()
{
    intakeMotor.move(0);
    topMotor.move(0);
}

void score_midgoal(int voltage = 127)
{
    wing.extend();
    intakeMotor.move(voltage);
    topMotor.move(-30);
}



void resting_state(bool trapDoor_commanded)
{
    intake_stop();
    wing.retract();
    if (trapDoor_commanded) {
        trapDoor.retract();
    }
}

void matchload_state(bool state)
{
    if (state) {
        matchload.extend();
    } else {
        matchload.retract();
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


void reset_odometry()
{

}


void MCL_reset(bool x = true, bool y = true)
{
    MCL::particle_mutex.take();
    float X = x ? MCL::X : chassis.getPose().x;
    float Y = y ? MCL::Y : chassis.getPose().y;
    MCL::particle_mutex.give();
    chassis.setPose(X, Y, chassis.getPose().theta);
}

pros::Task* fusionTask = nullptr;

void fusion_loop_fn(void* ignore) {
    const uint32_t LOOP_DELAY_MS = 10;
    uint32_t start_time = pros::millis();

    const float MCL_GAIN = 0.05f; 

    while (true) {
        lemlib::Pose odomPose = chassis.getPose(true);
        
        MCL::particle_mutex.take();
        float target_x = MCL::X;
        float target_y = MCL::Y;
        MCL::particle_mutex.give();

        float new_x = odomPose.x + (target_x - odomPose.x) * MCL_GAIN;
        float new_y = odomPose.y + (target_y - odomPose.y) * MCL_GAIN;
        chassis.setPose(new_x, new_y, odomPose.theta);

        pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
    }
}

void enable_fused_odometry(bool enable) {
    if (enable) {
        if (fusionTask == nullptr) {
            fusionTask = new pros::Task(fusion_loop_fn, NULL, "MCL_Fusion");
        }
    } else {
        if (fusionTask != nullptr) {
            fusionTask->remove(); 
            delete fusionTask;  
            fusionTask = nullptr; 
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


