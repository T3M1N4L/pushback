/**
 * @file autons.cpp
 * @brief Autonomous routine implementations
 * 
 * This file contains all autonomous routine implementations.
 * Add your autonomous code to these functions.
 */

#include "main.h"
#include "autons.hpp"

// External references from main.cpp
extern lemlib::Chassis chassis;
extern rd::Console console;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup intake;
extern pros::Controller controller;
extern pros::adi::Pneumatics toungeMech;
extern pros::adi::Pneumatics wing;

// ============================= Autonomous Routines ============================= //

/**
 * @brief Competition autonomous routine
 */
void compAuton() {
    console.println("=== COMPETITION AUTON STARTED ===");
    
    // Add your competition autonomous code here
    // Example:
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPose(0, 24, 0, 2000);
    // chassis.moveToPose(24, 24, 90, 2000);
    
    console.println("Competition auton complete!");
}

/**
 * @brief Skills autonomous routine
 */
void skillsAuton() {
    console.println("=== SKILLS AUTON STARTED ===");
    
    // Add your skills autonomous code here
    // Example:
    // chassis.setPose(0, 0, 0);
    // Full 1 minute skills run code...
    
    console.println("Skills auton complete!");
}

/**
 * @brief Do nothing autonomous
 */
void doNothing() {
    console.println("Do Nothing auton selected - robot inactive");
}

// ============================= Add More Autons Below ============================= //

// Example additional autonomous routines:
/*
void leftSideAuton() {
    console.println("=== LEFT SIDE AUTON ===");
    chassis.setPose(-48, 0, 0);
    // Your left side code...
}

void rightSideAuton() {
    console.println("=== RIGHT SIDE AUTON ===");
    chassis.setPose(48, 0, 0);
    // Your right side code...
}

void safeAuton() {
    console.println("=== SAFE AUTON ===");
    // Simple, reliable autonomous
    chassis.moveToPoint(0, 24, 2000);
}
*/
