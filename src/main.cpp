#include "main.h"
#include "globals.h"
#include "auton/autonRoutines.h"
#include "auton/autonFunctions.h"
#include "robodash/api.h"
#include <sys/_intsup.h>

// Create robodash console
rd::Console console("Console", &controller);

/**
 * @brief Do nothing autonomous
 */
void doNothing() {
    console.println("Do Nothing auton selected - robot inactive");
}

// Create robodash selector with autonomous routines from autonRoutines.h
// Format: {"Name", function, "image_path", color_hue}
// color_hue: 0=red, 60=yellow, 120=green, 180=cyan, 220=blue, 300=magenta
rd::Selector selector({
    {"Right Auton", right_auton, "", 0},
    {"Left Auton", left_auton, "", 0},
    {"Carry Auton", carry_auton, "", 60},
    {"Elim Auton", elim_auton, "", 300},
    {"AWP Auton", awp_auton, "", 220},
    {"Skills Auton", skills_auton, "", 120},
    {"Do Nothing", doNothing, "", 180}
}, &controller);

// Create image widget
rd::Image teamLogo("/img/gengy.bin", "Gengar");

// Create position display view
rd::Position position(&chassis, {"skills.bin", "match.bin"}, {"Skills", "Match"}, &controller);

// Create motor telemetry screen with motor groups and individual motors
// Automatically displays all motors from groups plus individual motors
std::vector<std::tuple<pros::MotorGroup*, const char*>> motor_groups = {
	{&leftMotors, "LFT"},
	{&rightMotors, "RGT"}
};
std::vector<std::tuple<pros::Motor*, const char*>> individual_motors = {
	{&intakeMotor, "INT"},
	{&topMotor, "TOP"}
};
rd::MotorTelemetry motorTelemetry("Motor Telemetry", motor_groups, individual_motors, &controller);

// Create PID tuner screen
rd::PIDTuner pidTuner("PID Tuner", &chassis, &controller);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // Clear controller LCD on startup
    controller.clear();
    pros::delay(50); // Wait for clear to complete
    
    console.println("Initializing robot...");
    console.println("Calibrating sensors...");
    
    chassis.calibrate(); // calibrate sensors
    
    console.println("Calibration complete!");
 
    // Configure PID tuner increment values (optional)
    pidTuner.set_increments(0.1, 0.001, 0.5, 0.1);
    
    // ============================= PID Tuner Mode ============================= //
    // Toggle between PID tuner values and lemlib defaults
    // When TRUE: PID tuner applies its values to the chassis (saved to SD card)
    // When FALSE: PID tuner does NOT touch chassis PID (uses lemlib defaults from above)
    pidTuner.set_use_tuner_pid(false);  // Set to false to use lemlib defaults
    
    console.println("Robot initialized successfully!");

 
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
 
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
 
    // thread to for position logging to console
    pros::Task screenTask([&]() {
        while (true) {
            // Position logging - not needed with new PID tuner screen
            pros::delay(1000);
        }
    });
    
// Background task to update motor telemetry
	pros::Task telemetryTask([&]() {
		while (true) {
			motorTelemetry.auto_update();
            pros::delay(50); // Update every 50ms
        }
    });
    
    // Background task to update PID tuner telemetry
    pros::Task pidTunerTask([&]() {
        while (true) {
            pidTuner.update();
            pros::delay(100); // Update every 100ms
        }
    });
    
    // Background task to update position display
    pros::Task positionTask([&]() {
        while (true) {
            position.update();
            pros::delay(50); // Update every 50ms
        }
    });
    
    // Background task to update console (for controller scrolling)
    pros::Task consoleTask([&]() {
        while (true) {
            console.update();
            pros::delay(50); // Update every 50ms
        }
    });
    
    // Background task to update selector (for controller navigation)
    pros::Task selectorTask([&]() {
        while (true) {
            selector.update();
            pros::delay(50); // Update every 50ms
        }
    });
}

/**
 * Runs once when entering disabled mode
 */
void disabled() {}

/**
 * Runs once when competition control is connected
 * Use this to focus the selector on screen
 */
void competition_initialize() {
    // Show the auton selector on screen when connected to competition switch
    selector.focus();
}

/**
 * Runs the autonomous routine
 */
void autonomous() {
    console.println("=== AUTONOMOUS STARTED ===");
    selector.run_auton();
    console.println("=== AUTONOMOUS COMPLETE ===");
}

/**     
 * Runs in driver control
 */
void opcontrol() {
    printf("\n=== DRIVER CONTROL STARTED ===\n");
    fflush(stdout);
    
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);
        
        // Button mappings for intake/scoring functions
        bool r1_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l1_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        
        if (r1_pressed && r2_pressed) {
            score_longgoal();  // R1+R2 → score long goal
        }
        else if (r1_pressed) {
            storage();  // R1 → storage
        }
        else if (l1_pressed) {
            score_bottomgoal();  // L1 → score low goal
        }
        else {
            intake_stop();  // Stop intake when no buttons pressed
        }
        // Wing control: L2 held extends wing (up)
        if (l2_pressed) {
            wing.extend();
        } else {
            wing.retract();
        }
        
        // Button mappings for tongue and middle goal
        bool a_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool x_pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        // A toggles tongue
        if (a_pressed) {
            tongue.extend();
        } else {
            tongue.retract();
        }

        // X held scores middle goal (pulls down pulldown), otherwise stay extended
        if (x_pressed) {
            score_midgoal();
        } else {
            pulldown.extend();
        }

        // delay to save resources
        pros::delay(10);
    }
}
