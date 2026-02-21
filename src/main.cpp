#include "main.h"
#include "robodash/api.h"
#include <sys/_intsup.h>

// Pneumatic mechanisms
pros::adi::Pneumatics toungeMech('E', false);
pros::adi::Pneumatics wing('B', false);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup leftMotors({-10, 9,-3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({6, -8, 7}, pros::MotorGearset::blue);
pros::MotorGroup intake({21, -18}, pros::MotorGearset::blue);
 
// Sensors
pros::Imu imu(4);
pros::Rotation verticalEnc(-17);

// Tracking wheels
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -4.5);
 
// Drivetrain configuration
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.95, 
                              lemlib::Omniwheel::NEW_4, 450, 8);
 
// Lateral motion controller
lemlib::ControllerSettings linearController(8.5, 0, 43, 3, 1, 100, 3, 500, 110);
 
// Angular motion controller
lemlib::ControllerSettings angularController(5.5, 0, 42.4, 3, 1, 100, 3, 500, 0);
 
// Odometry sensors
lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);
 
// Drive curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
 
// Create chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, 
                       sensors, &throttleCurve, &steerCurve);

// Create robodash console
rd::Console console("Console", &controller);

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

// Create robodash selector with autonomous routines
// Format: {"Name", function, "image_path", color_hue}
// color_hue: 0=red, 60=yellow, 120=green, 180=cyan, 220=blue, 300=magenta
rd::Selector selector({
    {"Competition Auton", compAuton, "", 0},      // Red
    {"Skills Auton", skillsAuton, "", 220},       // Blue
    {"Do Nothing", doNothing, "", 120}            // Green
}, &controller);

// Create image widget
rd::Image teamLogo("/img/gengy.bin", "Gengar");

// Create position display view
rd::Position position(&chassis, {"skills.bin", "match.bin"}, {"Skills", "Match"}, &controller);

// Create motor telemetry screen
rd::MotorTelemetry motorTelemetry("Motor Telemetry", {
    {&leftMotors, "LFT"},
    {&rightMotors, "RGT"},
    {&intake, "INT"}
}, &controller);

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
    pidTuner.set_use_tuner_pid(true);  // Set to false to use lemlib defaults
    
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
  
        // delay to save resources
        pros::delay(10);
    }
}
