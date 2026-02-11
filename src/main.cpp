#include "main.h"
#include "pid_tuner.hpp"
#include <sys/_intsup.h>

// Pneumatic mechanisms
pros::adi::Pneumatics toungeMech('E', false);
pros::adi::Pneumatics wing('B', false);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup leftMotors({-1, -2, -7}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({10, 9, 17}, pros::MotorGearset::blue);
pros::MotorGroup intake({3, -18}, pros::MotorGearset::blue);
 
// Sensors
pros::Imu imu(4);
pros::Rotation verticalEnc(-6);
pros::Rotation horizontalEnc(-15);

// Tracking wheels
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, .5);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -4.5);
 
// Drivetrain configuration
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.95, 
                              lemlib::Omniwheel::NEW_4, 450, 8);
 
// Lateral motion controller
lemlib::ControllerSettings linearController(8.5, 0, 43, 3, 1, 100, 3, 500, 110);
 
// Angular motion controller
lemlib::ControllerSettings angularController(5.5, 0, 42.4, 3, 1, 100, 3, 500, 0);
 
// Odometry sensors
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);
 
// Drive curves for driver control
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
 
// Create chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, 
                       sensors, &throttleCurve, &steerCurve);

// Create PID tuner
PIDTuner pidTuner(chassis, controller);
 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
 
    // Set initial PID values for the tuner to match your current configuration
    pidTuner.pid_tuner_set_linear(8.5, 0, 43, 3);  // Match linearController settings
    pidTuner.pid_tuner_set_angular(5.5, 0, 42.4, 3); // Match angularController settings
    
    // Configure increment values (optional)
    pidTuner.pid_tuner_increment_p_set(0.1);
    pidTuner.pid_tuner_increment_i_set(0.001);
    pidTuner.pid_tuner_increment_d_set(0.5);
    pidTuner.pid_tuner_increment_windup_set(0.1);
 
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
 
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
 
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            // Note: telemetrySink() may not be available in all LemLib versions
            // lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
void compAuton(){
}
void skillsAuton(){
}
void autonomous() {
skillsAuton();
    }
/**     
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // PID Tuner - Enable with X button (like EZ-Template)
        // When enabled:
        // - LEFT/RIGHT arrows: switch between Linear and Angular controllers
        // - UP/DOWN arrows: select constant to tune (kP, kI, kD, windupRange)
        // - A button: increase value
        // - Y button: decrease value
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            pidTuner.pid_tuner_toggle();
        }

        // Update the PID tuner
        pidTuner.pid_tuner_iterate();

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
 
 
        // delay to save resources
        pros::delay(10);
    }
}