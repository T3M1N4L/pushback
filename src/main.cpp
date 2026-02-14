#include "main.h"
#include "autons.hpp"
#include "pid_tuner.hpp"
#include "robodash/api.h"
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

// Create robodash console
rd::Console console;

// Create PID tuner
PIDTuner pidTuner(chassis, controller, &console);

// Create robodash selector with autonomous routines
// Format: {"Name", function, "image_path", color_hue}
// color_hue: 0=red, 60=yellow, 120=green, 180=cyan, 220=blue, 300=magenta
rd::Selector selector({
    {"Competition Auton", doNothing, "", 0},      // Red
    {"Skills Auton", doNothing, "", 220},       // Blue
    {"Do Nothing", doNothing, "", 120}            // Green
});

// Create motor telemetry screen
rd::MotorTelemetry motorTelemetry("Motor Telemetry", 6);

/**
 * Helper function to add motor group data to telemetry
 */
void add_motor_group_data(std::vector<rd::motor_data_t> &motors, pros::MotorGroup &group, 
                          const std::vector<int> &ports, const char *name) {
    auto vels = group.get_actual_velocity_all();
    auto powers = group.get_power_all();
    auto currents = group.get_current_draw_all();
    auto temps = group.get_temperature_all();
    auto torques = group.get_torque_all();
    
    for (size_t i = 0; i < ports.size(); i++) {
        motors.push_back({ports[i], name, (float)vels[i], (float)(powers[i]/1000.0f),
                         (float)(currents[i]/1000.0f), (float)temps[i], (float)(torques[i]/100.0f)});
    }
}
 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    
    console.println("Initializing robot...");
    console.println("Calibrating sensors...");
    
    chassis.calibrate(); // calibrate sensors
    
    console.println("Calibration complete!");
 
    // Set initial PID values for the tuner to match your current configuration
    pidTuner.pid_tuner_set_linear(8.5, 0, 43, 3);  // Match linearController settings
    pidTuner.pid_tuner_set_angular(5.5, 0, 42.4, 3); // Match angularController settings
    
    // Configure increment values (optional)
    pidTuner.pid_tuner_increment_p_set(0.1);
    pidTuner.pid_tuner_increment_i_set(0.001);
    pidTuner.pid_tuner_increment_d_set(0.5);
    pidTuner.pid_tuner_increment_windup_set(0.1);
    
    console.println("Robot initialized successfully!");
    
    // Selector callback - logs when an auton is selected
    selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
        if (routine == std::nullopt) {
            console.println("No autonomous selected");
        } else {
            console.printf("Selected: %s\n", routine.value().name.c_str());
        }
    });
 
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
 
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
 
    // thread to for position logging to console
    pros::Task screenTask([&]() {
        while (true) {
            // Only update position if PID tuner is not active
            if (!pidTuner.pid_tuner_enabled()) {
                // Update robot location on separate lines (in-place editing)
                char x_buffer[50], y_buffer[50], theta_buffer[50];
                snprintf(x_buffer, sizeof(x_buffer), "X: %.2f", chassis.getPose().x);
                snprintf(y_buffer, sizeof(y_buffer), "Y: %.2f", chassis.getPose().y);
                snprintf(theta_buffer, sizeof(theta_buffer), "Theta: %.2f", chassis.getPose().theta);
                
                console.update_line(0, x_buffer);
                console.update_line(1, y_buffer);
                console.update_line(2, theta_buffer);
            }
            // delay to save resources
            pros::delay(1000);
        }
    });
    
    // Background task to update motor telemetry
    pros::Task telemetryTask([&]() {
        while (true) {
            std::vector<rd::motor_data_t> motors;
            add_motor_group_data(motors, leftMotors, {1, 2, 7}, "L-DRIVE");
            add_motor_group_data(motors, rightMotors, {10, 9, 17}, "R-DRIVE");
            motorTelemetry.update(motors);
            pros::delay(50);
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
    console.println("=== DRIVER CONTROL STARTED ===");
    
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
        chassis.curvature(leftY, rightX);
 
 
        // delay to save resources
        pros::delay(10);
    }
}