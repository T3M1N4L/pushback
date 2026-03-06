#include "main.h"
#include "globals.h"
#include "auton/autonRoutines.h"
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
    {"Right Auton", right_auton, "", 0},          // Red
    {"Left Auton", left_auton, "", 0},            // Red
    {"Carry Auton", carry_auton, "", 60},         // Yellow
    {"Elim Auton", elim_auton, "", 300},          // Magenta
    {"AWP Auton", awp_auton, "", 220},            // Blue
    {"Skills Auton", skills_auton, "", 120},      // Green
    {"Do Nothing", doNothing, "", 180}            // Cyan
}, &controller);

// Create image widget
rd::Image teamLogo("/img/gengy.bin", "Gengar");

// Create position display view
rd::Position position(&chassis, {"skills.bin", "match.bin"}, {"Skills", "Match"}, &controller);

// Create motor telemetry screen
// Motor groups show all motors, individual motors show separately
rd::MotorTelemetry motorTelemetry("Motor Telemetry", leftMotors.size() + rightMotors.size() + 2, &controller);

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
    
    // Background task to update motor telemetry (shows all motors from groups + individual motors)
    pros::Task telemetryTask([&]() {
        while (true) {
            std::vector<rd::motor_data_t> all_motors;
            uint32_t current_time = pros::millis();
            
            // Helper lambda to build motor data from C API
            auto build_motor_data = [&current_time](int8_t port, const char* name) -> rd::motor_data_t {
                rd::motor_data_t data;
                int8_t abs_port = std::abs(port);
                data.port = abs_port;
                data.name = name;
                
                // Get raw values using C API directly
                double raw_vel = pros::c::motor_get_actual_velocity(abs_port);
                double raw_temp = pros::c::motor_get_temperature(abs_port);
                int32_t raw_current_ma = pros::c::motor_get_current_draw(abs_port);
                
                // Connection detection
                bool vel_valid = (raw_vel != PROS_ERR_F && !std::isnan(raw_vel) && !std::isinf(raw_vel));
                bool temp_valid = (raw_temp != PROS_ERR_F && !std::isnan(raw_temp) && raw_temp > 0.0);
                bool current_valid = (raw_current_ma != PROS_ERR);
                int valid_count = (vel_valid ? 1 : 0) + (temp_valid ? 1 : 0) + (current_valid ? 1 : 0);
                bool is_connected = (valid_count >= 2);
                
                data.connected = is_connected;
                
                if (is_connected) {
                    // Get gearset
                    pros::motor_gearset_e_t gearset = pros::c::motor_get_gearing(abs_port);
                    int gear_idx = (int)gearset;
                    if (gear_idx < 0 || gear_idx > 2) gear_idx = 2;
                    data.gearing = gear_idx;
                    
                    // Velocity: reverse if port was negative
                    float velocity = (float)raw_vel;
                    if (port < 0) velocity = -velocity;
                    
                    data.velocity_rpm = velocity;
                    data.power_w = (float)pros::c::motor_get_power(abs_port);
                    data.current_a = (float)(raw_current_ma / 1000.0);
                    data.temp_c = (float)raw_temp;
                    data.torque_nm = (float)pros::c::motor_get_torque(abs_port);
                } else {
                    // Disconnected
                    data.gearing = 2;
                    data.velocity_rpm = 0;
                    data.power_w = 0;
                    data.current_a = 0;
                    data.temp_c = 0;
                    data.torque_nm = 0;
                }
                
                return data;
            };
            
            // Add ALL motors from leftMotors group
            auto left_ports = leftMotors.get_port_all();
            for (auto port : left_ports) {
                all_motors.push_back(build_motor_data(port, "LFT"));
            }
            
            // Add ALL motors from rightMotors group
            auto right_ports = rightMotors.get_port_all();
            for (auto port : right_ports) {
                all_motors.push_back(build_motor_data(port, "RGT"));
            }
            
            // Add individual motors
            // Add individual motors (use their actual ports, respecting reversal)
            all_motors.push_back(build_motor_data(intakeMotor.get_port(), "INT"));
            all_motors.push_back(build_motor_data(topMotor.get_port(), "TOP"));
            
            motorTelemetry.update(all_motors);
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
