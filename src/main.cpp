#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// intake motors
pros::Motor leftIntakeMotor(-5, pros::MotorGearset::blue);
pros::Motor rightIntakeMotor(10, pros::MotorGearset::blue);

// pneumatics
// tongue mechanism on ADI port D, default retracted
pros::adi::Pneumatics toungeMech('D', false);
// wing mechanism on ADI port C, default retracted
pros::adi::Pneumatics wing('C', false);

// drivetrain motor groups
// left motors on ports 2, 3, 4 (all reversed)
pros::MotorGroup leftMotors({-2, -3, -4}, pros::MotorGearset::blue);
// right motors on ports 7, 8, 9
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue);

// intake motor group
pros::MotorGroup intake({-5, 10}, pros::MotorGearset::blue);


// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // track width in inches
                              lemlib::Omniwheel::NEW_325, // 3.25" omnis
                              450, // drivetrain rpm
                              8 // horizontal drift (traction wheels)
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0,  // integral gain (kI)
                                            3,  // derivative gain (kD)
                                            3,  // anti windup
                                            1,  // small error range (inches)
                                            100, // small error timeout (ms)
                                            3,  // large error range (inches)
                                            500, // large error timeout (ms)
                                            20  // max acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range (degrees)
                                             100, // small error timeout (ms)
                                             3, // large error range (degrees)
                                             500, // large error timeout (ms)
                                             0 // max acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        linearController,
                        angularController,
                        sensors,
                        &throttleCurve,
                        &steerCurve);

// drive mode selection
enum class DriveMode { TANK = 0, ARCADE = 1, CURVATURE = 2 };
static const char* DRIVE_MODE_NAMES[] = {"TANK", "ARCADE", "CURVATURE"};

/**
 * Runs initialization code
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::lcd::print(0, "Drive: Ready");
}

/**
 * Runs during autonomous
 */
void autonomous() {
    // simple autonomous routine (unchanged)
    leftMotors.move(20);
    rightMotors.move(70);
    intake.move(90);
    pros::delay(1675);

    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(850);

    intake.move(-80);
    pros::delay(1100);

    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(100);

    leftMotors.brake();
    rightMotors.brake();
}

/**
 * Runs during driver control
 */
void opcontrol() {
    DriveMode driveMode = DriveMode::TANK; // default drive mode
    pros::lcd::print(0, "Drive: %s",
                     DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);

    while (true) {
        // intake motor telemetry
        double leftIntakeRPM = leftIntakeMotor.get_actual_velocity();
        double leftIntakeTemp = leftIntakeMotor.get_temperature();
        double rightIntakeRPM = rightIntakeMotor.get_actual_velocity();
        double rightIntakeTemp = rightIntakeMotor.get_temperature();

        pros::lcd::print(3, "Intake L: %.1f RPM %.1f C",
                         leftIntakeRPM, leftIntakeTemp);
        pros::lcd::print(4, "Intake R: %.1f RPM %.1f C",
                         rightIntakeRPM, rightIntakeTemp);
        pros::lcd::print(5, "Tounge: %s | Wing: %s",
                         toungeMech.is_extended() ? "Extended" : "Retracted",
                         wing.is_extended() ? "Extended" : "Retracted");

        // brain screen button input
        uint8_t brainBtns = pros::lcd::read_buttons();
        bool btnLeft = brainBtns & LCD_BTN_LEFT;
        bool btnCenter = brainBtns & LCD_BTN_CENTER;
        bool btnRight = brainBtns & LCD_BTN_RIGHT;

        // left button toggles wing
        static bool lastBtnLeft = false;
        if (btnLeft && !lastBtnLeft) {
            wing.toggle();
        }
        lastBtnLeft = btnLeft;

        // center button = intake forward
        // right button = intake reverse
        if (btnCenter) {
            intake.move(127);
        } else if (btnRight) {
            intake.move(-127);
        }

        // joystick input
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // cycle drive mode with Y button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            driveMode = static_cast<DriveMode>(
                (static_cast<int>(driveMode) + 1) % 3
            );
            pros::lcd::print(0, "Drive: %s",
                             DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);
        }

        // drive control
        switch (driveMode) {
            case DriveMode::TANK:
                chassis.tank(leftY, rightY);
                break;
            case DriveMode::ARCADE:
                chassis.arcade(leftY, rightX);
                break;
            case DriveMode::CURVATURE:
                chassis.curvature(leftY, rightX);
                break;
        }

        // R2 controls wing (hold = extend)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            wing.extend();
        } else {
            wing.retract();
        }

        // L2 toggles tongue mechanism
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            toungeMech.toggle();
        }

        // intake control
        // R1 = forward, L1 = reverse
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        pros::delay(20); // save resources
    }
}
