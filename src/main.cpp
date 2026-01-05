#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
pros::Motor mainIntake(10, pros::MotorGearset::blue);
pros::Motor scoringIntake(-18, pros::MotorGearset::blue);

// pneumatics
pros::adi::Pneumatics toungeMech('A', false);
pros::adi::Pneumatics longMid('b', false);

// motor groups
pros::MotorGroup leftMotors({-10, 0, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, 2, 20}, pros::MotorGearset::blue);

// drivetrain
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10,
                              lemlib::Omniwheel::NEW_325, 450, 8);

// motion controllers
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

// input curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// odom sensors
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, nullptr);

// chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// Drive mode selection
enum class DriveMode { TANK = 0, ARCADE = 1, CURVATURE = 2 };
static const char* DRIVE_MODE_NAMES[] = {"TANK", "ARCADE", "CURVATURE"};

static volatile bool auto_brake_on = true;
static const int AUTO_BRAKE_JOY_THRESHOLD = 15; // analog threshold to release hold

static void enableAutoBrake() {
    leftMotors.move(0);
    rightMotors.move(0);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::print(2, "AutoBrake: ON");
}

void initialize() {
    pros::lcd::initialize();

    pros::lcd::print(0, "Drive: Ready");
    enableAutoBrake();
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // your simple autonomous sequence (preserved)
    leftMotors.move(20);
    rightMotors.move(70);
    mainIntake.move(90);
    pros::delay(1675);

    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(850);
    mainIntake.move(-80);
    pros::delay(1100);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(100);
    leftMotors.brake();
    rightMotors.brake();
}

void opcontrol() {
    bool toungeMechState = false;
    DriveMode driveMode = DriveMode::ARCADE; // default to TANK (you can change)
    pros::lcd::print(0, "Drive: %s", DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);

    while (true) {
        // Display motor telemetry
        double mainIntakeRPM = mainIntake.get_actual_velocity();
        double mainIntakeTemp = mainIntake.get_temperature();
        double scoringIntakeRPM = scoringIntake.get_actual_velocity();
        double scoringIntakeTemp = scoringIntake.get_temperature();

        pros::lcd::print(3, "Main: %.1f RPM %.1f C", mainIntakeRPM, mainIntakeTemp);
        pros::lcd::print(4, "Score: %.1f RPM %.1f C", scoringIntakeRPM, scoringIntakeTemp);

        // joystick control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        if (auto_brake_on) {
            bool joystickMoved = false;
            switch (driveMode) {
                case DriveMode::TANK:
                    joystickMoved = (std::abs(leftY) > AUTO_BRAKE_JOY_THRESHOLD) || (std::abs(rightY) > AUTO_BRAKE_JOY_THRESHOLD);
                    break;
                case DriveMode::ARCADE:
                case DriveMode::CURVATURE:
                    joystickMoved = (std::abs(leftY) > AUTO_BRAKE_JOY_THRESHOLD) || (std::abs(rightX) > AUTO_BRAKE_JOY_THRESHOLD);
                    break;
            }

            if (joystickMoved) {
                // Release hold and allow drive commands to operate normally
                auto_brake_on = false;
                leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                pros::lcd::print(2, "AutoBrake: OFF");
            } else {
                leftMotors.move(0);
                rightMotors.move(0);
                pros::delay(20);
                continue; // skip rest of loop so nothing tries to move drivetrain
            }
        }

        // cycle drive mode with Y button (new press)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            driveMode = static_cast<DriveMode>((static_cast<int>(driveMode) + 1) % 3);
            pros::lcd::print(0, "Drive: %s", DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);
        }

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

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toungeMechState = !toungeMechState;
            if (toungeMechState)
                toungeMech.extend();
            else
                toungeMech.retract();
        }

        // button states
        bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        
        // brain screen touch for ball stopping intake
        pros::screen_touch_status_s_t touch_status = pros::screen::touch_status();
        bool brainTouch = (touch_status.touch_status == pros::E_TOUCH_PRESSED || 
                           touch_status.touch_status == pros::E_TOUCH_HELD);

        if (brainTouch) {
            // Ball stopping mode - same as R1+R2
            mainIntake.move(127);
            scoringIntake.move(-20);
        } else if (R1 && R2) {
            mainIntake.move(127);
            scoringIntake.move(-20);
        } else if (R1) {
            mainIntake.move(127);
            scoringIntake.move(127);
        } else if (R2) {
            mainIntake.move(127);
            scoringIntake.move(127);
        } else if (L1 && L2) {
            mainIntake.move(-127);
            scoringIntake.move(-127);
        } else {
            mainIntake.move(0);
            scoringIntake.move(0);
        }

        pros::delay(20);
    }
}
