#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
pros::Motor mainIntake(-5, pros::MotorGearset::blue);
pros::Motor secondaryIntake(10, pros::MotorGearset::blue);

// pneumatics
pros::adi::Pneumatics toungeMech('C', false);
pros::adi::Pneumatics wing('D', false);
pros::adi::Pneumatics ballStopper1('A', false);
pros::adi::Pneumatics ballStopper2('B', false);


// motor groups
pros::MotorGroup leftMotors({-2, -3, -4}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({7, 8, 9}, pros::MotorGearset::blue);

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

enum class TripleState { BALL_STOP = 0, MID_GOAL = 1, HIGH_GOAL = 2 };
static const char* TRIPLE_STATE_NAMES[] = {"Ball Stop", "Mid Goal", "High Goal"};

static TripleState tripleState = TripleState::BALL_STOP;

static void applyTripleState(TripleState state) {
    tripleState = state;
    switch (tripleState) {
        case TripleState::BALL_STOP:
            ballStopper1.extend();
            ballStopper2.retract();
            break;
        case TripleState::MID_GOAL:
            ballStopper1.extend();
            ballStopper2.extend();
            break;
        case TripleState::HIGH_GOAL:
            ballStopper1.retract();
            ballStopper2.retract();
            break;
    }
    pros::lcd::print(1, "Triple: %s", TRIPLE_STATE_NAMES[static_cast<int>(tripleState)]);
}
// Brain button callbacks
static void onBrainBtnLeft() {
    // Cycle triple state
    auto next = static_cast<TripleState>((static_cast<int>(tripleState) + 1) % 3);
    applyTripleState(next);
}



void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "Drive: Ready");
    applyTripleState(tripleState);
    pros::lcd::register_btn0_cb(onBrainBtnLeft);
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

// Intake control functions
void ballStop() {
    // Ball stopping mode - extend piston to stop ball, run both intakes
    ballStopper1.extend();
    ballStopper2.retract();
    mainIntake.move(127);
    secondaryIntake.move(127);
}

void intakeMidGoal() {
    ballStopper1.extend();
    ballStopper2.extend();
    mainIntake.move(127);
    secondaryIntake.move(127);
}
void intakeHighGoal() {
    ballStopper1.retract();
    ballStopper2.retract();
    mainIntake.move(127);
    secondaryIntake.move(127);
}

void outtake() {
    ballStopper1.retract();
    ballStopper2.retract();
    mainIntake.move(-127);
    secondaryIntake.move(-127);
}

void intakeStop() {
    ballStopper1.retract();
    ballStopper2.retract();
    mainIntake.move(0);
    secondaryIntake.move(0);
}

void opcontrol() {
    bool wingState = false;
    DriveMode driveMode = DriveMode::TANK; // default to TANK (you can change)
    pros::lcd::print(0, "Drive: %s", DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);

    while (true) {
        // Display motor telemetry
        double mainIntakeRPM = mainIntake.get_actual_velocity();
        double mainIntakeTemp = mainIntake.get_temperature();
        double secondaryIntakeRPM = secondaryIntake.get_actual_velocity();
        double secondaryIntakeTemp = secondaryIntake.get_temperature();

        pros::lcd::print(3, "Intake1: %.1f RPM %.1f C", mainIntakeRPM, mainIntakeTemp);
        pros::lcd::print(4, "Intake2: %.1f RPM %.1f C", secondaryIntakeRPM, secondaryIntakeTemp);

        // joystick control
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


        // cycle drive mode with Y button (new press)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            driveMode = static_cast<DriveMode>((static_cast<int>(driveMode) + 1) % 3);
            pros::lcd::print(0, "Drive: %s", DRIVE_MODE_NAMES[static_cast<int>(driveMode)]);
        }

        // Brain buttons (hold) for intake control
        uint8_t brainBtns = pros::lcd::read_buttons();
        bool brainForward = brainBtns & LCD_BTN_CENTER;
        bool brainReverse = brainBtns & LCD_BTN_RIGHT;
        bool brainOverride = brainForward ^ brainReverse;
        if (brainForward && !brainReverse) {
            mainIntake.move(127);
            secondaryIntake.move(127);
        } else if (brainReverse && !brainForward) {
            mainIntake.move(-127);
            secondaryIntake.move(-127);
        }


        // Drive control
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
            wingState = !wingState;
            if (wingState)
                wing.extend();
            else
                wing.retract();
        }

        // button states (hold-based mapping)
        bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool A = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // Intake + piston control (priority: L1 > R2 > R1 > L2)
        if (brainOverride) {
            // brain buttons already handled intake power
        } else if (R1 && R2) {
            ballStop();
        } else if (R1) {
            intakeHighGoal();
        } else if (R2) {
            intakeMidGoal();
        } else if (L2) {
            outtake();
        } else {
            intakeStop();
        }
    // optional hold behavior for A can be added here if needed
        pros::delay(20);
    }
}
