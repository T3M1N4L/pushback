#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <sys/_intsup.h>
// tongue mechanism on ADI port D, default retracted
pros::adi::Pneumatics toungeMech('E', false);
// wing mechanism on ADI port C, default retracted
pros::adi::Pneumatics wing('B', false);
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-1, -2, -7},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, 9, 17}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::MotorGroup intake({3, -18}, pros::MotorGearset::blue);
 
// Inertial Sensor on port 10
pros::Imu imu(4);
 
// tracking wheels
pros::Rotation verticalEnc(-6);
pros::Rotation horizontalEnc(-15);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, .5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -4.5);
 
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.95, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
 
// lateral motion controller
lemlib::ControllerSettings linearController(8.5  , // proportional gain (kP)
                                            0, // integral gain (kI)
                                            43, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              110 // maximum acceleration (slew)
);
 
// angular motion controller
lemlib::ControllerSettings angularController(5.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             42.4, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
 
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
 
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
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
 void compAuton(){
  // chassis.setPose(-46.5,0,180);
    // chassis.moveToPoint(0, 48, 2000, {.forwards = true}, false);

    // set position to x:0, y:0, heading:0
    chassis.setPose(-46.5, 0, 180);
    // turn to face heading 90 with a very long timeout
        toungeMech.extend();
    chassis.moveToPose(
        -47.5,// = 0
        -48.8, // y = 0
        270,
        5000, // timeout of 4000ms
        {.lead=0, .maxSpeed = 70},
        false
    );

     wing.extend();
     intake.move(127);
    chassis.moveToPose(
        -57, // x = 0
        -48.8, // y = 0
        270,
        5000, // timeout of 4000ms
        {.forwards = true},
        false
    );
    pros::delay(3000);
    chassis.moveToPose(
       -33, // x = 0
        -48.5,// y = 0
        270,
        5000, // timeout of 4000ms
        {.forwards = false},
        false);
    chassis.turnToHeading(
        135, // heading to turn to
        5000 // timeout in milliseconds
    );

}
void skillsAuton(){
     // chassis.setPose(-46.5,0,180);
    // chassis.moveToPoint(0, 48, 2000, {.forwards = true}, false);

    // set position to x:0, y:0, heading:0
    chassis.setPose(-46.5, 0, 180);
    // turn to face heading 90 with a very long timeout
        toungeMech.extend();
// align w the 
    chassis.moveToPose(
        -54, // x = 0
        -48, // y = 0
        270, // theta = 0
        5000, // timeout of 4000ms
        {.lead=0, .maxSpeed = 70},
        true
    );

     wing.extend();
     intake.move(127);

    pros::delay(3000);
            chassis.moveToPose(
       -12, // x = 0
        -61, // y = 0
        270, // theta = 0
        2000, // timeout of 4000ms
        {.forwards = false, .lead= 0.3},
        true
        );
  intake.move(0);
toungeMech.retract();
            chassis.moveToPose(
       40, // x = 0
        -57, // y = 0
        225, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = false, .lead=0},
        false
        );

    chassis.moveToPose(
        36, // x = 0
        -40, // y = 0
        45, // timeout of 4000ms
        6000,
        {.forwards = false},
        false
        );
    chassis.moveToPose(
        30, // x = 0
        -55, // y = 0
        90, // theta = 0
       5000, // timeout of 4000ms
        {.forwards = false},
        false
        );
        wing.retract();
        intake.move(127);
       pros::delay(3000);
        toungeMech.extend();
        wing.extend();
        chassis.moveToPose(
        64, // x = 0
        -56.5, // y = 0
        90, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = true},
        false
        );
        pros::delay(3000);
        chassis.moveToPose(
        30, // x = 0
        -56, // y = 0
        90, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = false},
        false
        );
        wing.retract();
        pros::delay(3000);
        toungeMech.retract();
        intake.move(0);
                chassis.moveToPose(
        54, // x = 0
        -56, // y = 0
        0, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = true},
        false
        );
                chassis.moveToPose(
        64, // x = 0
        43, // y = 0
        90, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = true, .lead=0},
        false
        );
        toungeMech.extend();
        wing.extend();
        intake.move(127);
        chassis.moveToPose(
        71// x = 0
        44.5, // y = 0
        90, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = false, .lead=0},
        false
        );
        pros::delay(3000);
                chassis.moveToPose(
        21,//x = 0
        44.5, // y = 0
        90, // theta = 0
        5000, // timeout of 4000ms
        {.forwards = false, .lead=0},
        false
        );
        wing.retract();
pros::delay(3000);
        toungeMech.retract();
        intake.move(0);
                chassis.moveToPose(
        41,//x = 0
        44,// y = 0
        180,// theta = 0
        5000, // timeout of 4000ms
        {.forwards = true, .lead=0},
        false
        );
        chassis.moveToPose(
        41,//x = 0
        24,//y = 0
        270,// theta = 0
        5000, // timeout of 4000ms
        {.forwards = true, .lead=0},
        false
        );
        chassis.moveToPose(
        -59,//x = 0
        24,//y = 0
        270,// theta = 0
        5000, // timeout of 4000ms
        {.forwards = true, .lead=0},
        false
        );
        chassis.turnToHeading(180, 5000);
        chassis.moveToPose(
        -63, // 0
        24,//y = 0
        270,// theta = 0
        5000, // timeout of 4000ms
        {.forwards = true, .lead=0},
        false
        );
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
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
 
        // R2 extends/retracts wing
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
            intake.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_velocity(-600);
        } else {
            intake.move_velocity(0);
        }
 
        // delay to save resources
        pros::delay(10);
    }
}