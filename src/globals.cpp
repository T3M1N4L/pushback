#include "main.h"
#include <sstream>
#include <string>
#include "globals.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// ------------------------------Drivetrain--------------------------------
pros::MotorGroup rightMotors({-9, // front most motor (port 8)
                             8, // top middle motor (port 9) [reversed]
                             7  // middle motor (port 10)
}, pros::MotorGears::blue); 
pros::MotorGroup leftMotors({2, // front most motor (port 1)
                              -3, // top middle motor (port 2) [reversed]
                             -4  // middle motor (port 3)
}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                              &rightMotors,                // right motor group
                              10,                          // 10 inch track width
                              lemlib::Omniwheel::NEW_325,  // using new 3.25" omnis
                              450,                         // drivetrain rpm is 450
                              2                            // horizontal drift is 2. traction wheels would make it 8
);

// ------------------------ Pneumatics --------------------------
pros::adi::Pneumatics tongue('A', false); // tongue mech on port E, retracted initially
pros::adi::Pneumatics wing('B', false);   // wing mech on port B, retracted initially
pros::adi::Pneumatics pulldown('C', true); // middle goal pulldown on port C, extended initially

// ------------------------ Intake --------------------------
pros::Motor intakeMotor(-10, pros::v5::MotorGears::blue); // intake motor on port 18
pros::Motor topMotor(1, pros::v5::MotorGears::blue);     // top roller motor on port 7

// ------------------------ Sensors and Odometry ------------------------

pros::Imu imu(19);                 // Inertial sensor on port 19 
pros::Rotation horizontalEnc(20);  // horizontal tracking wheel on port 20
pros::Rotation verticalEnc(-16);   // vertical tracking wheel on port 16 [reversed]
pros::Distance rightDistance(6);   // right distance sensor on port 6
pros::Distance leftDistance(12);   // left distance sensor on port 12
pros::Distance frontDistance(13);  // front distance sensor on port 13
pros::Distance backDistance(11);   // back distance sensor on port 11




// -------------------------------Odometry Setup--------------------------------
lemlib::TrackingWheel horizontal_tracking_wheel(
      &horizontalEnc,           // horizontal rotation sensor
      lemlib::Omniwheel::NEW_2, // using new 2" omni for horizontal tracking
      -6.37728606611,           // back = negative | front = positive. Offset of 6.4 inches
      1
); 
lemlib::TrackingWheel vertical_tracking_wheel(
      &verticalEnc,             // vertical rotation sensor
      lemlib::Omniwheel::NEW_2, // using new 2" omni for vertical tracking
      0.110912828986            // left = negative | right = positive. Offset of 0.11 inches
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              40,  // derivative gain (kD)
                                              3,   // anti windup
                                              0.5, // small error range, in inches
                                              50,  // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              35   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3,      // proportional gain (kP)
                                              0.0015, // integral gain (kI)
                                              24,     // derivative gain (kD)
                                              3,      // anti windup
                                              1,      // small error range, in inches
                                              25,     // small error range timeout, in milliseconds
                                              3,      // large error range, in inches
                                              100,    // large error range timeout, in milliseconds
                                              0       // maximum acceleration (slew)
);






// -------------------------------Chassis Setup--------------------------------
lemlib::ExpoDriveCurve throttleCurve(5,    // joystick deadband out of 127
                                     10,   // minimum output where drivetrain will move out of 127
                                     1.01  // expo curve gain
);

lemlib::ExpoDriveCurve steerCurve(5,    // joystick deadband out of 127
                                  10,   // minimum output where drivetrain will move out of 127
                                  1.02  // expo curve gain
);


lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttleCurve, &steerCurve); 


bool midgoal_first = false;