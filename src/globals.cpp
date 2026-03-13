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
                              11,                          // 10 inch track width
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

pros::Imu imu(21);                 // Inertial sensor on port 19 
pros::Rotation horizontalEnc(-5);  // horizontal tracking wheel on port 20
pros::Rotation verticalEnc(-6);   // vertical tracking wheel on port 16 [reversed]
 pros::Distance rightDistance(11);   // right distance sensor on port 6
 pros::Distance leftDistance(11);   // left distance sensor on port 12
 pros::Distance frontDistance(11);  // front distance sensor on port 13
 pros::Distance backDistance(11);   // back distance sensor on port 11




// -------------------------------Odometry Setup--------------------------------
lemlib::TrackingWheel horizontal_tracking_wheel(
      &horizontalEnc,           // horizontal rotation sensor
      1.95, // using new 2" omni for horizontal tracking
      -1.25,           // back = negative | front = positive. Offset of 6.4 inches
      1
); 
lemlib::TrackingWheel vertical_tracking_wheel(
      &verticalEnc,             // vertical rotation sensor
      1.95, // using new 2" omni for vertical tracking
      0            // left = negative | right = positive. Offset of 0 inches
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15.5,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              2.5,  // derivative gain (kD)
                                              0,   // anti windup
                                              0.25, // small error range, in inches
                                              100,  // small error range timeout, in milliseconds
                                              2,   // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              0   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4.,      // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              15,     // derivative gain (kD)
                                              0,      // anti windup
                                              0.25,      // small error range, in inches
                                              100,     // small error range timeout, in milliseconds
                                              2,      // large error range, in inches
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