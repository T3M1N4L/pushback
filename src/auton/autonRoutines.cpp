#include "globals.h" 
#include "auton/autonFunctions.h"
#include "ltv.h"
#include "pros/rtos.hpp"




void skills_auton() {
chassis.setPose(-47.295, -1.25, 90);
chassis.moveToPose(-47.295, -47.5, 90, 2500);
tongue.extend();
chassis.turnToHeading(180, 1000);
chassis.moveToPose(-62, -47.5, 180, 2500);
}

void angular_test_auton() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 5000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 5000);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 5000);
    chassis.waitUntilDone();
    chassis.turnToHeading(360, 5000);
    chassis.waitUntilDone();
}   
void lateral_test_auton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0,48,0,8000);
}   