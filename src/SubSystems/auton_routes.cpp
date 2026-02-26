#include "auton_routes.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"

void runAutonomous(lemlib::Chassis& chassis) {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    
    // move forward 30 inches with timeout of 2000ms
    chassis.moveToPoint(0, 30, 2000);
    chassis.waitUntilDone();
}
