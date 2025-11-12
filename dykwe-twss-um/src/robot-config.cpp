#include "vex.h"
#include "robot-config.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain  Brain;

controller Controller1 = controller(primary);
motor FL = motor(PORT11, ratio6_1, true);
motor ML = motor(PORT12, ratio6_1, true);
motor BL = motor(PORT13, ratio6_1, false);
motor FR = motor(PORT18, ratio6_1, true);
motor MR = motor(PORT19, ratio6_1, false);
motor BR = motor(PORT20, ratio6_1, false);

motor intake = motor(PORT1, ratio18_1, true);
motor outake = motor(PORT2, ratio18_1, false);

pneumatics mlm = pneumatics(Brain.ThreeWirePort.A);
pneumatics bears = pneumatics(Brain.ThreeWirePort.C);
pneumatics mg = pneumatics(Brain.ThreeWirePort.D);
pneumatics dp = pneumatics(Brain.ThreeWirePort.E);

inertial Inertial = inertial(PORT15);
//color ColorSensor = color(PORT5);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/*
Used to initialize code/tasks/devices added using tools in VEXcode Pro.
This should be called at the start of your int main function.
*/


void vexcodeInit( void ) {
// Nothing to initialize
}