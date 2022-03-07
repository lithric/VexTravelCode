// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//    Ports: 
//      (Vision Sensor)Eyeball-21
//      (Motor Group)Arm-3R-8F
//      (Drivetrain)Drivetrain[7:5][Reversed]-1-11-10-9-5Inert
//      (Digital Out)StickyPiston-A
//      (Motor Group)Fork-15R-16F
//
//    make sure to configure controller to drivetrain                                                                        
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"
#include "C:/$HTOP/CompCodeTemplate.h"

using namespace vex;
void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
    int Brain_precision = 0;
    int Console_precision = 0;
    double max = 200;

    Code::DriveInstructions RightSideAuto;
    RightSideAuto.speed = max;

    RightSideAuto.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(0,max,1), // drive to goal fast
        __wait(1070), // *************
        __grab(1), // grab goal
        __wait(max), // **************
        __drive(0,max,-1), // go back
        __wait(570), // **************
        __lift(30), // lift a bit
        __wait(300), // **************
        __stop(), // *****************
        __turn(90,max), // turn to move goal out of the way
         __grab(0), // release goal
         __lift(-30), // reset lift
        __turn(-118,max), // face the middle goal
        __drive(0,max,1), // drive to goal fast
        __wait(950), // *************
        __grab(1), // grab goal
        __drive(0,max,-1), // drive back
        __wait(1400), // *************
        __grab(0), // release goal
        __stop(), // *****************
        __drive(-8), // move away from goal
    };

    Code::DriveInstructions LeftSideAuto;
    LeftSideAuto.speed = max;

    LeftSideAuto.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(0,max,1), // drive to goal fast
        __wait(1400), //
        __grab(1), // grab goal
        __wait(200),
        __drive(0,max,-1),
        __wait(1400),
        __stop(),
    };

    Code::DriveInstructions RightSideWin;
    RightSideWin.speed = max;

    RightSideWin.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(0,max,1), // drive to goal fast
        __wait(700), //
        __grab(1), // grab goal
        __wait(200),
        __drive(0,max,-1),
        __wait(700),
        __stop(),
    };

    Code::DriveInstructions LeftSideWin;
    LeftSideWin.speed = max;

    LeftSideWin.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(0,max,1), // drive to goal fast
        __wait(550), //
        __grab(1), // grab goal
        __wait(200),
        __drive(0,max,-1),
        __wait(550),
        __stop(),
    };

    Code::DriveInstructions GroupAuton;
    GroupAuton.speed = max;

    GroupAuton.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(0,max,1), // drive to goal fast
        __wait(550), //
        __grab(1), // grab goal
        __wait(200),
        __drive(0,max,-1),
        __wait(550),
        __stop(),
    };

    switch(Code::autonMode) {
        case 0:
            RightSideAuto.start();
            replaceB(4,"right auto");
        break;
        case 1:
            LeftSideAuto.start();
            replaceB(4,"left auto");
        break;
        case 2:
            RightSideWin.start();
            replaceB(4,"right win");
        break;
        case 3:
            LeftSideWin.start();
            replaceB(4,"left win");
        break;
        case 4:
            GroupAuton.start();
            replaceB(4,"group auton");
        break;
        default:
            replaceB(4,"nothing");
        break;
    }
}

int main() {
  // create competition instance
  competition Competition;
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(Code::userControl);

  // Run the pre-autonomous function.
  // add extra programs here
  Code::autonNames.push_back("GroupAuton");
  Code::preAutonomous();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
