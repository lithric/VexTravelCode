// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//    Ports: 
//      (Vision Sensor)Eyeball-15
//      (Motor Group)Arm-3R-8F
//      (Drivetrain)Drivetrain-1-11-10-9-4Inert
//      (Digital Out)StickyPiston-A
//
//    make sure to configure controller to drivetrain                                                                        
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"
#include <math.h>
#include <cmath>
#include <string>
#include <tgmath.h>
#include <thread>
#include <vector>
#include <map>
#include <typeinfo>
// cout << typeid(variable).name() << endl;

// namespaces and imported methods
using namespace vex;
using std::vector;
using std::string;
using std::map;
using std::abs;

// define some useful changes
#define RESERVE(key,val) key = val;auto RESERVE_ACT

#define _USE_MATH_DEFINES

#define anon(EXP) []() -> void{EXP;}
#define func(type,EXP) () -> type{EXP;}
#define opper(a,...) {vector<double>{a}, vector<double>{__VA_ARGS__}}
#define toggle(key,id) key.pressed( anon(singleAct[id] = true) )

#define setActionVelocity(act,...) set ## act ## Velocity(__VA_ARGS__)
#define actionFor(act,...) act ## For(__VA_ARGS__)

#define printB(a) Brain.Screen.print(a);Brain.Screen.newLine()
#define replaceB(a,b) Brain.Screen.clearLine(a);Brain.Screen.setCursor(a,1);Brain.Screen.print(b)
#define clearB(a) Brain.Screen.clearLine(a)
#define __drive(...) opper(0,__VA_ARGS__)
#define __turn(...) opper(1,__VA_ARGS__)
#define __grab(...) opper(2,__VA_ARGS__)
#define __lift(...) opper(3,__VA_ARGS__)
#define __wait(...) opper(4,__VA_ARGS__)
#define __stop(...) opper(5,__VA_ARGS__)
#define __chase(...) opper(6,__VA_ARGS__)
#define printIf(cond,T,F) anon(if(cond){Brain.Screen.print(T);Brain.Screen.newLine();}else{Brain.Screen.print(F);Brain.Screen.newLine();})

// assign global variables
    double rampThreshold = 20; // 20deg
    bool debugMode = false;
    vector<bool> singleAct = {false,false,false,false,false,false};
    int debugVal = 0;
    double MaxMotorSpeed = 200; // 200rpm
    double globalVelocity = 200; // 200rpm
    double globalAcceleration = 60; // 60cm/s^2
    double wheelDiam = 10.16;
    double localTime = 0.00;
    double botDiam = 39;
    double driveVel = 100;
    bool torqueMode = false;
//

// Begin project code

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
}

class DriveInstructions {
    private:
    public:
    vector<vector<vector<double>>> instructions;
    double speed = 20;
    void start() {
        for (auto const& instruct : instructions) {
            vector<double> key = instruct[0];
            vector<double> val = instruct[1];
            switch ((int)key[0]) {
                case 0: //__drive
                    val.size() > 1 ?
                    (void)Drivetrain.setDriveVelocity(val[1],percent):
                    (void)Drivetrain.setDriveVelocity(speed,percent);
                    abs(val[0]) > 0 ?
                    (void)Drivetrain.driveFor(forward,val[0],inches):
                    (void)Drivetrain.drive(val[2] > 0 ? forward:reverse);
                    Drivetrain.setDriveVelocity(speed,percent);
                break;
                case 1: //__turn
                    val.size() > 1 ?
                    Drivetrain.setTurnVelocity(val[1],percent):
                    Drivetrain.setTurnVelocity(speed,percent);
                    if (abs(val[0]) > 0) {
                        Drivetrain.turnFor(right,val[0],degrees);
                    }
                    else {
                        Drivetrain.turn(val[2] > 0 ? right:left);
                    }
                    Drivetrain.setTurnVelocity(speed,percent);
                break;
                case 2: //__grab
                    StickyPiston.set(val[0] == 1 ? true:false);
                break;
                case 3: //__lift
                    val.size() > 1 ?
                    Arm.setVelocity(val[1],percent):
                    Arm.setVelocity(speed,percent);

                    if (abs(val[0]) > 0) {
                        Arm.spinFor(forward,val[0] * 7,degrees);
                    }
                    else {
                        Arm.spin(val[2] > 0 ? forward:reverse);
                    }
                    Arm.setVelocity(speed,percent);
                break;
                case 4: //__wait
                    wait(val[0],seconds);
                break;
                case 5: //__stop
                    Drivetrain.stop();
                break;
                case 6: //__chase
                    //Drivetrain.drive(forward);
                    int centerFOV = 316/2; // the center of the screen
                    int offsetX = 30; // offset to match the center of the bot
                    map<string,int> objectBounds = {{"left",centerFOV+offsetX},{"right",centerFOV-offsetX}};
                    int lastSeen = 100; // callback number to establish where object was last seen
                    Eyeball.takeSnapshot(Eyeball__REDGOAL); // get data about where object is
                    while(true) {
                        // Eyeball.takeSnapshot(Eyeball__REDGOAL);
                        // Eyeball.largestObject.exists ? 
                        // Eyeball.largestObject.width < val[0] ?
                        // Eyeball.largestObject.centerX > objectBounds["left"] ?
                        // anon()
                        // :
                        // :
                        // :
                        if (Eyeball.largestObject.exists) {
                            if (Eyeball.largestObject.width < val[0]) {
                                if (Eyeball.largestObject.centerX > centerFOV + offsetX) {
                                    LeftDriveSmart.spin(forward);RightDriveSmart.stop();
                                    lastSeen = 1; // to the right
                                }
                                else if (Eyeball.largestObject.centerX < centerFOV - offsetX) {
                                    LeftDriveSmart.stop();RightDriveSmart.spin(forward);
                                    lastSeen = -1; // to the left
                                }
                                else {
                                    LeftDriveSmart.spin(forward);RightDriveSmart.spin(forward);
                                    lastSeen = 0; // straight ahead
                                }
                            }
                            else {
                                lastSeen = 100;
                            }
                        }
                        else {
                            switch(lastSeen) {
                                case 1:
                                    LeftDriveSmart.spin(forward);
                                    RightDriveSmart.spin(reverse);
                                break;
                                case -1:
                                    LeftDriveSmart.spin(reverse);
                                    RightDriveSmart.spin(forward);
                                break;
                                case 0:
                                    LeftDriveSmart.spin(forward);
                                    RightDriveSmart.spin(forward);
                                break;
                                default:
                                    LeftDriveSmart.stop();
                                    RightDriveSmart.stop();
                                break;
                            }
                        }
                        wait(0.05,seconds);
                    }
                    Drivetrain.stop();
                break;
            }
        }
    }
};
void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
int Brain_precision = 0;
int Console_precision = 0;


    DriveInstructions goingAroundTheField;
    goingAroundTheField.speed = 100;

    goingAroundTheField.instructions = {
        __drive(145),
        __turn(90),
        // first turn made
        __drive(40),
        __turn(-90), // turning maneuver to get around the corner
        __drive(13),
        __turn(90), // back on track
        // second turn made
        __drive(200),
        __turn(90),
        // third turn made
        __drive(200),
        __turn(90),
        // fourth turn made
        __drive(200),
    };

    DriveInstructions grabSomething;
    grabSomething.speed = 100;

    grabSomething.instructions = {
        __grab(0), // precautionary measure: opens the vice
        __drive(43), // drive to goal fast
        __drive(10,38), // slowly approach goal
        __grab(1), // grab goal
        __drive(-47), // drive back
        __turn(90), // turn to move goal out of the way
         __grab(0), // release goal
        __turn(-117), // face the middle goal
        __drive(48), // drive to goal fast
        __drive(12,38), // slowly approach goal
        __grab(1), // grab goal
        __drive(-48), // drive back
        __grab(0), // release goal
        __drive(-8), // move away from goal
    };

    DriveInstructions testDrive;
    testDrive.speed = 100;

    testDrive.instructions = {
        __chase(30)
    };

    //goingAroundTheField.start();
    grabSomething.start();
    //testDrive.start();
}

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  Arm.setVelocity(50,percent);
  Drivetrain.setDriveVelocity(100,percent);
  while (true) {
    wait(20, msec);
    while(true) {
        Controller1.ButtonY.pressed( anon(singleAct[0] = true) );
        Controller1.ButtonY.released( anon(
                if(singleAct[0]) {
                    debugMode = !debugMode;
                    singleAct[0] = false;
                    printB(debugMode ? "Debug Mode ON":"Debug Mode OFF");
                }
            ));
        Controller1.ButtonX.pressed( anon(singleAct[1] = true) );
        Controller1.ButtonX.released( anon(if(singleAct[1] && debugMode){Brain.Screen.clearScreen();singleAct[1] = false;}) );
        Controller1.ButtonL1.pressed( anon(singleAct[2] = true) );
        Controller1.ButtonL1.released( anon(if(singleAct[2] && debugMode){printB("hello");singleAct[2] = false;}) );
        Controller1.ButtonR1.pressed( anon(singleAct[3] = true) );
        Controller1.ButtonR1.released( anon(if(singleAct[3] && debugMode){driveVel += 5;replaceB(1,driveVel);singleAct[3] = false;}) );
        Controller1.ButtonR2.pressed( anon(singleAct[4] = true) );
        Controller1.ButtonR2.released( anon(if(singleAct[4] && debugMode){driveVel -= 5;replaceB(1,driveVel);singleAct[4] = false;}) );

        if (!debugMode) {
            Controller1.ButtonUp.pressed( anon(StickyPiston = !StickyPiston) );
            Controller1.ButtonL1.pressing() ? Arm.spin(forward):
            Controller1.ButtonL2.pressing() ? Arm.spin(reverse):
            Arm.stop(hold);
            Controller1.ButtonX.pressed( anon(singleAct[5] = true) );
            Controller1.ButtonX.released( anon(if (singleAct[5]) {torqueMode = !torqueMode;singleAct[5] = false;}) );
            //Controller1.toggle(ButtonX,6).act();
            if (torqueMode) {
                Drivetrain.stop();
                wait(100,msec);
            }
        }
        // replaceB(2,Controller1.Axis1.position());
        // replaceB(3,Controller1.Axis2.position());
        // replaceB(4,Controller1.Axis3.position());
    }
  }
}

int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}