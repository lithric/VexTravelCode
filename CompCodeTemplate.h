#pragma once
// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//    Ports: 
//      (Vision Sensor)Eyeball-15
//      (Motor Group)Arm-3R-8F
//      (Drivetrain)Drivetrain-1-11-10-9-5Inert
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
#include <iostream>
#include <fstream>
// cout << typeid(variable).name() << endl;

// namespaces and imported methods
namespace Code
{
    using namespace vex;

    // define some useful changes
    #define RESERVE(key,val) key = val;auto RESERVE_ACT

    #define _USE_MATH_DEFINES

    #define anon(EXP) []() -> void{EXP;}
    #define func(type,EXP) () -> type{EXP;}
    #define opper(a,...) {std::vector<double>{a}, std::vector<double>{__VA_ARGS__}}
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

    // Begin project code
    public:
    class DriveInstructions {
        private:
        public:
        std::vector<std::vector<std::vector<double>>> instructions;
        double speed = 20;
        void start() {
            for (auto const& instruct : instructions) {
                std::vector<double> key = instruct[0];
                std::vector<double> val = instruct[1];
                switch ((int)key[0]) {
                    case 0: //__drive
                        val.size() > 1 ?
                        (void)Drivetrain.setDriveVelocity(val[1],percent):
                        (void)Drivetrain.setDriveVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Drivetrain.driveFor(forward,val[0],inches):
                        (void)Drivetrain.drive(forward);
                        Drivetrain.setDriveVelocity(val[0] == 0 && val[2] > 0 ? speed:-speed,percent);
                    break;
                    case 1: //__turn
                        val.size() > 1 ?
                        (void)Drivetrain.setTurnVelocity(val[1],percent):
                        (void)Drivetrain.setTurnVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Drivetrain.turnFor(right,val[0],degrees):
                        (void)Drivetrain.turn(val[2] > 0 ? right:left);
                        Drivetrain.setTurnVelocity(speed,percent);
                    break;
                    case 2: //__grab
                        StickyPiston.set(val[0] == 1 ? true:false);
                    break;
                    case 3: //__lift
                        val.size() > 1 ?
                        (void)Arm.setVelocity(val[1],percent):
                        (void)Arm.setVelocity(speed,percent);
                        std::abs(val[0]) > 0 ?
                        (void)Arm.spinFor(forward,val[0] * 7,degrees):
                        (void)Arm.spin(val[2] > 0 ? forward:reverse);
                        Arm.setVelocity(speed,percent);
                    break;
                    case 4: //__wait
                        wait(val[0],msec);
                    break;
                    case 5: //__stop
                        Drivetrain.stop();
                        Arm.stop();
                    break;
                    case 6: //__chase
                        //Drivetrain.drive(forward);
                        int centerFOV = 316/2; // the center of the screen
                        int offsetX = 30; // offset to match the center of the bot
                        std::map<std::string,int> objectBounds = {{"left",centerFOV+offsetX},{"right",centerFOV-offsetX}};
                        int lastSeen = 100; // callback number to establish where object was last seen
                        //Eyeball.takeSnapshot(Eyeball__REDGOAL); // get data about where object is
                        while(true) {
                            //Eyeball.takeSnapshot(Eyeball__REDGOAL);
                            if (Eyeball.largestObject.exists) {
                                auto mids = Eyeball.largestObject.centerX;
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
                                    lastSeen = 100; // no where
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

    void userControl(void) {
        Brain.Screen.clearScreen();
        // place driver control in this while loop
        Arm.setVelocity(50,percent);
        Drivetrain.setDriveVelocity(100,percent);
        Fork.setVelocity(100,percent);
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
                        Controller1.ButtonRight.pressed( anon(StickyPiston = !StickyPiston) );
                        Controller1.ButtonL1.pressing() ? Arm.spin(forward):
                        Controller1.ButtonL2.pressing() ? Arm.spin(reverse):
                        Arm.stop(hold);
                        Controller1.ButtonR1.pressing() ? Fork.spin(forward):
                        Controller1.ButtonR2.pressing() ? Fork.spin(reverse):
                        Fork.stop(hold);
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
}