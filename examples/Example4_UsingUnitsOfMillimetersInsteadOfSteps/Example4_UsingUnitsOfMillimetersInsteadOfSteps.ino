
//      ******************************************************************
//      *                                                                *
//      *      Example using units of Millimeters, instead of Steps      *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// Many stepper motors are used in projects where the motions are linear 
// instead of rotational.  Examples are XY mechanisms such as plotters and 
// 3D printers.  In these cases issuing move commands in "Millimeters" is  
// much more intuitive than "Steps".  This program show how to work with  
// units in millimeters.
//
// Note: If your mechanism has a Homing Switch so that it can automatically  
// move to its "Home" position on startup, checkout the function:  
//    moveToHomeInMillimeters()
//  
//
// Documentation at:
//    https://github.com/Stan-Reifel/SpeedyStepper
//
//
// The motor must be connected to the Arduino with a driver board having a 
// "Step and Direction" interface.  It's VERY important that you set the 
// motor current first!  Read the driver board's documentation to learn how.

// ***********************************************************************


#include <SpeedyStepper.h>


//
// pin assignments
//
const int LED_PIN = 13;
const int MOTOR_STEP_PIN = 3;
const int MOTOR_DIRECTION_PIN = 4;


//
// create the stepper motor object
//
SpeedyStepper stepper;



void setup() 
{
  //
  // setup the LED pin and enable print statements
  //
  pinMode(LED_PIN, OUTPUT);   
  Serial.begin(9600);


  //
  // connect and configure the stepper motor to its IO pins
  //
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
}



void loop() 
{
  //
  // First you must tell the library "how many steps it takes to move
  // one millimeter".  In this example I'm using this stepper motor with 
  // a built in lead-screw:
  //    Stepper Motor with 38cm Lead Screw:  https://www.pololu.com/product/2690
  //
  stepper.setStepsPerMillimeter(25 * 1);    // 1x microstepping


  //
  // set the speed and acceleration rates for the stepper motor
  //
  stepper.setSpeedInMillimetersPerSecond(10.0);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(10.0);


  //
  // Use "absolute" positioning to:
  // Move to positions: 8mm, 16mm, 24mm, then back to the home position
  //
  stepper.moveToPositionInMillimeters(8.0);
  delay(1000);
  stepper.moveToPositionInMillimeters(16.0);
  delay(1000);
  stepper.moveToPositionInMillimeters(24.0);
  delay(1000);
  stepper.moveToPositionInMillimeters(0.0);
  delay(1000);


  //
  // increase the speed, then go to position -16mm
  //
  stepper.setSpeedInMillimetersPerSecond(25.0);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(25.0);
  stepper.moveToPositionInMillimeters(-16.0);
  delay(1000);


  //
  // Use "relative" positioning to make several more moves, 1mm each
  //
  for (int i = 0; i < 16; i++)
  { 
    stepper.moveRelativeInMillimeters(1.0);
    delay(100);
  }
 delay(3000);
}

