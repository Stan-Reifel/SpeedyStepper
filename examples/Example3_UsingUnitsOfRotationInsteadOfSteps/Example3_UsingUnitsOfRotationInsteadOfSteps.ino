
//      ******************************************************************
//      *                                                                *
//      *        Example using units of Rotation, instead of Steps       *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// Frequently issuing move commands in "Steps" is not as intuitive as other 
// units, such as Millimeters or Rotations.  This example shows how to work 
// in Rotations, rather than Steps.  Typical values sent to a move command 
// might be 1.0 for one revolution forward, or -2.5 for two and a half 
// rotations backward.
//
// Note: If your mechanism has a Homing Switch so that it can automatically  
// move to its "Home" position on startup, checkout the function:  
//    moveToHomeInRevolutions()
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
  // first you must tell the library "how many steps makes one rotation"
  //     use 200  if your motor has 200 steps/rev, with 1x microstepping
  //     use 400  if your motor has 200 steps/rev, with 2x microstepping
  //     use 800  if your motor has 200 steps/rev, with 4x microstepping
  //     use 1600 if your motor has 200 steps/rev, with 8x microstepping
  //
  stepper.setStepsPerRevolution(200);


  //
  // set the speed and acceleration rates for the stepper motor
  //
  stepper.setSpeedInRevolutionsPerSecond(1.0);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(1.0);


  //
  // Use "absolute" positioning to:
  // Rotate 1/4 turn to the 3 o'clock position, then to 6 o'clock,
  // then 9 o'clock, finally return to the home position
  //
  stepper.moveToPositionInRevolutions(0.25);
  delay(1000);
  stepper.moveToPositionInRevolutions(0.5);
  delay(1000);
  stepper.moveToPositionInRevolutions(0.75);
  delay(1000);
  stepper.moveToPositionInRevolutions(0.0);
  delay(1000);


  //
  // increase the speed, then go to position -3.5 rotations
  //
  stepper.setSpeedInRevolutionsPerSecond(2.5);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(2.0);
  stepper.moveToPositionInRevolutions(-3.5);
  delay(1000);


  //
  // Use "relative" positioning to make several more moves
  //
  stepper.moveRelativeInRevolutions(1.0);
  delay(1000);

  for (int i = 0; i < 10; i++)
  { 
    stepper.moveRelativeInRevolutions(0.1);
    delay(100);
  }
  delay(900);

  stepper.moveRelativeInRevolutions(1.5);
  delay(3000);
}

