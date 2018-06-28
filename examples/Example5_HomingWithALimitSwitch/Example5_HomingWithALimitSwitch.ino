
//      ******************************************************************
//      *                                                                *
//      *   Example of finding the Home position using a limit switch    *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// Many stepper motor projects need to move to exact positions.  This usually 
// requires that the stepper motor library knows where the motor is before it 
// makes its first move.  To achieve this automatically, you will use a 
// Limit Switch that is pressed by the mechanism when the motor moves 
// all-the-way to one end of its travel.  This procedure called "Homing", is 
// executed when power is first applied.  A homing program moves the motor 
// toward the limit switch until the switch is pressed, then resets the  
// library so that the motor is now at position 0.
//
// Home automatically is easily done using one of these functions:
//    moveToHomeInSteps()
//    moveToHomeInRevolutions()
//    moveToHomeInMillimeters()
//
// This example shows how to home your device.  The mechanism in my example has 
// a stepper motor with a lead-screw.  On its left side is a limit switch that 
// the mechanism bumps into when the motor has reached its left most limit of 
// travel.  Note: The switch must be configured to go LOW when pressed at the
// home position. 
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
const int LIMIT_SWITCH_PIN = 9;


//
// create the stepper motor object
//
SpeedyStepper stepper;



void setup() 
{
  //
  // setup the LED pin, limit switch pin, and enable print statements
  //
  pinMode(LED_PIN, OUTPUT); 
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(9600);


  //
  // connect and configure the stepper motor to its IO pins
  //
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
}



void loop() 
{
  //
  // First you must tell the library "how many steps it takes to move one
  // millimeter".  My setup is configured with a lead-screw having 
  // 25 full-steps/mm and 1x microstepping.
  //
  stepper.setStepsPerMillimeter(25 * 1);    // 1x microstepping


  //
  // set the speed and acceleration rates for the stepper motor
  //
  stepper.setSpeedInMillimetersPerSecond(10.0);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(10.0);


  //
  // move the motor toward the limit switch to find the "Home" position
  //
  const float homingSpeedInMMPerSec = 5.0;
  const float maxHomingDistanceInMM = 380;   // since my lead-screw is 38cm long, should never move more than that
  const int directionTowardHome = -1;        // direction to move toward limit switch: 1 goes positive direction, -1 backward
  
  if(stepper.moveToHomeInMillimeters(directionTowardHome, homingSpeedInMMPerSec, maxHomingDistanceInMM, LIMIT_SWITCH_PIN) != true)
  {
    //
    // this code is executed only if homing fails because it has moved farther 
    // than maxHomingDistanceInMM and never finds the limit switch, blink the 
    // LED fast forever indicating a problem
    //
    while(true)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  }


  //
  // homing is now complete, the motor is stopped at position 0mm
  //
  delay(500);


  //
  // if you want your 0 origin someplace else, you can change it 
  //
  //stepper.setCurrentPositionInMillimeters(325);


  //
  // indicate the program has finished by blinking slowly forever
  //
  while(true)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(750);
    digitalWrite(LED_PIN, LOW);
    delay(750);
  }
}

