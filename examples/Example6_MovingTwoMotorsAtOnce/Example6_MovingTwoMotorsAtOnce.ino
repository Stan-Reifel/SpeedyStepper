
//      ******************************************************************
//      *                                                                *
//      *     Example of running two or more motors at the same time     *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// This example shows how to run two motors at the same time.  The previous 
// examples used function calls that were "blocking", meaning that they did 
// not return until the motion was complete.  This example show how to use
// a "polling" method instead, thus allowing you to do two or more things 
// at once.
//
// A possible area of concern is that running multiple motors simultaneously  
// may limit the top speed of each.  This library can generate a maximum of  
// about 12,500 steps per second using an Arduino Uno.  Running just one  
// motor in full step mode, with a 200 steps per rotation motor, the maximum  
// speed is about 62 RPS or 3750 RPM (most stepper motor can't go this fast). 
// Driving one motor in half step mode, a maximum speed of 31 RPS or 1875 RPM  
// can be reached.  In quarter step mode about 15 RPS or 937 RPM.  Running  
// multiple motors at the same time will reduce the maximum speed, for example  
// running two motors will reduce the maximum step rate by half or more.
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
const int MOTOR_X_STEP_PIN = 3;
const int MOTOR_X_DIRECTION_PIN = 4;
const int MOTOR_Y_STEP_PIN = 10;
const int MOTOR_Y_DIRECTION_PIN = 5;


//
// create two stepper motor objects, one for each motor
//
SpeedyStepper stepperX;
SpeedyStepper stepperY;



void setup() 
{
  //
  // setup the LED pin and enable print statements
  //
  pinMode(LED_PIN, OUTPUT);   
  Serial.begin(9600);


  //
  // connect and configure the stepper motors to their IO pins
  //
  stepperX.connectToPins(MOTOR_X_STEP_PIN, MOTOR_X_DIRECTION_PIN);
  stepperY.connectToPins(MOTOR_Y_STEP_PIN, MOTOR_Y_DIRECTION_PIN);
}


void loop() 
{
  //
  // setup the speed, acceleration and number of steps to move for the 
  // X motor, note: these commands do not start moving yet
  //
  stepperX.setSpeedInStepsPerSecond(100);
  stepperX.setAccelerationInStepsPerSecondPerSecond(100);
  stepperX.setupRelativeMoveInSteps(200);


  //
  // setup the speed, acceleration and number of steps to move for the 
  // Y motor
  //
  stepperY.setSpeedInStepsPerSecond(100);
  stepperY.setAccelerationInStepsPerSecondPerSecond(100);
  stepperY.setupRelativeMoveInSteps(-200);


  //
  // now execute the moves, looping until both motors have finished
  //
  while((!stepperX.motionComplete()) || (!stepperY.motionComplete()))
  {
    stepperX.processMovement();
    stepperY.processMovement();
  }


  //
  // now that the rotations have finished, delay 1 second before starting 
  // the next move
  //
  delay(1000);


  //
  // use the function below to move two motors with speed coordination
  // so that both stop at the same time, even if one moves farther than
  // the other
  //
  long stepsX = -200 * 1;
  long stepsY = 200 * 5;
  float speedInStepsPerSecond = 500;
  float accelerationInStepsPerSecondPerSecond = 500;
  moveXYWithCoordination(stepsX, stepsY, speedInStepsPerSecond, accelerationInStepsPerSecondPerSecond);
  delay(3000);
}



//
// move both X & Y motors together in a coordinated way, such that they each 
// start and stop at the same time, even if one motor moves a greater distance
//
void moveXYWithCoordination(long stepsX, long stepsY, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond)
{
  float speedInStepsPerSecond_X;
  float accelerationInStepsPerSecondPerSecond_X;
  float speedInStepsPerSecond_Y;
  float accelerationInStepsPerSecondPerSecond_Y;
  long absStepsX;
  long absStepsY;

  //
  // setup initial speed and acceleration values
  //
  speedInStepsPerSecond_X = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond;
  
  speedInStepsPerSecond_Y = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond;


  //
  // determine how many steps each motor is moving
  //
  if (stepsX >= 0)
    absStepsX = stepsX;
  else
    absStepsX = -stepsX;
 
  if (stepsY >= 0)
    absStepsY = stepsY;
  else
    absStepsY = -stepsY;


  //
  // determine which motor is traveling the farthest, then slow down the
  // speed rates for the motor moving the shortest distance
  //
  if ((absStepsX > absStepsY) && (stepsX != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absStepsY / (float) absStepsX;
    speedInStepsPerSecond_Y = speedInStepsPerSecond_Y * scaler;
    accelerationInStepsPerSecondPerSecond_Y = accelerationInStepsPerSecondPerSecond_Y * scaler;
  }
  
  if ((absStepsY > absStepsX) && (stepsY != 0))
  {
    //
    // slow down the motor traveling less far
    //
    float scaler = (float) absStepsX / (float) absStepsY;
    speedInStepsPerSecond_X = speedInStepsPerSecond_X * scaler;
    accelerationInStepsPerSecondPerSecond_X = accelerationInStepsPerSecondPerSecond_X * scaler;
  }

  
  //
  // setup the motion for the X motor
  //
  stepperX.setSpeedInStepsPerSecond(speedInStepsPerSecond_X);
  stepperX.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_X);
  stepperX.setupRelativeMoveInSteps(stepsX);


  //
  // setup the motion for the Y motor
  //
  stepperY.setSpeedInStepsPerSecond(speedInStepsPerSecond_Y);
  stepperY.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_Y);
  stepperY.setupRelativeMoveInSteps(stepsY);


  //
  // now execute the moves, looping until both motors have finished
  //
  while((!stepperX.motionComplete()) || (!stepperY.motionComplete()))
  {
    stepperX.processMovement();
    stepperY.processMovement();
  }
}


