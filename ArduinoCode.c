//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ arm  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* This is a program for controlling the two linear actuators on the robotic arm
 * Takes input from serial: <upper speed int> <lower speed int> 314
 *    Upper speed and lower speed are expected to be in the range [-100, 100].
 *    Numbers outside of this range will be treated as the maximum
 * Will deactivate all movement if too long passes before receiving another update
 * Author: Gregory O'Hagan
 */

#include <Stepper.h>
#include <PID_v1.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include "A4988.h"
#include <math.h>

// arm defines
#define UPPER_PWM 7
#define UPPER_DIR 8

#define LOWER_PWM 12
#define LOWER_DIR 13

#define STEPPER_ENABLE 11
#define STEPPER_DIR 10
#define STEPPER_PULSE 9

// gripper defines
//Pitch is up/down motion of gripper
#define PITCH_PWM 25
#define PITCH_DIR 47

//Set RPM and step number for Pitch Stepper
#define RPM 25
#define STEPS 400

//Spin is the rotational motion of the gripper
#define SPIN_PWM 24
#define SPIN_DIR 46

//Claw is the opening/closing of the fingers
#define CLAW_PWM 5
#define CLAW_DIR 6

//Define encoder digits
#define ENCODER_A 50
#define ENCODER_B 51

#define LIMITSWITCH 30

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ global variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// input coordinates
int x_coordinate;
int y_coordinate;
int z_coordinate;

// Upper arm variables
int upperDirection = 0;
double upper_max_lim = 900;     // angle = 49deg
double upper_min_lim = 620;     // angle = 115deg
int theta3;                     // upper arm
double Setpoint_upper;          // target upper arm position
double Input_upper;              // upper arm pot reading
double Output_upper;             // PID speed control
int error_upper = 10;           // error boundary
int arm_count = 0;
double average_upper;

// Lower arm variables
int lowerDirection = 0;
double lower_max_lim = 400;     // angle = 20deg
double lower_min_lim = 200;     // angle = 90deg
int theta2;                     // lower arm
double Setpoint_lower;          // target lower arm position
double Input_lower;             // lower arm pot reading
double Output_lower;            // PID speed control
int error_lower = 10;           // error boundary
double average_lower;

// claw variables
int error_spin = 30;            // position error boundary for spin
int currentStateCLK = 0;            // encoder variable
int stateB;                     // encoder variable
int average_dir = 0;            // encoder counter
int previousStateCLK = 0;           // encoder variable
int Setpoint_claw;              // claw goal position
int claw_max_pos = 400;         // claw max limit
int claw_min_pos = 0;           // claw max limit
int clawState = 0;                  // 0 = off, 1 = on
int clawDirection = 0;          // direction
int count2 = 0;                 // average counter for claw function
int claw_position = 0;          // current position of claw
int Limit = 0;                  // used for claw initialization

// pitch variables
double average_pitch;           // average of pitch position readings
double Setpoint_pitch;          // pitch goal position
double Input_pitch;             // pitch position reading
int count = 0;                  // average counter for pitch function
int theta4;                     // pitch angle
int error_pitch = 15;           // position error boundary for spin

// spin variables
double Setpoint_spin;
double Input_spin;

//Completion flags
bool upperarmDone = 0;
bool lowerarmDone = 0;
bool pitchDone = 0;
bool clawDone = 0;
bool Event = 0;

// serial read variables
int input1;						// x or theta1
int input2;						// y or theta2
int input3;						// z or theta3
int input4;						// gripper coordinate
int input_count = 0;
char data;
String command = "";
bool restart = 0;

// inverse kinematics variables
int a1, a2, a3;
int wx, wy, wz;
int r1, r2, r3;
int X0, Y0, Z0;
int phi1, phi2;
int length1, length2;

// stepper setup
A4988 stepper_pitch(STEPS, PITCH_DIR, PITCH_PWM);
A4988 stepper_spin(STEPS, SPIN_DIR, SPIN_PWM);

// PID functions
//Specify the links and initial tuning parameters
double Kp=2, Ki=0, Kd=0;
PID myPID_upper_rev(&average_upper, &Output_upper, &Setpoint_upper, Kp, Ki, Kd, REVERSE);
PID myPID_upper_fwd(&average_upper, &Output_upper, &Setpoint_upper, Kp, Ki, Kd, DIRECT);
PID myPID_lower_rev(&average_lower, &Output_lower, &Setpoint_lower, Kp, Ki, Kd, REVERSE);
PID myPID_lower_fwd(&average_lower, &Output_lower, &Setpoint_lower, Kp, Ki, Kd, DIRECT);

// Setup code, runs once
void setup() {
  Serial.begin(9600); // Opens serial port, sets data rate to 9600 bps

  // arm setup
  pinMode(UPPER_PWM, OUTPUT);
  pinMode(UPPER_DIR, OUTPUT);
   
  pinMode(LOWER_PWM, OUTPUT);
  pinMode(LOWER_DIR, OUTPUT);

  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, HIGH);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_PULSE, OUTPUT);
  digitalWrite(STEPPER_PULSE, LOW);

  //turn the PID on
  myPID_upper_rev.SetMode(AUTOMATIC);
  myPID_upper_fwd.SetMode(AUTOMATIC);
  myPID_lower_rev.SetMode(AUTOMATIC);
  myPID_lower_fwd.SetMode(AUTOMATIC);

  // gripper setup
  pinMode(PITCH_PWM, OUTPUT);
  pinMode(PITCH_DIR, OUTPUT);
 
  pinMode(CLAW_PWM, OUTPUT);
  pinMode(CLAW_DIR, OUTPUT);

  pinMode(SPIN_DIR, OUTPUT);
  pinMode(SPIN_PWM, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(LIMITSWITCH, INPUT);
  
  stepper_pitch.begin(RPM, 1);
  stepper_spin.begin(RPM, 1);

  Setpoint_spin = 710;            // neutral position for claw spin

  // inputs
  Setpoint_claw = 60;             // claw position
  theta4 = 90;                    // pitch angle
  theta3 = 90;                    // Upper arm angle
  theta2 = 90;                    // Lower arm angle

  angle_upper_to_reading(theta3);
  angle_lower_to_reading(theta2);
  pitchAngleToPot(theta4);

  // Reset claw
  initializeClaw();
}


// Main code, runs repeatedly
void loop() {

  // Move upper arm down (in case of fire)
//  while(1){
//    digitalWrite(UPPER_DIR, 1);
//    analogWrite(UPPER_PWM, 127);
//    Serial.println(analogRead(A0));
//  }

  // Move upper arm up (in case of fire)
//  while(1){
//    digitalWrite(UPPER_DIR, 0);
//    analogWrite(UPPER_PWM,127);
//  }

  // Move lower arm down (back)
//  while(1){
//    digitalWrite(LOWER_DIR, 1);
//    analogWrite(LOWER_PWM, 127);
//    Serial.println(analogRead(A1));
//  }


  //spin(); // spin to gripper to neutral position (Setpoint_spin)
  
  if((upperarmDone && lowerarmDone) == 0){
    //Serial.print(upperarmDone && lowerarmDone);
    arms();             // move lower and upper arms to desired angles (theta3, theta2)
    
  } else if(pitchDone == 0){
    // move to desired pitch (theta4)
    pitch();    // runs to theta4
    
  } else if (clawDone == 0){
    // move to desired gripper position (claw_position)
    gripper();
    
  } else if (upperarmDone == 1 && lowerarmDone == 1 && pitchDone == 1 && clawDone == 1){
    read_inputs();
  }
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ arm functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void arms(){

  // Read Pot and determine direction
  Input_upper = analogRead(A0);
  Input_lower = analogRead(A1);

//-----------------------------------Limit Check-------------------------------------------//
  if ((Setpoint_upper > upper_min_lim) && (Setpoint_upper < upper_max_lim) &&   // limit check
    (Setpoint_lower > lower_min_lim) && (Setpoint_lower < lower_max_lim) && (Input_upper > 65)){
      
    average_upper = average_upper + Input_upper;
    average_lower = average_lower + Input_lower;

    if (arm_count == 9){
      average_upper = average_upper/10;
      average_lower = average_lower/10;
      
//-----------------------------------Upper--------------------------------------------------//
 
      if(average_upper > Setpoint_upper + error_upper) {          // increase angle
        myPID_upper_rev.Compute();
        upperDirection = 0;
        digitalWrite(UPPER_DIR, upperDirection);
        digitalWrite(UPPER_PWM, Output_upper);
        
      } else if (average_upper < Setpoint_upper - error_upper) {  // decrease angle
        myPID_upper_fwd.Compute();
        upperDirection = 1;
        digitalWrite(UPPER_DIR, upperDirection);
        digitalWrite(UPPER_PWM, Output_upper);

      } else {
        digitalWrite(UPPER_PWM, LOW);
        upperarmDone = 1;
      }
     
//-----------------------------------Lower--------------------------------------------------//
      if (average_lower > (Setpoint_lower + error_lower)) {         // increase angle
        myPID_lower_rev.Compute();
        lowerDirection = 1;
        digitalWrite(LOWER_DIR, lowerDirection);
        digitalWrite(LOWER_PWM, Output_lower);
        
      } else if (average_lower < (Setpoint_lower - error_lower)) {  // decrease angle
        myPID_lower_fwd.Compute();
        lowerDirection = 0;
        digitalWrite(LOWER_DIR, lowerDirection);
        digitalWrite(LOWER_PWM, Output_lower);
      
      } else {
        digitalWrite(LOWER_PWM, LOW);
        lowerarmDone = 1;
      }
    }

    arm_count++;
    if (arm_count > 9){
      arm_count = 0;
      average_upper = 0;
      average_lower = 0;
    }
  }
  if((upperarmDone && lowerarmDone) == 1){
    delay(2000);
  }
}

// Rotates pitch motor to desired pitch (Setpoint_pitch)
void pitch(){
  // Read Pot and determine direction
  Input_pitch = analogRead(A10);
  // Take average of the potentiometer reading for increased accuracy
  average_pitch = average_pitch + Input_pitch;

  // Take average of 6 potentiometer values
  if (count == 5){
    average_pitch = average_pitch/6;

    if(average_pitch < (Setpoint_pitch - error_pitch)){         //Increase angle
      stepper_pitch.rotate(10);    
    } else if(average_pitch > (Setpoint_pitch + error_pitch)){  // Decrease angle
      stepper_pitch.rotate(-10);
    } else {
      stepper_pitch.stop();
      pitchDone = 1;
    } 
  }

  count++;
  if (count > 5){   // reset at 6
    count = 0;
    average_pitch = 0;
  }
}

// angle to reading for upper arm
int angle_upper_to_reading(int angle) {
  Setpoint_upper = (0.0145*pow(angle,2) - 5.8438*angle + 1136.6);   // quadratic
}

// angle to reading for lower arm
int angle_lower_to_reading(int angle) {
  Setpoint_lower = (-0.0161*pow(angle,2) - 2.2429*angle + 550.38);  // quadratic
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ gripper functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// moves gripper to claw_position
void gripper(){
  // clawDirection => 0 = open claw, 1 = close claw
  // clawPosition => higher = more open, lower = more closed 

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ read claw position ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (clawState == 1){  // only read values in motor is running

    stateB = digitalRead(ENCODER_B);
    currentStateCLK = digitalRead(ENCODER_A);

    // stateB = (PORTB & 4) >> 2;
    // currentStateCLK = (PORTB & 8) >> 3;
    
    Serial.print("\n stateB:  ");
    Serial.print(stateB); 
    Serial.print(" stateA: ");
    Serial.print(currentStateCLK);
 

    // If the previous and the current state of the inputCLK are different then a pulse has occured
    if (currentStateCLK != previousStateCLK){
      Serial.print("Average dir: ");
      Serial.print(average_dir); 
     
      // If the inputDT state is different than the inputCLK state then 
      if (stateB != currentStateCLK) {    // Encoder is rotating CW (closing)
        average_dir--;
      } else {                           // Encoder is rotating CCW (opening)
        average_dir++;
      }
          
     previousStateCLK = currentStateCLK;
    } 
    // Update previousStateCLK with the current state
   
  
    if (count2 == 5){
      if (average_dir < 0){   // average CW (closing)
        claw_position--;
      } else {                // average CCW (opening)
        claw_position++;
      }
    }
     
    count2++;
    if (count2 > 10){   // reset at 11
      count2 = 0;
      average_dir = 0;
    }
  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ finish reading claw position ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // move claw to desired position
  if (claw_position > Setpoint_claw + 2){    // claw too open
    analogWrite(CLAW_PWM, 100);
    clawDirection = 1;
    digitalWrite(CLAW_DIR, clawDirection);
    clawState = 1;

    Serial.println("test2");
    
  } else if (claw_position < Setpoint_claw - 2) { // claw too closed
    analogWrite(CLAW_PWM, 100);
    clawDirection = 0;
    digitalWrite(CLAW_DIR, clawDirection);
    clawState = 1;
    
  } else {    // turn off claw
    analogWrite(CLAW_PWM, 0);
    clawState = 0;
    clawDone = 1;
  }

//  if (clawDone == 0){
//    Serial.print("Claw position: ");
//    Serial.print(claw_position);
//    Serial.print("Setpoint_claw: ");
//    Serial.print(Setpoint_claw);
//    Serial.print("\n");
//  }
}

// Rotates spin motor to desired angle
void spin(){
  Input_spin = analogRead(A5);

  if (Input_spin < (Setpoint_spin - error_spin)){            // reading too low
    stepper_spin.rotate(-5);   // spin CW (from above)
  } else if (Input_spin > (Setpoint_spin + error_spin)){    // reading too high
    stepper_spin.rotate(5);    // spin CCW (from above)
  } else {
    stepper_spin.stop();
  }
}

// converts given pitch angle to equivalent potentiometer reading
double pitchAngleToPot(int angle){
  Setpoint_pitch = double((4.0227 * angle) + 93.03);
}

//Set the claw position so we know where it is
void initializeClaw(){
  //Read the limitSwitch as high or low
  
  Limit = digitalRead(LIMITSWITCH);
  
  //Close the claw until the limit switch is triggered
  //Now we know where the motor is and can use the encoder
  while(Limit != 1){ //Run until the limit switch is triggered
    //Set direction to reverse
    Limit = digitalRead(LIMITSWITCH);
    clawDirection = 1;    // close
    digitalWrite(CLAW_DIR, clawDirection);
    //Write speed between 0 and 255 to motor
    analogWrite(CLAW_PWM, 100);
  }
  stopClaw(); //Make sure claw is stopped

  //Open claw slightly
  clawDirection = 0;
  digitalWrite(CLAW_DIR, clawDirection);
  analogWrite(CLAW_PWM, 100);
  delay(2000);
  stopClaw();
}

void stopClaw(){
  //Set claw outputs to low
  digitalWrite(CLAW_DIR, LOW);
  digitalWrite(CLAW_PWM, LOW);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ input function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void read_inputs() {
  while (Serial.available() > 0){
    data = Serial.read();
    if (isdigit(data) && (data != ' ')){
      command += data;
    }

    if (data == '\n'){
      if (input_count == 0){
        input1 = command.toInt();
        Serial.print("upper angle = ");
        Serial.print(input1);
        Serial.print("\n");
      
      } else if (input_count == 1){
        input2 = command.toInt();
        Serial.print("lower angle = ");
        Serial.print(input2);
        Serial.print("\n");

      } else if (input_count == 2){
        input3 = command.toInt();
        Serial.print("pitch angle = ");
        Serial.print(input3);
        Serial.print("\n");
        
      } else {
        input4 = command.toInt();
        Serial.print("gripper setting = ");
        Serial.print(input4);
        Serial.print("\n");

        theta3 = input1;
        theta2 = input2;
        theta4 = input3;
        Setpoint_claw = input4;

        angle_upper_to_reading(theta3);
        angle_lower_to_reading(theta2);
        pitchAngleToPot(theta4);

        upperarmDone = 0;
        lowerarmDone = 0;
        pitchDone = 0;
        clawDone = 0;

        //restart = 1;
        input_count = 0;
        command = "";
        
        return;
      }
      
      command = "";
      input_count++;
      break;
    }
  }
}


void inverse_kinematics(){
  // y_coordinate, x_coordinate, and z_coordinate are input values for the desired positions of the arm
  // a1, a2 and a3 are lengths of the arms sections
  // theta1, theta2, theta3, theta4 are measuring from current position
  // phi1, phi2, phi3, and r1, r2, r3 are calculated based on inputs
  
  a1 = 61;    //cm from rotating points
  a2 = 73;    //cm from rotating point to end effector
  a3 = 33;    
  wx = 0;
  wy = 0;
  wz = -1;
  
  X0 = x_coordinate - a3*wx;
  Y0 = y_coordinate - a3*wy;
  Z0 = z_coordinate - a3*wz;

  // unused
  // theta1 = 180/PI * atan(y_coordinate/x_coordinate);

  r1 = sqrt((X0)^2 + (Y0)^2);

  r2 = Z0;

  phi2 = 180/PI * atan(r2/r1);

  r3 = sqrt((r1)^2 + (r2)^2);

  phi1 = 180/PI * acos(((a2)^2 - (a1)^2 - (r3)^2)/(-2*a1*r3));

  theta2 = phi1 + phi2;

  theta3 = 180/PI * acos(((r3)^2 - (a1)^2 - (a2)^2)/(-2*a1*a2));

  
  // calculates theta 4 constrained to "straight down"
  
  length1 = 180/PI * a1 * cos(theta2);
  length2 = r1 - length1;
  theta4 = 180/PI * asin(length2/a2);
}
