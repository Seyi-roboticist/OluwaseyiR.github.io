/*
  Developer: Group 15 (Team of 2), Software and Electrical Lead -> Seyi R. Afolayan 
  Project: Lab 3 - Autonomous Navigation
  Improvement: This sketch can be modularized! 
*/ 

// Headers and libraries 
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

// Creating Pixy and Motor Shield Objects
DualMAX14870MotorShield robot;
Pixy2 pixy;

// Global pin configurations and contant defintions 
const int encoder0PinA = 2; 
const int encoder0PinB = 3; 
volatile long encoder0Pos = 0;
int encoder0PinALast = LOW; 
volatile int n = LOW;
const int pingPin {10};
const int IR_left = A0; 
const int IR_right = A1;  
const double sideDistance = 12.5; 
const int pixyDist = 50;

// Encoder parameters for precise navigation 
const double wheelDiameter = 6.5; // as per DFRobot 65mm Rubber Wheel Pair - Blue. I need very these measurements in person
const double wheelCircumference = wheelDiameter * (22.0/7.0); 
const uint8_t CPR = 48; // As per the pololu-25d-metal-gearmotors
const double wheelBase = 15; // This need gotten from the CAD


// Simple state machine for motor movements 
enum motorsMove {
  idle, 
  move_forward, 
  move_backward, 
  left_turn,
  right_turn, 
  u_turn, 
  obstacle_avoidance
};

// We assumed in the last office hours that we are just moving forward for now 
motorsMove currentState = move_forward; 

void setup(){
  // Begin the Serial communication
  Serial.begin(115200); 

  // Initialize and enable pixy and robot motor drivers, respectively. 
  pixy.init(); 
  robot.enableDrivers(); 
  pinMode(pingPin, OUTPUT); 
  digitalWrite(pingPin, LOW);

  // Encoder stuffs 
  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 
}

void loop(){
  // So we should continually measure the three distances from the sensor
  double frontDist = measureDistance(); 
  double leftDist = measureIRDistance(IR_left); 
  double rightDist = measureIRDistance(IR_right);

  // We will adjust this distance as we see fit -- Perhaps implement a PID controller here
  if ((leftDist || rightDist) < sideDistance && frontDist < sideDistance)
  currentState = obstacle_avoidance; 

  switch(currentState){
    case idle: {
      delay(4000); // stay put for 2s 
      currentState = move_forward; 
      break;
    }
    case move_forward: {
      forward(); // moving forward 
      checkForCues(); // checking to make sure we can read the cues and its distance 
      break;
    }
    case move_backward: {
      backward();  // reverse
      delay(2000); 
      stop(); 
      currentState = idle;
      break; 
    }
    case left_turn: {
      left(); 
      delay(1000); 
      currentState = move_forward; // after turning left move forward (Need to implement logic for encodeder and pi/2)
      break;
    }
    case right_turn: {
      right(); 
      delay(1000); 
      stop(); 
      currentState = move_forward; 
      break;
    }
    case u_turn: {
      // preliminary design of u_turn 
      performUturn();
      break;
    }
    case obstacle_avoidance: {
      stop(); 
      delay(1000); 
      currentState = determineTurn(leftDist, rightDist); 
      break;
    }
  }
}

// Measure Ping Sensor Distance 
double measureDistance() {
  pinMode(pingPin, OUTPUT); 
  pinMode(pingPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(pingPin, HIGH); 
  delayMicroseconds(5); 
  digitalWrite(pingPin, LOW); 
  pinMode(pingPin, INPUT); 
  size_t duration = pulseIn(pingPin, HIGH); 
  double dist = duration * 0.034 / 2; 
  return dist;
}

// Measuring the Distances with the IR Sensors
double measureIRDistance(int sensorPin) {
  uint16_t sensorVal = analogRead(sensorPin); 
  float voltage = sensorVal * (5.0 / 1023.0); 
  double dist = 27.86 * pow(voltage, -1.15); // I can replace this with the IR equation if I have time or use LUT
  return dist; 
}

// function to check for color blocks using Pixy2 Camera 
void checkForCues() {
  pixy.ccc.getBlocks(); // Get color blocks detected by the Pixy2 camera 
  if (pixy.ccc.numBlocks) {
    Serial.print(pixy.ccc.blocks[0].m_signature); 
    Serial.print(" "); 
    Serial.println(pixy.ccc.blocks[0].m_width); 
    if (pixy.ccc.blocks[0].m_signature == 1 && pixy.ccc.blocks[0].m_width >= pixyDist) {
      currentState = left_turn; // I will need to double check this to make sure that this the right cue for left turning
    }
    else if (pixy.ccc.blocks[0].m_signature == 2 && pixy.ccc.blocks[0].m_width >= pixyDist){
      currentState = right_turn; 
    }
    else if (pixy.ccc.blocks[0].m_signature == 3 && pixy.ccc.blocks[0].m_width >= pixyDist){
      currentState = u_turn;
    }
  }
}

// Function to determine the direction turn 
motorsMove determineTurn(float leftDist, float rightDist){
  if (leftDist > rightDist) {
    return left_turn; 
  }
  else {
    return right_turn;
  }
}

// Implementing Encoder functions
double countToDistance(int counts){
  return (counts / (double)CPR) * wheelCircumference;
}

// Encoder Functions 
// Interrupt on A changing state
void doEncoderA() {
  n = digitalRead(encoder0PinA); 
  // I am checking the current state of the encoder here
  // Essentially, if A is changing (from low to high) and if B is high, increment. Otherwise, decrement. 
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
     if (digitalRead(encoder0PinB) == LOW) {
        encoder0Pos--; // CW
     }
     else{
      encoder0Pos++; // CCW
     }
  }
  // Just like buttonstates, we need to update the last state 
  encoder0PinALast = n;
}
// Interrupt on B changing state
void doEncoderB() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++; //
  }
  else {
    encoder0Pos--;
  }
}

// Implementing the angle to counts function 
int angleToCounts(double angle) {
  double arcLength = (2 * (22.0/7.0) * wheelBase * angle) / 360.0; 
  int counts = (arcLength * CPR) / wheelCircumference; 
  return counts;
}

// The Uturn function 
void performUturn() {
  static bool isTurning = false; // Obviously, I need to initially set this flag to false 
  static long startEncoderPos = 0; 
  
  // Starting the turn
  if (!isTurning){
    startEncoderPos = encoder0Pos; // record the encoder position at this turn and set flag to true
    isTurning = true; 
    setMotorSpeed(-200, 200);
  }
  long encoderDiff = abs(encoder0Pos - startEncoderPos); 
  int countsForUturn = angleToCounts(180.0); 
  if (encoderDiff >= countsForUturn) {
    stop(); // Essentially, I stop the motors once 180 degree is reached and hopefully facing the next cue
    isTurning = false; // setting this back to false 
    currentState = move_forward; // we keep moving until we sense a new obstacle in front
  }

}

// Motor control functions
// Might need to implement the setMotorSpeed function for some turning
void setMotorSpeed(int leftSpeed, int rightSpeed){
  robot.setM1Speed(leftSpeed); 
  robot.setM2Speed(rightSpeed);
}
void stop() {
  robot.setM1Speed(0);
  robot.setM2Speed(0);
}

void forward() {
  robot.setM1Speed(400);
  robot.setM2Speed(-400);
}

void backward() {
  robot.setM1Speed(-400);
  robot.setM2Speed(400);
}

void left() {
  robot.setM1Speed(-150);
  robot.setM2Speed(150);
}

void right() {
  robot.setM1Speed(150);
  robot.setM2Speed(-150);
}
