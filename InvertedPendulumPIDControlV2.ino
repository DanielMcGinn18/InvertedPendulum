//  Daniel McGinn - Tufts University Mechanical Engineering
//  ME 180 Digital Controls of Dynamic Systems
//  Inverted Pendulum Controller
 /* Bipolar Stepper Motor C104 */
//  Datasheet: https://cdn-shop.adafruit.com/product-files/324/C140-A+datasheet.jpg
 /* Adafruit Motor Shield */
//  Technical Details: https://www.adafruit.com/product/1438?gclid=CjwKCAiAl7PgBRBWEiwAzFhmmsYg8ZNfud-ACm6uEw0K0GdpUw9f-Pz6IYIsgT3rCuLK2Oo-ehxf3BoCK9EQAvD_BwE
 /* Rotary Encoder E6A2-CW3C */
//  Code Citation: https://www.sparkfun.com/products/10932
//  Datasheet: https://cdn.sparkfun.com/assets/6/5/d/0/c/COM-10932_YUMO_A6A2__E6A2_Encoder_Datasheet.pdf
 /* PID Controller */
 // Code Citation: https://www.asee.org/file_server/papers/attachment/file/0005/8013/Turner_Cooley_A_low_cost_and_flexible_open_source_inverted_pendulum_for_feedback_control_laboratory_coursesASEE_2015.pdf

/* Include Libraries */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>

/* Define Variables for Motor */
#define STEPPER_STEPS               200
#define STEPPER_RPM                 500

/* Motor Shield Settings */
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(STEPPER_STEPS, 2);

/* Rotary Encoder Settings */
// Pins 2 & 3 must be used for interupts on Arduino Uno 
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ 
#define rotaryA 2 //Black Wire
#define rotaryB 3 //White Wire
#define LED 13 //Use onboard LED
volatile int stateA = LOW; //Declare Variables (allow change at any time)
volatile int stateB = LOW;
volatile int counter = 0; //Start Counter at 0
volatile int lastCounter = 0;

/* Define Variables for PID Control */
float kp,ki, kd; //declare variables used to adjust gain constants
//Declare necessary global variables to be passed between calls of ComputPID()
float Setpoint=755, Output, error;
unsigned long Told;
float ErrorSum, ErrorOld;
int steps;

void setup() {
  Serial.begin (9600); //Start Serial Monitor

  /* Initialize Motor Shield in Stepper Mode */
  Serial.println(F("Initialising the stepper motor (200 steps)"));
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(STEPPER_RPM);
  Serial.println("");
  
  /* Initialize Rotary Encoder Pins */
  pinMode (rotaryA, INPUT_PULLUP); //Initialize digital pins as an input with the internal pull-up resistor enabled
  pinMode (rotaryB, INPUT_PULLUP);
  pinMode (LED, OUTPUT); //Initilize onboard LED pin

  /* Monitor External Interuputs for Rotary Encoder */
  attachInterrupt(digitalPinToInterrupt(rotaryA), changeA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(rotaryB), changeB, CHANGE);
  //Change triggers the interrupt whenever the pin changes value
  stateA = digitalRead(rotaryA);
  stateB = digitalRead(rotaryB);
  //Initilize stateA & stateB
  Serial.println("Encoder Initialized");
}

void loop() {
    /* Read Encoder */
    if (counter != lastCounter) {
//    Serial.println(counter); // Show Encoder Value
    lastCounter = counter;
   }
   /* PID Control*/
   SetTuning(.35,0,0);
   ComputePID(); // Call the PID Function
//   Serial.print("Error: ");Serial.println(error); // Print Error
   steps=int(61.11*tan((Output)/2/PI)); // convert from encoder pulses to motor steps
   if(steps>25) steps=25; // Set Max steps
   if(steps<0) steps=0; // Set Min steps
   Serial.print("Steps: ");Serial.println(steps); //Print Steps
   if(error<0){ // If errror is less than 0, move to the left
   Serial.print("Direction: Left      ");
   myMotor->step(steps, FORWARD, DOUBLE); //Move Left
   }
   if(error>0){ // If errror is greater than 0, move to the right
   myMotor->step(steps, BACKWARD, DOUBLE); //Move Right
   Serial.print("Direction: Right     ");
   }
}
void ComputePID() { // PID Control
    error = Setpoint - counter; // Calculate error between Setpoint and Input from Encoder
    float errorABS = abs(error);
    float Tnew = millis(); // Store current time
    float dT = Tnew-Told; // Calculate the time change since the last PID calculation by subtracting previous time from current time
    ErrorSum += (errorABS * dT); // Approximate the integral by summing the total error
    float dError = (errorABS - ErrorOld) / dT; // Approximate the derivative by dividing the change in error by the time change
    Output = kp * errorABS + ki * ErrorSum + kd * dError; // Compute the PID controller output
    ErrorOld = errorABS; Told = Tnew; // Update variables for use in next iteration of ComputePID() 
}
void changeA() { // Called when A changes
   stateA = !stateA;

   if (stateA == HIGH) { // If A is High and B is Low then CCW
     if (stateB == LOW ) {
       counter++;
       if (counter == 1024) {
         counter = 0;
       }
     } else {  // Else CW
         counter--;
         if (counter == -1) {
           counter = 1023;
         }
     }
 }
 }
 void changeB() { // Called when B changes
   stateB = !stateB;
 }
 
 void SetTuning(float KP, float KI, float KD) // Set Gain Constants
{
    kp = KP; //set proportional constant 
    ki = KI; //set integral constant
    kd = KD; //set derivative constant
}
