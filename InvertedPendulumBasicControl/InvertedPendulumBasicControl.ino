
//  Daniel McGinn
//  ME 180 Digital Controls of Dynamic Systems
//  Inverted Pendulum Controller
 /* Bipolar Stepper Motor C104 */
//  Datasheet: https://cdn-shop.adafruit.com/product-files/324/C140-A+datasheet.jpg
 /* Rotary Encoder E6A2-CW3C */
//  Code Refactored from https://www.sparkfun.com/products/10932
//  Datasheet: https://cdn.sparkfun.com/assets/6/5/d/0/c/COM-10932_YUMO_A6A2__E6A2_Encoder_Datasheet.pdf

/* Include Libraries */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>

/* Define Variables */
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

void setup() {
  Serial.begin (9600); //Start Serial Monitor

  /* Initialise the motor shield in stepper mode */
  Serial.println(F("Initialising the stepper motor (200 steps)"));
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->setSpeed(STEPPER_RPM);
  Serial.println("");
  
  /* Initialise the Motor Shield in Stepper Mode */
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
    if (counter != lastCounter) {
    Serial.println(counter); // Show Encoder Value
    lastCounter = counter;
   }
   /* Controller Code Bellow*/
   if (counter>500){
    myMotor->step(1, BACKWARD, DOUBLE); //Move Right
    }
   if (counter<500){
    myMotor->step(1, FORWARD, DOUBLE); //Move Left
    }
}

void changeA() { //Called when A changes
   stateA = !stateA;

   if (stateA == HIGH) { //If A is High and B is Low then CCW
     if (stateB == LOW ) {
       counter++;
       if (counter == 1000) {
         counter = 0;
       }
     } else {  //Else CW
         counter--;
         if (counter == -1) {
           counter = 999;
         }
     }
 }
 }
 void changeB() { //Called when B changes
   stateB = !stateB;
 }
