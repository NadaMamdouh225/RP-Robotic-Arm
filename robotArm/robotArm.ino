/*
  This Library is written for A4988 Basic operation Example
  Author: Bonezegei (Jofel Batutay)
  Date: March 2024 
*/

#include <Bonezegei_A4988.h>
#include <string.h>
#include "conf.hpp"

float pres_current_distance_mm = 0;
float rev_current_angle_deg = 0;

Bonezegei_A4988 revoluteStepper(REVOLUTE_PIN_DIR, REVOLUTE_PIN_STEP);
Bonezegei_A4988 persimiticStepper(PERSIMITIC_PIN_DIR, PERSIMITIC_PIN_STEP);

int revoluteSteps = 0;
int persimiticSteps = 0;

String input1;
String input2;

void setRevoluteJoint(int angle)
{
  int steps = ((double)angle/360.0) * COMPLETE_REVOLUTION;
  if(steps > revoluteSteps){
    revoluteStepper.step(FORWARD, steps -  revoluteSteps);
    revoluteSteps = steps;
  }
  else{
    revoluteStepper.step(REVERSE, revoluteSteps - steps);
    revoluteSteps = steps;
  }
}

void setPersimiticJoint(int distance)
{
  int steps = distance*PRES_SIGNAL_STEPS_PER_MM/400;
  if(steps > persimiticSteps){
    persimiticStepper.step(REVERSE, steps - persimiticSteps);
    persimiticSteps = steps;
  }
  else{
    persimiticStepper.step(FORWARD, persimiticSteps - steps);
    persimiticSteps = steps;
  }
}

void setup(){
  revoluteStepper.begin();
  persimiticStepper.begin();
  revoluteStepper.setSpeed(50000);
  persimiticStepper.setSpeed(50000);
  Serial.begin(115200);
  pinMode(8, OUTPUT);
  digitalWrite(8, 0);
}

void loop(){
  int rev_input, pres_input;
  
  // Prompt user to enter X and Y coordinates
  // Serial.println("Enter X and Y coordinates (separated by a space):");
  while (Serial.available() <= 0); // Wait for input
  String input = Serial.readStringUntil('\n'); // Read input as a string
  
  // Parse X and Y coordinates
  sscanf(input.c_str(), "%d %d", &rev_input, &pres_input);


//  Serial.print("Enter the distance: ");
//  while(!Serial.available());
//  input1 = Serial.readString();
//  Serial.println(input1);

//  Serial.print("Enter the angle: ");
//  while(!Serial.available());
//  input2 = Serial.readString();
//  Serial.println(input2);

 setPersimiticJoint(pres_input);
 setRevoluteJoint(rev_input);
 delay(100);
}
