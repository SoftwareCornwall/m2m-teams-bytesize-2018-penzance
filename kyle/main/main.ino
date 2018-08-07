#include "Arduino.h"
#include "Motor.h"
#include "Sensor.h"


namespace 
{
  const byte interrupt_pin_2 = 2;
  const byte interrupt_pin_3 = 3;

  auto speed = 200;

  int demo_section = 1;

  // Instantiate the two motor objects, passing in the side of the robot the motor is on
  Motor left_motor{"left"};
  Motor right_motor{"right"};
  int LLight = 15;
  int rLight = 14;
  int speakerPin = 6;
  // Instantiate a sensor object
  Sensor sensor{};
  #define c 261
  #define d 294
  #define e 329
  #define f 349
  #define g 391
  #define gS 415
  #define a 440
  #define aS 455
  #define b 466
  #define cH 523
  #define cSH 554
  #define dH 587
  #define dSH 622
  #define eH 659
  #define fH 698
  #define fSH 740
  #define gH 784
  #define gSH 830
  #define aH 880
}

void setup()
{
  Serial.begin(115200);
  pinMode(interrupt_pin_2, INPUT_PULLUP);
  pinMode(interrupt_pin_3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_2), left_motor_pulse_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_3), right_motor_pulse_interrupt, RISING);

  pinMode(speakerPin, OUTPUT);
}

void loop()
{  
  Pathfinder_maze();
  
}

void sensormove(){

  delay(50);

  if (sensor.distance_down() > 14) {
    stop_motors();
    reverse();
    delay(500);
    stop_motors();
    delay(10000);
  }
  else{
    forward();
  }
  

  
}



void left_motor_pulse_interrupt()
{
  left_motor.inc_pulse();
}

void right_motor_pulse_interrupt()
{
  right_motor.inc_pulse();
}

void figure8() {
  for(int i=0; i<3; i++){
    forward_small();
    righturn();
  }
  forward_small();
  forward_small();
  for(int i=0; i<3; i++){
    lefturn();
    forward_small();
  }
  delay(8000);
}

void forward2m(){

  if (left_motor.get_pulse_count() < 1074) // this tells the rover to stop after it reaches 1074 ticks while driving
  { 
      forward();
    if (left_motor.get_pulse_count() == right_motor.get_pulse_count()){
      forward();
    }
    else if (left_motor.get_pulse_count() > right_motor.get_pulse_count()){ // if the left motor goes too fast this will turn it off untill they are equal again
      left_motor.stop();
    }
    else if (left_motor.get_pulse_count() < right_motor.get_pulse_count()){ // if the right motor goes too fast this will turn it off untill they are equal again
      right_motor.stop();
    }
  }
  else
  {
    stop_motors(); // stops the rover
  }

  // this functions turns the motor on at the base speed 
}
void forward(){
  left_motor.forward(speed);
  right_motor.forward(speed);
}

// this function tell the motors to stop
void stop_motors(){
    left_motor.stop();
    right_motor.stop();
}  
void reverse(){
  left_motor.reverse(speed);
  right_motor.reverse(speed);
}

void righturn(){
  right_motor.reverse(speed);
  left_motor.forward(speed);
  delay (900);
  stop_motors();
}

void lefturn(){
  right_motor.forward(speed);
  left_motor.reverse(speed);
  delay (785);
  stop_motors();
}

void forward_small(){
  forward();
  delay(1000);
  stop_motors();
  
}

void frontsensormove(){

  delay(50);

  if (sensor.distance_forwards() < 14) {
    stop_motors();
    reverse();
    delay(500);
    stop_motors();
    delay(10000);
  }
  else{
    forward();
  }
  

  
}

void headlights(){
  digitalWrite(LLight, HIGH);
  digitalWrite(rLight, LOW);// turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LLight, LOW);
  digitalWrite(rLight, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);   
  digitalWrite(rLight, LOW);
  delay(1000);
   digitalWrite(LLight, HIGH);
  digitalWrite(rLight, HIGH);
  delay(1000);
   digitalWrite(LLight, LOW);
  digitalWrite(rLight, LOW);
  delay(1000);// wait for a second
}

void weelies(){
  reverse();
  delay(300);
  forwardfast(255);
  delay(2000);
}

void forwardfast(int wellie_speed) {
  left_motor.forward(wellie_speed);
  right_motor.forward(wellie_speed);
}

void March() {
  {    
    //for the sheet music see:
    //http://www.musicnotes.com/sheetmusic/mtd.asp?ppn=MN0016254
    //this is just a translation of said sheet music to frequencies / time in ms
    //used 500 ms for a quart note
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, a, 500); 
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, a, 500); 
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, a, 500);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW); 
    beep(speakerPin, f, 350);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, cH, 150);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);

      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, a, 500);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, f, 350);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, cH, 150);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, a, 1000);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    //first bit
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, eH, 500);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, eH, 500);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, eH, 500);  
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);  
    beep(speakerPin, fH, 350); 
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, cH, 150);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);

      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, gS, 500);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, f, 350);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);


      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, cH, 150);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
      
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, a, 1000);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);

    //second bit...
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, aH, 500);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);
    beep(speakerPin, a, 350);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, a, 150);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, aH, 500);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, gSH, 250); 
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, gH, 250);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    

      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, fSH, 125);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, fH, 125);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);        
    beep(speakerPin, fSH, 250);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    delay(250);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, aS, 250);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, dSH, 500);  
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, dH, 250);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);  
    beep(speakerPin, cSH, 250); 
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);     
    //start of the interesting bit
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, cH, 125);  
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);    
    beep(speakerPin, b, 125);  
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);
    beep(speakerPin, cH, 250);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);          
    delay(250);
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);    
    beep(speakerPin, f, 125);
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);      
    beep(speakerPin, gS, 500);  
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);       
    beep(speakerPin, f, 375);  
      digitalWrite(rLight, LOW);
      digitalWrite(LLight, LOW);       
    beep(speakerPin, a, 125); 
      digitalWrite(rLight, HIGH);
      digitalWrite(LLight, HIGH);       
    
    beep(speakerPin, cH, 500); 
    beep(speakerPin, a, 375);  
    beep(speakerPin, cH, 125); 
    beep(speakerPin, eH, 1000); 
    //more interesting stuff (this doesn't quite get it right somehow)
    
    beep(speakerPin, aH, 500);
    beep(speakerPin, a, 350);
    FlashLights(); 
    beep(speakerPin, a, 150);
    beep(speakerPin, aH, 500);
    beep(speakerPin, gSH, 250); 
    beep(speakerPin, gH, 250);
    
    beep(speakerPin, fSH, 125);
    beep(speakerPin, fH, 125);    
    beep(speakerPin, fSH, 250);
    delay(250);
    beep(speakerPin, aS, 250);    
    beep(speakerPin, dSH, 500);  
    beep(speakerPin, dH, 250);  
    beep(speakerPin, cSH, 250);  
    //repeat... repeat
    
    beep(speakerPin, cH, 125);  
    beep(speakerPin, b, 125);  
    beep(speakerPin, cH, 250);      
    delay(250);
    beep(speakerPin, f, 250);  
    beep(speakerPin, gS, 500);  
    beep(speakerPin, f, 375);  
    beep(speakerPin, cH, 125); 
           
    beep(speakerPin, a, 500);            
    beep(speakerPin, f, 375);            
    beep(speakerPin, c, 125);            
    beep(speakerPin, a, 1000);       
    //and we're done \ó/    
}
}

void beep (unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds)
{ 
    //use led to visualize the notes being played
    
    int x;   
    long delayAmount = (long)(1000000/frequencyInHertz);
    long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
    for (x=0;x<loopTime;x++)   
    {    
        digitalWrite(speakerPin,HIGH);
        delayMicroseconds(delayAmount);
        digitalWrite(speakerPin,LOW);
        delayMicroseconds(delayAmount);
    }    
    
    //set led back to low
    
    delay(20);
    //a little delay to make all notes sound separate
}    

void Demo_Sensor() {
    
    delay(50);
  Serial.println("Sensor down reading");
  Serial.println(sensor.distance_down());
  if (sensor.distance_down() > 14) {
    Serial.println("Stop motors");
    stop_motors();
    reverse();
    Serial.println("Reverse");
    delay(1000);
    stop_motors();
    righturn();
    righturn();
    stop_motors();
    demo_section = 2;
    Serial.println("demo_section = 2");
  }
  else{
    forward();
  }
  

}


void Demo_Straight_Move(){
  int take_pulse_reading = left_motor.get_pulse_count();

  if (left_motor.get_pulse_count() < take_pulse_reading + 550) // this tells the rover to stop after it reaches 1074 ticks while driving
  { 
      forward();
    if (left_motor.get_pulse_count() == right_motor.get_pulse_count()){
      forward();
    }
    else if (left_motor.get_pulse_count() > right_motor.get_pulse_count()){ // if the left motor goes too fast this will turn it off untill they are equal again
      left_motor.stop();
    }
    else if (left_motor.get_pulse_count() < right_motor.get_pulse_count()){ // if the right motor goes too fast this will turn it off untill they are equal again
      right_motor.stop();
    }
  }
  else
  {
    stop_motors(); // stops the rover
    //demo_section = 3;
    //Serial.println("demo_section = 3");
  }
}

void Demo_Show() {
  if (demo_section == 1){
    Demo_Sensor();
  }
  if (demo_section == 2){
    delay(2000);
    demo_forward();
  }
  if (demo_section == 3){
    delay(1000);
    Spin();
    March();
  }
  
  

}
     
void FlashLights(){
  digitalWrite(rLight, HIGH);
  digitalWrite(LLight, HIGH);
  delay(500);
  digitalWrite(rLight, LOW);
  digitalWrite(LLight, LOW);
  delay(500);
  
}

void Spin(){
  right_motor.reverse(speed);
  left_motor.forward(speed);
}


void demo_forward(){
  for(int i=0; i<3; i++){
  forward_small();
  }
  delay(500);
  demo_section = 3;
  Serial.println("demo_section = 3");
  
}


void Pathfinder_maze(){
  
  int sensor_reading = sensor.distance_forwards();
  if (sensor_reading > 18 or sensor_reading == 0){
      //right_motor.forward(speed);
      //left_motor.reverse(speed);
      //delay(785);

      forward();
  }
  if (sensor.distance_forwards() < 18){
    right_motor.reverse(speed);
    left_motor.forward(speed);
    delay(900);

    sensor_reading = sensor.distance_forwards();

    if(sensor_reading < 18) {
      right_motor.reverse(speed);
      left_motor.forward(speed);
      delay(1800);
    }

    sensor_reading = sensor.distance_forwards();

    if(sensor_reading < 18) {
      right_motor.reverse(speed);
      left_motor.forward(speed);
      delay(1800);
    } else {
      forward();
    }
  } else { 
      forward();
  }
}


