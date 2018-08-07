#include "Arduino.h"
#include "Motor.h"
#include "Sensor.h"

namespace 
{
  const byte interrupt_pin_2 = 2;
  const byte interrupt_pin_3 = 3;
  int i = 0;
  int left_pulse = 0;
  int right_pulse = 0;
  int moonwalking = 0;
  int base_speed = 200;
  int wheelie = 0;
  auto speed_leftmotor = 200;
  auto speed_rightmotor = 200;
  // Instantiate the two motor objects, passing in the side of the robot the motor is on
  Motor left_motor{"left"};
  Motor right_motor{"right"};

  // Instantiate a sensor object
  Sensor sensor{};
}

void setup()
{
  Serial.begin(115200);
  pinMode(interrupt_pin_2, INPUT_PULLUP);
  pinMode(interrupt_pin_3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_2), left_motor_pulse_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin_3), right_motor_pulse_interrupt, RISING);

}
//challenge 0
void loop()
{  
  /*
  // Move robot forwards for one second
  left_motor.forward(speed);
  right_motor.forward(speed);

  delay(1000);
  
  // Turn robot for one second
  left_motor.forward(speed);
  right_motor.reverse(speed);

  delay(1000);
  
  // Move robot backwards for one second
  left_motor.reverse(speed);
  right_motor.reverse(speed);

  delay(1000);

  // Stop the robots motors
  left_motor.stop();
  right_motor.stop();

  exit(0);
  
  challenge_1();
  

  
  //challenge 2
  // this will make the rover move in a figure of eight

  for (i=0; i <3; i++)
  {
  forward();
  delay(1500);
  degree_turn("right",2000);
  delay(500);
  }
   forward();
  delay(1500);
  stop_function();
  delay(500);
  for (i=0; i <4; i++)
  {
  forward();
  delay(1500);
  degree_turn("left",1800);
  delay(500);

  }
   right();  
  delay(1550);
  stop_function(); 
  delay(1000000000000);
  

  //challenge 3
  //if the sensors detect no ground, the rover will stop
  forward_cliff();
  down_sensor();

    

  //challenge 4
  //if the sensors detect no ground or a wall, the rover will stop
  forward();
  down_sensor();
  forward_sensor();
  

  //challenge 5
  //the rover will pull a wheelie.
  reverse();
  delay(1000);
  forward_variable(255);
  delay(4000);
  stop_function();
  delay(12000000000000);


  //challenge 6 
  //the rover will run to a wall, avoid it, turn 360 and do a moonwalk
  forward();
  speed_adjust();
  forward_sensor_dance();
 
 
  //challenge 7 WIP
  //the rover will navigate a martian maze
  forward();
  maze_navigation();
    */
  //show off
  /*
  forward_variable(150);
  delay(15000);
  reverse();
  delay(1000);
  forward_variable(255);
  delay(4000);
  stop_function();
  delay(2000);
  degree_turn("right",2250);
  stop_function();
  delay
  */
  forward();
  speed_adjust();
  //delay(5000);

  



  
}
//challenge 1
// this makes the rover move for 2 meters in a straight line

void challenge_1()
{
  if (right_motor.get_pulse_count() < 100)
  {
    speed_adjust();
    forward();
    Serial.println("adjusts");
    Serial.println(left_motor.get_pulse_count());
  }
  if (right_motor.get_pulse_count() > 100)
  {
    stop_function();
    Serial.println("stop inside challenge");
    exit(0);

  }
  
}

















//forward function

void forward()
{
  left_motor.forward(speed_leftmotor);
  right_motor.forward(speed_rightmotor);
   
}

void forward_variable(int this_speed)
{
  left_motor.forward(this_speed);
  right_motor.forward(this_speed);
}

//backward function
void reverse()
{
  left_motor.reverse(speed_leftmotor);
  right_motor.reverse(speed_rightmotor);    
}

//Turns in a direction at a certian speed
void turn(String direction_turn, int turn_speed)
{
  if (direction_turn == "right")
  {
    left_motor.forward(base_speed);
    right_motor.reverse(base_speed);
  }
  if (direction_turn == "left")
  {
    left_motor.reverse(base_speed);
    right_motor.forward(base_speed);
  }
}

//stop fucntion
void stop_function()
{
  left_motor.stop();
  right_motor.stop();    
}



//change the speeds depending on the pulses of each motor
void speed_adjust()
{
  while (left_motor.get_pulse_count() != right_motor.get_pulse_count()) 
  {
    //checls wether the counts for each motor are the same
    if (left_motor.get_pulse_count() < right_motor.get_pulse_count())
    {
      if (speed_leftmotor <= 255 and speed_rightmotor >= 120)
        speed_leftmotor ++;
        speed_rightmotor --;

    }
    if (left_motor.get_pulse_count() > right_motor.get_pulse_count())
    {
      if (speed_leftmotor <= 255 and speed_rightmotor >= 120)
      {
        speed_leftmotor --;
        speed_rightmotor ++;
      }
      
    }
    else
    {
      speed_leftmotor == 200;
      speed_rightmotor == 200;
    }


  }
}

// Turns the rover in a direction and with a time delay
void degree_turn(String direction_of_rover,int time_delay)
{
  if (direction_of_rover == "right")
  {
    turn("right",200);

  }
  else
  {
    turn("left",200);
  }
  delay(time_delay);
  stop_function();
}
//checks if anything is below the rover, and if there isnt, the rover reverses.
void down_sensor()
{ 
  int read_distance_dwn = sensor.distance_down();
  if (read_distance_dwn > 15 and read_distance_dwn != 0)
  {
    reverse();
    delay(4000);
    stop_function();
    delay(4000);
  }
}
// Chekcs if anything is in front of the rover, and if there is, the rover stops.

void forward_sensor()
{ 
   int read_distance_fwd = sensor.distance_forwards();
  if (read_distance_fwd < 20 and read_distance_fwd != 0)
  {
    //Serial.println(sensor.distance_forwards());
    //delay(50);
    //if (sensor.distance_forwards() != 0);
    //{
      reverse();
      delay(1000);
      degree_turn("right",4500);
      stop_function();
      delay(2000);
    //}
  }
  else
  {
   Serial.println(sensor.distance_forwards());
   delay(50);
  }
  
}

//this detecst a wall infront, and if it finds one, it does a dance.
void forward_sensor_dance()
{
  if (sensor.distance_forwards() < 20 )
  {
    if (sensor.distance_forwards() > 0)
    {
      degree_turn("right",3700);
      reverse();
      delay(2000);
      degree_turn("right",1800);
      forward();
      delay(2000);
      stop_function();
      delay(1000000);
     }
  }
  else
  {
    delay(50);
  }
}
void maze_navigation()
{
  if (sensor.distance_forwards() <=20);
  {
    stop_function();
    delay(400);

    if (sensor.distance_forwards() == 0);
    {
      forward_variable(200);
    }

    turn("right",500);
    stop_function();
    delay(500);
    int right_maze = sensor.distance_forwards();

    turn("left",500);
    stop_function();
    delay(500);
    int left_maze = sensor.distance_forwards();

    if (left_maze > right_maze)
    {
      forward_variable(200);
      delay(300);
    
    }
    else
    {
    turn("right",500);
    delay(1000);
    }
  }
  delay(60);
}
void left_motor_pulse_interrupt()
{
  left_motor.inc_pulse();
}

void right_motor_pulse_interrupt()
{
  right_motor.inc_pulse();
}
