#include "Arduino.h"
#include "Motor.h"
#include "Sensor.h"

namespace
{
const byte interrupt_pin_2 = 2;
const byte interrupt_pin_3 = 3;

auto speed = 200;

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

void loop()
{
  Serial.println("Got here");
  robot_follow_front(20);

} // 1cm 30ms

void robot_follow_front(int distance) 
{
  Serial.println("Start of function");
  delay(100);
  if (sensor.distance_forwards() > distance) {
    robot_forward();
    Serial.println("HI");
    Serial.println(sensor.distance_forwards());
  } else {
    Serial.println("Else");
    for (int x=10; x<180; x+10) {
      left_motor.reverse(200); right_motor.forward(200);
      delay(x*10);
      int left = sensor.distance_forwards();
      robot_stop(200);
      
      left_motor.forward(200); right_motor.reverse(200);
      delay(x*10);
      int right = sensor.distance_forwards();
      robot_stop(200);
  
      if (left < distance) {
        left_motor.reverse(200); right_motor.forward(200);
        delay(x*2*10);
        break;
      } else if (right < distance) {
        break;
      }
    }
  }
}

boolean is_closer_to_sensor_down(int distance)
{ // checks whether an object is 'distance' from down facing sensor
  delay(60);
  if (sensor.distance_down() < distance) {
    return true;
  }
  return true;
}

boolean is_closer_to_sensor_front(int distance)
{ 
  delay(60);
  if (sensor.distance_forwards() == 0) {
    return true;
  }
  if (sensor.distance_forwards() < distance) {
    return true;
  }
  return false;
}

void robot_forward()
{ // forwars the robot
  left_motor.forward(speed);
  right_motor.forward(speed);
  speed_adjustment();
  delay(100);
}

void robot_reverse()
{ // reverses the robot
  left_motor.reverse(speed);
  right_motor.reverse(speed);
  speed_adjustment();
  delay(100);
}

void robot_forward_distance(int distance)
{ // forwars the robot
  left_motor.forward(speed);
  right_motor.forward(speed);
  speed_adjustment();
  delay(distance * 30);
}

void robot_reverse_distance(int distance)
{ // reverses the robot
  left_motor.reverse(speed);
  right_motor.reverse(speed);
  delay(distance * 30);
}

void robot_stop(int time)
{ // stops the motors
  left_motor.stop();
  right_motor.stop();
  delay(time);
}


void robot_wheelie() {
  right_motor.reverse(255);
  left_motor.reverse(255);
  delay(500);
  right_motor.forward(255);
  left_motor.forward(255);
  delay(500);
}

void robot_moonwalk() {
  if (is_closer_to_sensor_front(15) == false) {
    robot_reverse_distance(50);
    robot_turn_left(4);
    robot_reverse_distance(50);
    robot_turn_left(2);
    robot_reverse_distance(50);
  }
  robot_forward();
}


void robot_turn_left(int quarter_turns)
{ // turns anticlockwise 90' * 'quarter_turns'
  for (int x = 1; x <= quarter_turns; x++) {
    left_motor.forward(speed);
    right_motor.reverse(speed);
    delay(900);
  }
}
// line 100
void robot_turn_right(int quarter_turns)
{ // turns clockwise 90' * 'quarter_turns'
  for (int x = 1; x <= quarter_turns; x++) {
    left_motor.reverse(speed);
    right_motor.forward(speed);
    delay(900);
  }
}

void robot_turn_left_corner(int quarter_turns)
{ // turns in a corner anticlockwise
  for (int x = 1; x <= quarter_turns; x++) {
    robot_turn_left(1);
    robot_forward();
  }
}

void robot_turn_right_corner(int quarter_turns)
{ // turns in a corner clockwise
  for (int x = 1; x <= quarter_turns; x++) {
    robot_turn_right(1);
    robot_forward();
  }
}


void speed_adjustment()
{ // adjusts the speed of each motor to make the course straight
  while (left_motor.get_pulse_count() != right_motor.get_pulse_count()) {
    // breaks out of the loop when both counts are the same
    int count_control = 0;
    if (count_control >= 2500) {
      left_motor.set_speed(left_motor.get_speed() + 1);
    }
    // checks wether the counts for the motors are the same
    if (left_motor.get_pulse_count() < right_motor.get_pulse_count()) { // left is slower than right ?
      left_motor.set_speed(left_motor.get_speed() + 1); // increases left motor speed
      right_motor.set_speed(right_motor.get_speed() - 1); // decreases right motor speed

    } else { // right is slower that left ?
      left_motor.set_speed(left_motor.get_speed() - 1); // decreases left motor speed
      right_motor.set_speed(right_motor.get_speed() + 1); // increases right motor speed
    }
    count_control++;
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
