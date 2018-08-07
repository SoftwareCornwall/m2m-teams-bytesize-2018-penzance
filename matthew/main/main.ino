#include "Arduino.h"
#include "Motor.h"
#include "Sensor.h"

namespace 
{
  const byte interrupt_pin_2 = 2;
  const byte interrupt_pin_3 = 3;

  auto counterleft = 0;
  auto counterright = 0;
  auto movementtype = 3;
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
  demo();  
//maze();
//forwardstraight();
//moonwalk();
//popawheely();
//sensorforward();
//figureofeight();
//move2mforward();
}

void move2mforward()
{
if (right_motor.get_pulse_count() < 1073)
  {
   //this if statment checks if it has moved 2 metres and if it hasn't it moves forward
    Serial.println("forward");
    Serial.println(right_motor.get_pulse_count());
    forward();
  }
  else
  {
    //if it has moved 2 metres it stops
    Serial.println("stop");
    stop_motors();
  }

  //exit(0);
}

void sensorforward()
{
  delay(50);
  if (sensor.distance_forwards() > 14)
  {
    forward();
  }
  else
  {
    stop_motors();
    backward();
    delay(500);
    stop_motors();
    delay(1000000);
  }
}

void moonwalk()
{
  delay(50);
  if (sensor.distance_forwards() == 0 and movementtype == 0) {
    forwardstraight();
    Serial.println("type 0");
    Serial.println(sensor.distance_forwards());
    Serial.println("==0");  
  }
  if (sensor.distance_forwards() > 14 and movementtype == 0)
  {
    forwardstraight();
    Serial.println("type 0");
    Serial.println(">14");
    Serial.println(sensor.distance_forwards());
  }
  if (sensor.distance_forwards() < 14 and sensor.distance_forwards() != 0)
  {
    Serial.println(sensor.distance_forwards());
    Serial.println("moonwalk");
    Serial.println(movementtype);
    movementtype = 1;
    stop_motors();
    backward();
    delay(500);
    turn_right_90();
    turn_right_90();
    turn_right_90();
    turn_right_90();
    backward();
    delay(1000);
    turn_right_90();
    turn_right_90();
    stop_motors();
    delay(250);
    forward();
    delay(1000);
    stop_motors();
    delay(1000000);

  }
}

void maze()
{
delay(50);
  if (sensor.distance_forwards() == 0)
  {
    forwardstraight();
  }
  if (sensor.distance_forwards() > 25)
  {
    forwardstraight();
  }

  if (sensor.distance_forwards() < 25 and sensor.distance_forwards() != 0)
  {
    stop_motors();
    delay(100);
    backward();
    delay(200);
    turn_left();
    delay(200);
  }
  if (sensor.distance_down() < 10)
  {
    stop_motors();
    delay(1000000);
  }
}

void popawheely()
{
  backward();
  delay(200);
  forwardfast();
  delay(2000);
}

void demo()
{

    if (movementtype == 3)
    {
      forward();
      delay(3000);
      turn_right_90();
      forward();
      delay(3000);
      turn_right_90();
      forward();
      delay(3000);
      turn_right_90();
      forward();
      delay(1500);
      turn_right_90();
    }
    
    movementtype = 0;



  if (sensor.distance_forwards() == 0 and movementtype == 0) {
    forward();
  }
  
  if (sensor.distance_forwards() > 14 and movementtype == 0)
  {
    forward();
    Serial.println("type 0");
    Serial.println(">14");
    Serial.println(sensor.distance_forwards());
  }
  if (sensor.distance_forwards() < 14 and sensor.distance_forwards() != 0)
  {
    Serial.println(sensor.distance_forwards());
    Serial.println("moonwalk");
    Serial.println(movementtype);
    movementtype = 1;
    stop_motors();
    backward();
    delay(500);
    turn_right_90();
    turn_right_90();
    turn_right_90();
    turn_right_90();
    backward();
    delay(1000);
    turn_right_90();
    turn_right_90();
    stop_motors();
    delay(250);
    forward();
    delay(1000);
    stop_motors();
    delay(1000000);
  }
}

void sensordown()
{
  delay(50);
  if (sensor.distance_down() < 14)
  {
    forwardstraight();
  }
  else
  {
    stop_motors();
    backward();
    delay(500);
    stop_motors();
    delay(1000000);
  }
}

void figureofeight()
{
  forward();
  delay(1000);
  turn_right_90();
  
  forward();
  delay(1000);
  turn_right_90();
  
  forward();
  delay(1000);
  turn_right_90();

  forward();
  delay(2000);
  
  turn_left_90();
  
  forward();
  delay(1000);
  turn_left_90();
  
  forward();
  delay(1000);
  turn_left_90();

  forward();
  delay(1000);

  stop_motors();
  delay(999999);
}

void left_motor_pulse_interrupt()
{
  left_motor.inc_pulse();
}

void right_motor_pulse_interrupt()
{
  right_motor.inc_pulse();
}

void forward()
{
  // Move robot forward
  left_motor.forward(200);
  right_motor.forward(150);
}

void forwardfast()
{
  //Move robot forward faster
  left_motor.forward(speed+55);
  right_motor.forward(speed+55);
}

void forwardstraight()
{
  if (left_motor.get_pulse_count() == right_motor.get_pulse_count())
  {

    forward();
  }
  if (left_motor.get_pulse_count() > right_motor.get_pulse_count())
  {

    left_motor.forward(speed-100);
  }
  if (left_motor.get_pulse_count() < right_motor.get_pulse_count())
  {

    right_motor.forward(speed-100);
  }
}

void backward()
{
  // Move robot backward
  left_motor.reverse(speed);
  right_motor.reverse(speed);
}

void stop_motors()
{
  // Stop the robots motors
  left_motor.stop();
  right_motor.stop();
}

void turn_right()
{
  //Turns the robot right
  left_motor.forward(speed);
  right_motor.reverse(speed);

}

void turn_right_move()
{
  right_motor.forward(speed-50);
  left_motor.forward(speed);
}
void turn_left_move()
{
  left_motor.forward(speed-50);
  right_motor.forward(speed);
}

void turn_right_90()
{
  //Turns the robot right
  left_motor.forward(speed);
  right_motor.reverse(speed);
  delay(875);
}

void turn_left()
{
  //turns the robot left
  left_motor.reverse(speed);
  right_motor.forward(speed);

}

void turn_left_90()
{
  //turns the robot left
  left_motor.reverse(speed);
  right_motor.forward(speed);
  delay(785);
}

