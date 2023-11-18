#include <AccelStepper.h>
#include <time.h>
#include <Wire.h>
#include <Servo.h>

// Pins
#define dirPin 2
#define stepPin 3
#define enablePin 4
#define servoPin 9

// Misc
#define motorInterfaceType 1
#define maxSpeed 10000

#define gearRatio 39.5
#define stepsPerRevolution 2048.

#define servo_gearRatio 3.333;

uint32_t heading;

uint32_t pitch;
float pitch_float;
int pitch_offset = 8;

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
Servo pitchServo;

void receiveEvent(int bytes)
{
  heading = 0;
  // read the 2 bytes (because heading can be > 255)
  for (int i = 0; i < 2; i++)
  {
    heading += Wire.read();
  }
  pitch = Wire.read();
  pitch_float = (pitch + pitch_offset) * servo_gearRatio;
}

void rotate()
{
}

void setup()
{
  Wire.begin(9);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // enable stepper

  stepper.setMaxSpeed(maxSpeed);

  stepper.setAcceleration(maxSpeed);
  stepper.setSpeed(maxSpeed);

  stepper.setCurrentPosition(0); // set current pos to home (0 steps)

  pitchServo.attach(servoPin);
}

void loop()
{
  int currentHeading = map(stepper.currentPosition(), 0, (stepsPerRevolution * gearRatio), 0, 360);
  int delta_angle = heading - currentHeading;
  if (delta_angle > 180)
  {
    delta_angle = (360 - delta_angle) * -1;
    // stepper.setSpeed(abs(maxSpeed) * -1);
  }
  else if (delta_angle < -180)
  {
    delta_angle = 360 - abs(delta_angle);
    // stepper.setSpeed(maxSpeed);
  }

  // Serial.println(delta_angle);
  stepper.move(delta_angle);
  while (stepper.distanceToGo() != 0)
  {
    if (delta_angle < 0)
    {
      stepper.setSpeed(maxSpeed * -1);
    }
    else
    {
      stepper.setSpeed(maxSpeed);
    }
    stepper.runSpeed();
  }

  if (pitch < 90)
  {
    pitchServo.write(pitch_float);
  }
}