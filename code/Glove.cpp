#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>

#include <mbed.h>
#include <rtos.h>

#define button1 3
#define LED_white 5
const int triggerPin = A5;

volatile bool triggerState = false;

float thetaM;
float phiM;

float thetaG = 0;
float phiG = 0;

float theta;
float phi;
float psi;

float phiRAD;
float thetaRAD;

float magXf = 0;
float magYf = 0;
float magZf = 0;

float pitchf = 0;
float rollf = 0;
float yawf = 0;

float dt;
unsigned long millisOld;

uint32_t heading;
byte heading_and_pitch[3];
float headingRaw;

uint32_t oldheading = 0;

boolean viewInSerialPlotter = true; // true optimises for serial plotter, false for serial monitor

BLEService gloveService("044cddba-fb4d-47a4-90da-9f7578a26931");
BLECharacteristic headingInt("b3de23b1-12fe-4266-a590-53528c11116d", BLERead, 3);

BLEIntCharacteristic buttonPress("6541935a-5c22-464b-b150-c335eaa92910", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic triggerPull("626488b2-70d1-4b89-ad13-45e8ee61ebd8", BLERead | BLENotify);

bool deviceConnected = false;
bool stationary = false;

// Threads
rtos::Thread thread1;
rtos::Thread thread2;

struct EulerAngles
{
   float pitch;
   float roll;
   float yaw;
};

float EarthMagnetStrength = 49.3; //= µT

boolean magnetOK = false;
uint8_t magnetODRindex = 8; // (0..8)->{0.625,1.25,2.5,5.0,10,20,40,80,400}Hz
uint8_t magnetFSindex = 0;  // 0=±400.0; 1=±800.0; 2=±1200.0 , 3=±1600.0  (µT)

void raw_N_Magnet(unsigned int N, float &averX, float &averY, float &averZ)
{
   float x, y, z;
   averX = 0;
   averY = 0;
   averZ = 0;
   for (int i = 1; i <= N; i++)
   {
      while (!IMU.magnetAvailable())
         ;
      IMU.readRawMagnet(x, y, z);
      averX += x;
      averY += y;
      averZ += z;
      digitalWrite(LED_BUILTIN, (millis() / 125) % 2); // blink onboard led every 250ms
      if ((i % 30) == 0)
         Serial.print('.');
   }
   averX /= N;
   averY /= N;
   averZ /= N;
   digitalWrite(LED_BUILTIN, 0); // led off
}

void calibrateMagnet() // measure Offset and Slope of XYZ
{
   digitalWrite(LED_white, HIGH);
   float x, y, z, Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
   unsigned long count = 0;
   IMU.setMagnetODR(8);                // Fast rate 400Hz
   raw_N_Magnet(10, Xmin, Ymin, Zmin); // find some starting values
   Xmax = Xmin;
   Ymax = Ymin;
   Zmax = Zmin;
   int i = 0;
   while (digitalRead(button1) == HIGH) // measure until enter key pressed
   {
      raw_N_Magnet(10, x, y, z); // average over a number of samples to reduce the effect of outlyers
      Xmax = max(Xmax, x);
      Xmin = min(Xmin, x);
      Ymax = max(Ymax, y);
      Ymin = min(Ymin, y);
      Zmax = max(Zmax, z);
      Zmin = min(Zmin, z);
      i++;
   }

   // signal that calibration is done
   digitalWrite(LED_white, LOW);
   delay(300);
   digitalWrite(LED_white, HIGH);
   delay(300);
   digitalWrite(LED_white, LOW);

   IMU.setMagnetOffset((Xmax + Xmin) / 2, (Ymax + Ymin) / 2, (Zmax + Zmin) / 2);                                                                        // store offset
   IMU.setMagnetSlope((2 * EarthMagnetStrength) / (Xmax - Xmin), (2 * EarthMagnetStrength) / (Ymax - Ymin), (2 * EarthMagnetStrength) / (Zmax - Zmin)); // store slope
   magnetOK = true;
   IMU.setMagnetODR(magnetODRindex);
}

void magnetometerDoNMeasurements(unsigned int N, float &averX, float &averY, float &averZ)
{
   float x, y, z;
   averX = 0;
   averY = 0;
   averZ = 0;
   for (int i = 1; i <= N; i++)
   {
      while (!IMU.magnetAvailable())
         ;
      IMU.readMagnet(x, y, z);
      averX += x;
      averY += y;
      averZ += z;
   }
   averX /= N;
   averY /= N;
   averZ /= N;
}

void accelerometerDoNMeasurements(unsigned int N, float &averX, float &averY, float &averZ)
{
   float x, y, z;
   averX = 0;
   averY = 0;
   averZ = 0;
   for (int i = 1; i <= N; i++)
   {
      while (!IMU.accelAvailable())
         ;
      IMU.readAccel(x, y, z);
      averX += x;
      averY += y;
      averZ += z;
   }
   averX /= N;
   averY /= N;
   averZ /= N;
}

void gyroscopeDoNMeasurements(unsigned int N, float &averX, float &averY, float &averZ)
{
   float x, y, z;
   averX = 0;
   averY = 0;
   averZ = 0;
   for (int i = 1; i <= N; i++)
   {
      while (!IMU.gyroAvailable())
         ;
      IMU.readGyro(x, y, z);
      averX += x;
      averY += y;
      averZ += z;
   }
   averX /= N;
   averY /= N;
   averZ /= N;
}

EulerAngles calcHeading()
{

   float magX, magY, magZ;
   float accX, accY, accZ;
   float gyrX, gyrY, gyrZ;

   // average measurements to reduce a little noise

   magnetometerDoNMeasurements(5, magX, magY, magZ);
   accelerometerDoNMeasurements(5, accX, accY, accZ);
   gyroscopeDoNMeasurements(5, gyrX, gyrY, gyrZ);

   thetaM = atan2(accX / 9.81, accZ / 9.81) / 2 / 3.141592654 * 360;
   phiM = atan2(accY / 9.81, accZ / 9.81) / 2 / 3.141592654 * 360;

   dt = (millis() - millisOld) / 10000.;
   millisOld = millis();

   // complimentary filter for pitch (theta) and roll (phi)
   theta = (theta + gyrY * dt) * .90 + (thetaM * .10);
   phi = (phi - gyrX * dt) * .90 + (phiM * .10);

   IMU.readMagnet(magX, magY, magZ);

   phiRAD = phi / 360 * (2 * PI);
   thetaRAD = theta / 360 * (2 * PI);

   float Xm;
   float Ym;

   // low pass filter for magnetometer
   magXf = magX * 0.25 + (magXf * (1.0 - 0.25));
   magYf = magY * 0.25 + (magYf * (1.0 - 0.25));
   magZf = magZ * 0.25 + (magZf * (1.0 - 0.25));

   // tilt compensated vector calculations for heading
   Xm = magXf * cos(thetaRAD) + magZf * sin(thetaRAD);
   Ym = magXf * sin(phiRAD) * sin(thetaRAD) + magYf * cos(phiRAD) - magZf * sin(phiRAD) * cos(thetaRAD);

   // Xm=magX*cos(thetaRAD)-magY*sin(phiRAD)*sin(thetaRAD)+magZ*cos(phiRAD)*sin(thetaRAD);
   // Ym=magY*cos(phiRAD)+magZ*sin(phiRAD);

   // final heading angle calculation
   psi = atan2(Ym, Xm) / (2 * PI) * 360;

   if (psi < 0)
   {
      psi = 360 + psi;
   }

   // raw heading
   headingRaw = atan2(magX, magY) / (2 * 3.14) * 360;

   // Gyro data filtered
   // thetaG=thetaG+gyrY*dt;
   // phiG=phiG-gyrX*dt;

   EulerAngles angles;

   angles.pitch = theta;
   angles.roll = phi;
   angles.yaw = psi;

   return angles;
}

EulerAngles calcAverageAngles()
{
   int N = 16;
   float averPitch = 0;
   float averRoll = 0;
   float averYaw = 0;
   for (int i = 1; i <= N; i++)
   {
      EulerAngles angles = calcHeading();
      averPitch += angles.pitch;
      averRoll += angles.roll;
      averYaw += angles.yaw;
   }
   averPitch /= N;
   averRoll /= N;
   averYaw /= N;

   return {averPitch, averRoll, averYaw};
}

EulerAngles calcLowPassAngles()
{
   EulerAngles angles = calcHeading();

   float pitchf = angles.pitch * 0.25 + (pitchf * (1.0 - 0.25));
   float rollf = angles.roll * 0.25 + (rollf * (1.0 - 0.25));
   float yawf = angles.yaw * 0.25 + (yawf * (1.0 - 0.25));

   return {pitchf, rollf, yawf};
}

bool triggered = true;
void triggerCallback()
{
   while (true)
   {
      if (analogRead(triggerPin) > 390 && triggered)
      {
         triggerPull.writeValue((uint32_t)0);
         digitalWrite(LED_white, LOW);
         triggered = false;
      }
      else if (analogRead(triggerPin) <= 390)
      {
         triggerPull.writeValue((uint32_t)1);
         digitalWrite(LED_white, HIGH);
         triggered = true;
      }
      rtos::ThisThread::yield();
   }
}

void headingCallback()
{
   while (true)
   {
      if (stationary == false)
      {
         EulerAngles angles = calcAverageAngles(); // calculate heading
         heading = angles.yaw;

         // only update when tilted if there was a significant change in heading
         if (angles.pitch > 15 && (sqrt((oldheading - heading) * (oldheading - heading))) > 8)
         {
            Serial.print("Heading is at ");
            Serial.print(heading);
            Serial.println(" degrees");

            if (heading > 255)
            {
               heading_and_pitch[0] = (uint32_t)255;
               heading_and_pitch[1] = heading - (uint32_t)255;
            }
            else
            {
               heading_and_pitch[0] = heading;
               heading_and_pitch[1] = 0;
            }
            heading_and_pitch[2] = angles.roll; // roll is pitch bc of rotation in glove mounting

            headingInt.writeValue(heading_and_pitch, 3); // and update the heading characteristic
            oldheading = heading;
         }
         // when not very tilted just update normally
         if (angles.pitch < 15)
         {
            if ((sqrt((oldheading - heading) * (oldheading - heading))) > 3)
               Serial.print("Heading is at ");
            Serial.print(heading);
            Serial.println(" degrees");

            if (heading > 255)
            {
               heading_and_pitch[0] = (uint32_t)255;
               heading_and_pitch[1] = heading - (uint32_t)255;
            }
            else
            {
               heading_and_pitch[0] = heading;
               heading_and_pitch[1] = 0;
            }
            heading_and_pitch[2] = angles.roll; // roll is pitch bc of rotation in glove mounting

            headingInt.writeValue(heading_and_pitch, 3); // and update the heading characteristic
            oldheading = heading;
         }
      }
      rtos::ThisThread::yield();
   }
}

void setup()
{
   Serial.begin(9600);
   // while(!Serial);                   // wait till the serial monitor connects

   pinMode(button1, INPUT_PULLUP);
   pinMode(LED_white, OUTPUT);

   delay(1);
   if (!IMU.begin())
   { // initialize the magnetometer
      Serial.println("Failed to initialize IMU!");
      while (1)
         ;
   }

   /*****************   For a proper functioning of the compass the magnetometer needs to be calibrated    ********************
   *****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/

   // Calibrate Magnetometer
   calibrateMagnet();

   // Accelerometer code
   IMU.setAccelFS(3);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(-0.015487, -0.010553, -0.001073);
   IMU.setAccelSlope(0.997558, 0.993326, 1.003259);

   // Gyroscope code
   IMU.setGyroFS(2);
   IMU.setGyroODR(5);
   IMU.setGyroOffset(1.581726, -0.599609, 0.703522);
   IMU.setGyroSlope(1.797827, 1.807842, 1.948031);

   /******************************************************************************************************************************
   ****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****
   ****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
   *******************************************************************************************************************************/

   IMU.magnetUnit = MICROTESLA; //   GAUSS   MICROTESLA   NANOTESLA

   if (!BLE.begin())
   {
      Serial.println("* Starting Bluetooth Low Energy module failed!");
      while (1)
         ;
   }

   BLE.setLocalName("TurretGlove");
   BLE.setAdvertisedService(gloveService);      // add the service UUID
   gloveService.addCharacteristic(headingInt);  // add the heading characteristic
   gloveService.addCharacteristic(buttonPress); // add the button characteristic
   gloveService.addCharacteristic(triggerPull); // add the trigger characteristic

   BLE.addService(gloveService); // Add the heading service
   byte oldh[3] = {0, 0, 0};
   headingInt.writeValue(oldh, 3); // set initial value for this characteristic
   triggerPull.writeValue((uint32_t)0);

   BLE.advertise();
   Serial.println("Arduino Nano 33 BLE Sense (Central Device). Waiting for connections...");

   thread1.start(headingCallback);
   thread2.start(triggerCallback);

   millisOld = millis();
}

bool pressed = true;
void loop()
{
   // wait for a Bluetooth® Low Energy central
   BLEDevice central = BLE.central();

   // if a central is connected to the peripheral:
   if (central)
   {
      Serial.print("Connected to central: ");
      // print the central's BT address:
      Serial.println(central.address());
      // turn on the LED to indicate the connection:
      digitalWrite(LED_BUILTIN, HIGH);

      // check the heading level every 500ms while the central is connected:
      while (true)
      {
         if (central.connected())
         {

            // BUTTONS
            if (digitalRead(button1) == LOW)
            { // pressed
               buttonPress.writeValue((uint32_t)1);
               // wait for confirmation --> stationary mode activated or not
               uint32_t value;
               delay(1500);
               // buttonPress.readValue(value);
               // if (value == 0)
               stationary = !stationary;
               pressed = true;
            }
            else if (digitalRead(button1) == HIGH && pressed)
            {
               buttonPress.writeValue((uint32_t)0);
               digitalWrite(LED_BUILTIN, HIGH);
               pressed = false;
            }

            // controller.run();
            rtos::ThisThread::yield();
         }
      }

      // when the central disconnects, turn off the LED:
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
      stationary = false;
      pressed = true;
   }
}