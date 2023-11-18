#include <time.h>
#include <ArduinoBLE.h>
#include <Wire.h>
#include <Arduino_Threads.h>

#include <Thread.h>
#include <ThreadController.h>

// BLUETOOTH STUFF
const char *deviceServiceUuid = "044cddba-fb4d-47a4-90da-9f7578a26931";
const char *headingServiceCharacteristicUuid = "b3de23b1-12fe-4266-a590-53528c11116d";
const char *buttonServiceCharacteristicUuid = "6541935a-5c22-464b-b150-c335eaa92910";
const char *triggerServiceCharacteristicUuid = "626488b2-70d1-4b89-ad13-45e8ee61ebd8";

byte heading[3];
uint32_t buttonState;
uint32_t triggerState;

uint32_t offset;

#define buzzerPin 5
#define gunPin 6

long homing_pos = -1;
float TravelX;

ThreadController controller = ThreadController();

Thread headingT = Thread();
Thread triggerT = Thread();

BLECharacteristic headingCharacteristic;
BLECharacteristic triggerCharacteristic;

bool stationary = true;

void headingCallback()
{
  if (!stationary)
  {
    headingCharacteristic.readValue(&heading, 3);
    yield();

    int newHeading = heading[0] + heading[1];
    newHeading -= offset;
    if (newHeading < 0)
    {
      newHeading = 360 + newHeading;
    }
    int pitch = heading[2];
    Serial.print("Heading is at ");
    Serial.println(newHeading);
    Serial.print("Tilt is at ");
    Serial.println(pitch);

    byte headingTransmit[3];

    if (newHeading > 255)
    {
      headingTransmit[0] = 255;
      headingTransmit[1] = newHeading - 255;
    }
    else
    {
      headingTransmit[0] = newHeading;
      headingTransmit[1] = 0;
    }
    headingTransmit[2] = pitch;

    yield();
    // send heading to Arduino UNO over serial
    Wire.beginTransmission(9);
    Wire.write(headingTransmit, 3);
    Wire.endTransmission();
  }
}
int shots = 0;

void triggerCallback()
{
  if (triggerCharacteristic.valueUpdated())
  {
    triggerCharacteristic.readValue(triggerState);
    if (triggerState == 1 && shots == 0)
    {
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(buzzerPin, LOW);
      shots++;
    }
    if (triggerState == 0)
    {
      shots = 0;
    }
  }
}

void home(BLECharacteristic buttonCharacteristic, BLECharacteristic headingCharacteristic)
{
  Serial.println("Homing . . .");

  // buzz once
  digitalWrite(buzzerPin, HIGH); // turn on
  delay(1000);
  digitalWrite(buzzerPin, LOW); // turn off

  buttonCharacteristic.readValue(buttonState);
  while (buttonState == 0)
  {
    // wait for button to be pressed on glove
    Serial.println("Waiting for button to be pressed.");

    headingCharacteristic.readValue(&heading, 3); // get heading when button is pressed --> offset
    offset = heading[0] + heading[1];
    buttonCharacteristic.readValue(buttonState);
  }
  buttonCharacteristic.writeValue((uint32_t)1);
  Serial.print("Home! ");
  Serial.print("Offset is ");
  Serial.println(offset);

  // buzz twice
  digitalWrite(buzzerPin, HIGH); // turn on
  delay(250);
  digitalWrite(buzzerPin, LOW); // turn off
  delay(100);
  digitalWrite(buzzerPin, HIGH); // turn on
  delay(250);
  digitalWrite(buzzerPin, LOW); // turn off
}

void controlPeripheral(BLEDevice peripheral)
{
  Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect())
  {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  }
  else
  {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }
  Serial.println("- Discovering peripheral device attributes...");

  if (peripheral.discoverAttributes())
  {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  }
  else
  {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  headingCharacteristic = peripheral.characteristic(headingServiceCharacteristicUuid);

  if (!headingCharacteristic)
  {
    Serial.println("* Peripheral device does not have a heading characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!headingCharacteristic.canRead())
  {
    Serial.println("* Peripheral does not have a readable heading characteristic!");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic buttonCharacteristic = peripheral.characteristic(buttonServiceCharacteristicUuid);

  if (!buttonCharacteristic)
  {
    Serial.println("* Peripheral device does not have a button characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!buttonCharacteristic.canRead())
  {
    Serial.println("* Peripheral does not have a readable button characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!buttonCharacteristic.canSubscribe())
  {
    Serial.println("Button characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  }
  else if (!buttonCharacteristic.subscribe())
  {
    Serial.println("Subscription to button characteristic failed!");
    peripheral.disconnect();
    return;
  }
  else
  {
    Serial.println("Subscribed to button characteristic.");
  }

  triggerCharacteristic = peripheral.characteristic(triggerServiceCharacteristicUuid);

  if (!triggerCharacteristic)
  {
    Serial.println("* Peripheral device does not have a trigger characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!triggerCharacteristic.canRead())
  {
    Serial.println("* Peripheral does not have a readable trigger characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!triggerCharacteristic.canSubscribe())
  {
    Serial.println("Trigger characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  }
  else if (!triggerCharacteristic.subscribe())
  {
    Serial.println("Subscription to trigger characteristic failed!");
    peripheral.disconnect();
    return;
  }
  else
  {
    Serial.println("Subscribed to trigger characteristic.");
  }

  // setup code - once peripheral connected
  if (peripheral.connected())
  {
    BLE.stopAdvertise();

    home(buttonCharacteristic, headingCharacteristic);
    delay(2000);
  }

  // Code that should get executed once paired goes here
  while (peripheral.connected())
  {
    headingT.enabled = true;
    triggerT.enabled = true;

    if (buttonCharacteristic.valueUpdated())
    {
      buttonCharacteristic.readValue(buttonState);
      if (buttonState == 1)
      {
        stationary = !stationary;
        buttonCharacteristic.writeValue((uint32_t)0);

        digitalWrite(buzzerPin, HIGH);
        delay(100);
        digitalWrite(buzzerPin, LOW);
        delay(100);
        digitalWrite(buzzerPin, HIGH);
        delay(100);
        digitalWrite(buzzerPin, LOW);
      }
    }

    controller.run();
  }
  Serial.println("- Peripheral device disconnected!");
  stationary = true;
  shots = 0;
}

void connectToPeripheral()
{
  BLEDevice peripheral;

  Serial.println("- Discovering peripheral device...");
  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);

  if (peripheral)
  {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(); // I2C communication to UNO
  while (!Serial)
    ;

  // MORE BLUETOOTH STUFF
  if (!BLE.begin())
  {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  BLE.setLocalName("Nano 33 BLE (Central)");
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE Sense (Central Device)");
  Serial.println(" ");

  pinMode(buzzerPin, OUTPUT); // set arduino pin to output mode
  pinMode(gunPin, OUTPUT);

  headingT.onRun(headingCallback);
  headingT.setInterval(1000);

  triggerT.onRun(triggerCallback);
  triggerT.setInterval(10);

  controller.add(&headingT);
  controller.add(&triggerT);
}

void loop()
{
  connectToPeripheral();
}
