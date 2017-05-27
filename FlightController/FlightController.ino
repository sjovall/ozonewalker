/*-------------------------------------
OzoneWalker Flightcomputer Main Program
Copyright OzoneWalker 2014
---------------------------------------*/

#include "Preferences.h"
#include <Wire.h>
#include <Adafruit_MPL115A2.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// Sensors
DHT HumiditySensor(humidityPin, DHTTYPE);
Adafruit_MPL115A2 PressureSensor;

// Radiomodule
SoftwareSerial Xbee(XbeeRX, XbeeTX);

// Timing
unsigned long currentTime;
unsigned long updateTime;

boolean sensorReadings = true;
boolean GPStracking = true;

void setup()
{
  // Enable serial communication for Xbee and GPS(Hardware)
  Xbee.begin(baudRate);
  Serial.begin(baudRate);
  
  // Start sensors
  HumiditySensor.begin();
  PressureSensor.begin();
  
  // Outputs
  pinMode(coolingFan, OUTPUT);
}

void loop()
{
  // Stores time since program was started
  currentTime = millis();
  
  // Define all readings
  double altitude = getAltitude();
  double humidity = HumiditySensor.readHumidity();
  double externalTemp = HumiditySensor.readTemperature();
  double internalTemp = PressureSensor.getTemperature();
  double pressure = PressureSensor.getPressure();
  double ozoneLevel = analogRead(gasSensor);
  double batteryLevel = analogRead(batteryPin) / 10.23;
  double signalStrenght = getRSSIvalue();
  
  // Send data to groundcontrol with definite frequency
  int deltaTime = currentTime - updateTime;
  if(deltaTime >= transmitDelay)
  {
    updateTime = currentTime;
    
    // Send readings
    if(sensorReadings == true)
    {
      Send('A', altitude);
      Send('H', humidity);
      Send('T', externalTemp);
      Send('t', internalTemp);
      Send('P', pressure);
      Send('O', ozoneLevel);
      Send('B', batteryLevel);
      Send('S', signalStrenght);
    }
  }
  
  // If data is recieved from groundcontrol
  if(Xbee.available() > 0)
  {
    if(Xbee.peek() == 'C')
    {
      String command = Xbee.readStringUntil('\n');
      Serial.println(command);
      executeCommand(command);
      command = "";
    }
    else
    {
      Serial.println("Recieved junkdata");
      
      // If there's any data in the serial buffer, remove everything
      while(Xbee.available() > 0)
      {     
        // Stop process if protocolcharacter is found
        if(Xbee.peek() == 'C')
        {
          break;
        }
        else
        {
          Xbee.read();
        }
      }
      
      Serial.println("Junkdata was removed from serial buffer");
    }
  }
 
  // Send GPS-coordinates
  if(Serial.available() > 0)
  { 
    String GPSstring = Serial.readStringUntil('\n');
    if(GPStracking == true)
    {
      Xbee.println(GPSstring);
    }
    GPSstring = "";
  }  
}

// Method for sending readings by OWTP
void Send(char datatype, double reading)
{
  Xbee.print('%');
  Xbee.print(datatype);
  Xbee.print(reading);
  Xbee.print('#');
  Xbee.print('\n');  
}
 
// Method for obtaining altitude from the barometric pressure formula
double getAltitude()
{
  double h;  // Current Altitude (meters)
  double T = PressureSensor.getTemperature() + 273.15; // Temperature of air (Kelvin)
  double po = 101.325;  // Pressure @ Sea level (kPA) 
  double p = PressureSensor.getPressure();  // Pressure @ current altitude (kPA)
  static double R = 8.314; // Universal gas constant (J/mol*k)
  static double M = 0.02897; // Molarmass of air (kg/mol)
  static double g = 9.8; // Gravitational acceleration (m/s^2)
  
  h = -((R * T * log(p / po)) / (M * g)); // Barometric Formula
  
  return h;
}

double getRSSIvalue()
{
  int rawSignal = pulseIn(5, LOW, 200);
  
  double percentage = map(rawSignal, 200, 0, 0, 100);
  
  if(digitalRead(5) == 0)
  {
    percentage = 0;
  }
  
  return percentage;
  
}
 
 // Method for executing commands
 void executeCommand(String command)
 {
   if(command[0] == 'C' && command[command.length()-1] == '#') // Check if the data is a command - (first character is 'C') and make sure that you recieved the whole command - (last character is '#')
      {
        Serial.println("Command confirmed");
        switch(command[3]) // Check if it's an enable - 'E' or disable command - 'D'
        {
          Serial.println("Command ID confirmed");
          Serial.println(command[2]);
          case 'E':
            switch(command[2])  // Check the id-number of the command (2 numbers - ex: 05)
            {
              case '0':
                Serial.println("Command 00 Enabled");
                digitalWrite(coolingFan, HIGH); // Enable cooling fan
              break;
              case '1':
                Serial.println("Command 01 Enabled");
                pinMode(4, OUTPUT);
                digitalWrite(4, HIGH);
              break;
              case '2':
                Serial.println("Command 02 Enabled");
                pinMode(5, OUTPUT);
                analogWrite(5, 150);
              break;
              case '3':
                Serial.println("Command 03 Enabled");
                GPStracking = true;
              break;
              case '4':
                Serial.println("Command 04 Enabled");
                sensorReadings = true; 
              break;
              case '5':
                Serial.println("Command 05 Enabled");
              break;
            }
          break;
          
          case 'D':
            switch(command[2])
            {
              case '0':
                Serial.println("Command 00 Disabled");
                digitalWrite(coolingFan, LOW); // Disable cooling fan
              break;
              case '1':
                Serial.println("Command 01 Disabled");
                pinMode(4, OUTPUT);
                digitalWrite(4, LOW);
              break;
              case '2':
                Serial.println("Command 02 Disabled");
                pinMode(5, OUTPUT);
                analogWrite(5, 0);
              break;
              case '3':
                Serial.println("Command 03 Disabled");
                GPStracking = false;
              break;
              case '4':
                Serial.println("Command 04 Disabled");
                sensorReadings = false;
              break;
              case '5':
                Serial.println("Command 05 Disabled");
              break;
            }
          break;
          
        }
     }
}

    
 
     




