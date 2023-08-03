#include <SPI.h>
#include <PID_v1.h>
#include <max6675.h>

#define MAX6675_CONV_TIME 300 // [ms] convertion time (see MAX6675 Datasheet)
#define MAX6675_CS   10
#define MAX6675_SO   12
#define MAX6675_SCK  13
#define RELAY 4
#define MAX_TEMP_READ_ATTEMPT 100

#define TEMP_LIMIT 1100         // [C°] Temperature threshold limit
#define PERIOD 1500             // [ms] PID Output (PWM) period


MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);
String cmd = "";

bool pid_ena = true;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 10, Ki = 0.3, Kd = 0;

//Specify the links and initial tuning parameters
PID ovenPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long start = 0;
unsigned long last_read_time = 0;
unsigned long windowStartTime = 0;
unsigned int i = 0;
unsigned long uptime = 0;

double temperature = 0.0;
double temperature_tmp = 0.0;

void setup()
{
  delay(1000);
  Serial.begin(19200);

  //Configure pin mode
  pinMode(RELAY, OUTPUT);
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);

  start = millis();
  windowStartTime = start;
  last_read_time =  start;

  temperature_tmp = thermocouple.readCelsius();
  i = 0;
  while ((isnan(temperature_tmp) or temperature_tmp <= 0.0) and i < MAX_TEMP_READ_ATTEMPT) {
    delay(300);
    temperature_tmp = thermocouple.readCelsius();
    i++;
  }

  temperature = temperature_tmp;
  Setpoint = temperature_tmp;

  pid_ena = true;
  ovenPID.SetOutputLimits(0, PERIOD);
  ovenPID.SetMode(AUTOMATIC);

}

void loop() {

  // Temperature acquisition
  if (millis() - last_read_time > MAX6675_CONV_TIME) {
    temperature_tmp = thermocouple.readCelsius();
    if (isnan(temperature_tmp) or temperature_tmp == 0.0) {}
    else {
      temperature = temperature_tmp;
    }
    last_read_time = millis();
  }

  // !!! ALERT Safety condition are commented !!!
  // Safety
  //if (temperature > TEMP_LIMIT and digitalRead(RELAY) == HIGH) {
  //  pid_ena = false;
  //  digitalWrite(RELAY, LOW);
  //}
  //else
  //{
  
    // Check serial connection
    if (Serial)
    {
      // Check serial input
      if (Serial.available() > 0)
      {
        cmd = Serial.readStringUntil('\n');

        // Manual mode ON
        if (cmd == "ON") {
          pid_ena = false;
          digitalWrite(RELAY, HIGH);
        }

        // Manual mode OFF
        else if (cmd == "OFF") {
          pid_ena = false;
          digitalWrite(RELAY, LOW);
        }

        else if (cmd == "READ") {
          Serial.println(temperature);
        }

        // Stop PID
        else if (cmd == "STOP") {
          pid_ena = false;
          digitalWrite(RELAY, LOW);
        }

        // Start PID
        else if (cmd == "START") {
          pid_ena = true;
        }

        // Setpoint configuration
        else if (cmd.substring(0, 8) == "SETPOINT") {
          Setpoint = cmd.substring(9).toDouble();
        }

        // Read Setpoint
        else if (cmd == "READSETPOINT") {
          Serial.println(Setpoint);
        }

        // Read DEBUG
        else if (cmd == "DEBUG") {
          uptime = millis() - start;
          Serial.println("Setpoint: " + String(Setpoint, 2) + "  - Output: " + String(Output, 2) +  "  - PID enable: " + String(pid_ena) + "  - Init temp attempts: " + String(i)  + "  - UpTime: " + String(uptime) + " ms");
        }

        else {
          //TODO 
        }

      } // Check serial input

    } // Close check serial connection

    // PID
    if (pid_ena) {

      Input = temperature;
      ovenPID.Compute();

      if (millis() - windowStartTime > PERIOD) {
        //time to shift the Relay Window
        windowStartTime += PERIOD;
      }

      if (Output >= millis() - windowStartTime) {
        digitalWrite(RELAY, HIGH);
      }
      else {
        digitalWrite(RELAY, LOW);
      }
    }
  //}

}
