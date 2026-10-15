// Temperature war
//Pressure
// Speed
// trip
// total trip

// DONE


#include "driver/twai.h"
#include <Arduino.h>
#include <string.h>
#include <7Semi_MAX31865.h>
#include <Preferences.h>

// =====================================================
// SERIAL DEBUG CONTROLS
// Comment out any DEBUG_* line to disable that category
// =====================================================

class NullSerialClass {
public:
  template<typename T>
  void print(T) {}

  template<typename T>
  void println(T) {}

  void println() {}

  template<typename T, typename U>
  void print(T, U) {}

  template<typename T, typename U>
  void println(T, U) {}
};

NullSerialClass NullSerial;

#define DEBUG_H2
#define DEBUG_PRESSURE
#define DEBUG_TEMP
#define DEBUG_SPEED
#define DEBUG_TRIP
#define DEBUG_TOTAL_TRIP
#define DEBUG_CAN
#define DEBUG_CONTROL
#define DEBUG_TELEMETRY

#ifdef DEBUG_H2
#define H2 Serial
#else
#define H2 NullSerial
#endif

#ifdef DEBUG_PRESSURE
#define PRESSURE Serial
#else
#define PRESSURE NullSerial
#endif

#ifdef DEBUG_TEMP
#define TEMP Serial
#else
#define TEMP NullSerial
#endif

#ifdef DEBUG_SPEED
#define SPEED Serial
#else
#define SPEED NullSerial
#endif

#ifdef DEBUG_TRIP
#define TRIP Serial
#else
#define TRIP NullSerial
#endif

#ifdef DEBUG_TOTAL_TRIP
#define TOTAL_TRIP Serial
#else
#define TOTAL_TRIP NullSerial
#endif

#ifdef DEBUG_CAN
#define CAN Serial
#else
#define CAN NullSerial
#endif

#ifdef DEBUG_CONTROL
#define CONTROL Serial
#else
#define CONTROL NullSerial
#endif

#ifdef DEBUG_TELEMETRY
#define TELEMETRY Serial
#else
#define TELEMETRY NullSerial
#endif



// =====================================================
// CAN PINS
// =====================================================
#define CAN_TX GPIO_NUM_7
#define CAN_RX GPIO_NUM_6

#define H2_DIN_PIN 5


#define ADC_MAX 4095.0

#define ADC_REF 3.3
#define RL 10.0     // kΩ
#define R0_MQ 10.0  // CHANGE after calibration


#define CS_PIN 14
#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 21

#define TEMP_THRESHOLD 1000.0
#define H2LEAK_THRESHOLD 1000.0

#define RREF 426.0
#define R0 100.0

#define PRESSURE_PIN 8

#define TACHOMETER_PIN 11
#define PULSES_PER_REVOLUTION 1.0f
#define WHEEL_CIRCUMFERENCE_M 1.0f
#define SAMPLE_TIME_MS 1000UL

#define H2_CUTOFF 10

float SENSOR_MAX_PRESSURE = 300.0;

volatile unsigned long pulseCount = 0;
unsigned long previousTime = 0;

float wheelRPM = 0.0f;
float totalDistanceM = 0.0f;
float tripDistanceM = 0.0f;

float h2LeakPPM = 0.0f;
bool h2Status = false;

bool vehicleRunning = false;

MAX31865_7Semi rtd(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);

Preferences preferences;



// =====================================================
// CAN IDs
// =====================================================
#define CAN_ID_CONTROL 0x100
#define CAN_ID_CONTROL_ACK 0x101

#define CAN_ID_TEMP_SPEED 0x200
#define CAN_ID_TEMP_SPEED_ACK 0x201

#define CAN_ID_PRESSURE_TRIP 0x202
#define CAN_ID_PRESSURE_TRIP_ACK 0x203

#define CAN_ID_TOTAL_TRIP 0x204
#define CAN_ID_TOTAL_TRIP_ACK 0x205

// =====================================================
// MESSAGE TYPES
// =====================================================
#define MSG_CONTROL 0x01
#define MSG_TEMP_SPEED 0x02
#define MSG_PRESSURE_TRIP 0x03
#define MSG_TOTAL_TRIP 0x04

// =====================================================
// ACK / RETRY
// =====================================================
#define ACK_TIMEOUT_MS 1000
#define MAX_RETRIES 3

// =====================================================
// TELEMETRY STATE
// =====================================================
enum TelemetryState {
  SEND_PACKET_1,
  WAIT_PACKET_1_ACK,

  SEND_PACKET_2,
  WAIT_PACKET_2_ACK,

  SEND_PACKET_3,
  WAIT_PACKET_3_ACK,

  TELEMETRY_DONE
};

TelemetryState telemetryState = SEND_PACKET_1;

uint8_t telemetrySequence = 0;

uint8_t retryCount = 0;

unsigned long ackStartTime = 0;
unsigned long telemetryStartTime = 0;


// =====================================================
// TEST TELEMETRY VALUES
// =====================================================
bool temperatureWarning;

float speed;
float pressure;
float trip;
float totalTrip;


float readH2PPM() {

  int adcValue = analogRead(H2_DIN_PIN);

  float voltage = (adcValue / 4095.0) * 3.3;

  float h2PPM = (voltage / 3.3) * 1600.0;

  if (h2PPM < 0)
    h2PPM = 0;

  if (h2PPM > 1600)
    h2PPM = 1600;

  H2.print("H2 Voltage = ");
  H2.print(voltage, 3);
  H2.print(" V | H2 = ");
  H2.print(h2PPM, 0);
  H2.println(" PPM");
  return h2PPM;
}


// =====================================================
// Cylinder Pressure
// =====================================================
float readPressure() {
  int adcValue = analogRead(PRESSURE_PIN);
  // float voltage = (adcValue * ADC_REF) / ADC_MAX;
  // float current_mA =
  //   4.0 + (voltage / 3.3) * 16.0;
  // float pressureBar =
  //   ((current_mA - 4.0) / 16.0)
  // * SENSOR_MAX_PRESSURE;
  float pressureBar = (adcValue / 4095.0) * 600.0;

  if (pressureBar < 0)
    pressureBar = 0;

  // PRESSURE.print(" | Pressure = ");
  PRESSURE.print(pressureBar, 2);
  PRESSURE.println(" bar");
  return pressureBar;
}

bool readTemp() {
  float ohms = rtd.readResistance();
  float tempC = rtd.readTemperatureC();


  TEMP.print(F("R: "));
  TEMP.print(ohms, 3);
  TEMP.print(F(" Ω"));
  TEMP.print(F("  T: "));
  TEMP.print(tempC, 2);
  TEMP.println(F(" °C"));

  if (tempC > TEMP_THRESHOLD) {
    return true;
  }
  return false;
}

void IRAM_ATTR tachometerISR() {
  pulseCount++;
}

void setupTachometer() {
  pinMode(TACHOMETER_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(TACHOMETER_PIN),
    tachometerISR,
    RISING);
  preferences.begin("vehicle", false);

  totalDistanceM = preferences.getFloat("totalDist", 0.0f);  // Load previously saved total distance

  TOTAL_TRIP.print("Saved Total Distance: ");
  TOTAL_TRIP.print(totalDistanceM, 2);
  TOTAL_TRIP.println(" m");

  previousTime = millis();
}

void readTachometer() {
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= SAMPLE_TIME_MS) {
    float timeIntervalSec = (currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float revolutions = (float)pulses / PULSES_PER_REVOLUTION;

    wheelRPM = (revolutions / timeIntervalSec) * 60.0f;

    float distance = revolutions * WHEEL_CIRCUMFERENCE_M;

    float speed = (distance / timeIntervalSec) * 3.6f;

    if (pulses > 0) {
      vehicleRunning = true;  // here the distance will keep adding till the vehicle is moving

      tripDistanceM += distance;
      totalDistanceM += distance;
    }

    else {
      if (vehicleRunning) {
        TOTAL_TRIP.println("VEHICLE STOPPED");

        preferences.putFloat("totalDist", totalDistanceM);  // it will save distance to PMS

        TOTAL_TRIP.println("Total distance saved to PMS.");

        TOTAL_TRIP.print("Saved Total Distance: ");

        TOTAL_TRIP.print(totalDistanceM, 2);
        TOTAL_TRIP.println(" m");

        TRIP.print("Trip Distance: ");
        TRIP.print(tripDistanceM, 2);
        TRIP.println(" m");

        vehicleRunning = false;

        // Reset trip for next journey
        // tripDistanceM = 0.0f;

        TRIP.println("Trip distance reset.");
      }
    }

    SPEED.print("Pulses         : ");
    SPEED.println(pulses);

    SPEED.print("RPM            : ");
    SPEED.println(wheelRPM, 2);

    SPEED.print("Speed          : ");
    SPEED.print(speed, 2);
    SPEED.println(" km/h");

    TRIP.print("Trip Distance  : ");
    TRIP.print(tripDistanceM, 2);
    TRIP.println(" m");

    TOTAL_TRIP.print("Total Distance : ");
    TOTAL_TRIP.print(totalDistanceM, 2);
    TOTAL_TRIP.println(" m");

    SPEED.print("Vehicle State  : ");

    if (vehicleRunning) {
      SPEED.println("RUNNING");
    }
  }
}

float readSpeed() {
  return (wheelRPM * WHEEL_CIRCUMFERENCE_M / 60.0f) * 3.6f;  // or store speed in a global
}

float readTrip() {
  return tripDistanceM;
}

float readTotalTrip() {
  return totalDistanceM;
}

// =====================================================
// START NEW TELEMETRY SET
// =====================================================
void generateTelemetry() {

  temperatureWarning = readTemp();
  // random(0, 2);

  speed = readSpeed();
  // random(0, 10001) / 100.0f;

  pressure = readPressure();
  // random(0, 10001) / 100.0f;

  trip = readTrip();
  // random(0, 10001) / 100.0f;

  totalTrip = readTotalTrip();
  // random(0, 50001) / 100.0f;

  h2LeakPPM = readH2PPM();
  //random(0, 50001) / 100.0f;

  TEMP.print("Temperature Warning : ");
  TEMP.println(
    temperatureWarning ? "HIGH TEMPERATURE" : "NORMAL");

  SPEED.print("Speed               : ");
  SPEED.println(speed, 2);

  PRESSURE.print("Pressure            : ");
  PRESSURE.println(pressure, 2);

  TRIP.print("Trip                : ");
  TRIP.println(trip, 2);

  TOTAL_TRIP.print("Total Trip          : ");
  TOTAL_TRIP.println(totalTrip, 2);

  H2.print("h2LeakPPM      : ");
  H2.println(h2LeakPPM, 2);
}


// =====================================================
// SEND CAN MESSAGE
// =====================================================
bool sendCAN(twai_message_t& message) {
  esp_err_t result = twai_transmit(
    &message,
    pdMS_TO_TICKS(100));

  if (result == ESP_OK) {
    return true;
  }

  // CAN.print("CAN TX FAILED, ID = 0x");
  // CAN.print(message.identifier, HEX);
  // CAN.print(" ERROR = ");
  // CAN.println(result);

  return false;
}

// =====================================================
// SEND CONTROL ACK
// =====================================================
void sendControlACK(uint8_t sequence) {
  twai_message_t msg = {};

  msg.identifier = CAN_ID_CONTROL_ACK;
  msg.data_length_code = 8;

  msg.data[0] = sequence;
  msg.data[1] = MSG_CONTROL;
  msg.data[2] = 0x00;

  for (int i = 3; i < 8; i++)
    msg.data[i] = 0;

  if (sendCAN(msg)) {
    CAN.println(
      "BACK -> CONTROL ACK SENT");
  }
}


// =====================================================
// PROCESS FRONT → BACK CONTROL PACKET
// =====================================================
void processControlPacket(twai_message_t& msg) {
  if (msg.data_length_code != 8)
    return;

  if (msg.data[1] != MSG_CONTROL)
    return;

  uint8_t sequence =
    msg.data[0];

  bool ignition =
    msg.data[2] != 0;

  bool h2Supply =
    msg.data[3] != 0;

  bool emergency =
    msg.data[4] != 0;


  CONTROL.println();
  CONTROL.println("========================================");
  CONTROL.println("BACK RECEIVED CONTROL");
  CONTROL.println("========================================");

  CONTROL.print("Sequence     : ");
  CONTROL.println(sequence);

  CONTROL.print("Ignition     : ");
  CONTROL.println(
    ignition ? "ON" : "OFF");

  CONTROL.print("H2 Supply    : ");
  CONTROL.println(
    h2Supply ? "ON" : "OFF");

  CONTROL.print("Emergency    : ");
  CONTROL.println(
    emergency ? "ON" : "OFF");

  // if (emergency) {
  //   ESP.restart();
  // }

  // for (int i = 0; i < 8; i++) {
  //   if (msg.data[i] < 0x10)
  //     CONTROL.print("0");

  //   if (h2Supply && !emergency) {
  //     digitalWrite(H2_CUTOFF, HIGH);
  //     CONTROL.println("H2 CUTOFF: ON");
  //   } else {
  //     digitalWrite(H2_CUTOFF, LOW);
  //     CONTROL.println("H2 CUTOFF: OFF");
  //   }
  // }


  if (emergency) {
    digitalWrite(H2_CUTOFF, LOW);
    CONTROL.println("H2_CUTOFF DUE TO EMERGENCY");
    sendControlACK(sequence);
    ESP.restart();
    return;
  }
  if (h2Supply) {
    digitalWrite(H2_CUTOFF, HIGH);
    CONTROL.println("H2_CUTOFF : ON");
  } else {
    digitalWrite(H2_CUTOFF, LOW);
    CONTROL.println("H2_CUTOFF : OFF");
  }

  // CONTROL.print(
  //   msg.data[i],
  //   HEX);
  // CONTROL.print(" ");
  // CONTROL.println();
  
  // Send ACK
  sendControlACK(sequence);
}


// =====================================================
// PROCESS ACK FROM FRONT
// =====================================================
void processTelemetryACK(twai_message_t& msg) {
  uint32_t id =
    msg.identifier;

  uint8_t sequence =
    msg.data[0];

  uint8_t type =
    msg.data[1];

  uint8_t status =
    msg.data[2];


  if (status != 0x00) {
    CONTROL.println(
      "Telemetry ACK rejected");

    return;
  }


  // ---------------------------------------------
  // Packet 1 ACK
  // ---------------------------------------------
  if (id == CAN_ID_TEMP_SPEED_ACK && type == MSG_TEMP_SPEED) {
    if (sequence == telemetrySequence) {
      CAN.print("ACK : Packet 1 : 0x");
      CAN.println(msg.identifier, HEX);
      CAN.println(
        "BACK: Packet 1 ACK received");

      telemetryState =
        SEND_PACKET_2;

      retryCount = 0;
    }

    return;
  }


  // ---------------------------------------------
  // Packet 2 ACK
  // ---------------------------------------------
  if (id == CAN_ID_PRESSURE_TRIP_ACK && type == MSG_PRESSURE_TRIP) {
    if (sequence == telemetrySequence) {
      CAN.println(
      "BACK: Packet 2 ACK received");

      telemetryState =
        SEND_PACKET_3;

      retryCount = 0;
    }

    return;
  }


  // ---------------------------------------------
  // Packet 3 ACK
  // ---------------------------------------------
  if (id == CAN_ID_TOTAL_TRIP_ACK && type == MSG_TOTAL_TRIP) {
    if (sequence == telemetrySequence) {
      CAN.println(
        "BACK: Packet 3 ACK received");

      telemetryState =
        TELEMETRY_DONE;

      retryCount = 0;
    }

    return;
  }
}


// =====================================================
// RECEIVE CAN MESSAGES
//
// This runs continuously.
// This is what allows simultaneous communication.
// =====================================================
void receiveCAN() {
  twai_message_t msg;

  while (twai_receive(
           &msg,
           0)
         == ESP_OK) {
    if (msg.extd)
      continue;

    if (msg.identifier == CAN_ID_CONTROL) {
      processControlPacket(msg);
    } else if (
      msg.identifier == CAN_ID_TEMP_SPEED_ACK || msg.identifier == CAN_ID_PRESSURE_TRIP_ACK || msg.identifier == CAN_ID_TOTAL_TRIP_ACK) {
      processTelemetryACK(msg);
    }
  }
}




// =====================================================
// SEND PACKET 1
// =====================================================
void sendPacket1() {
  twai_message_t msg = {};

  msg.identifier =
    CAN_ID_TEMP_SPEED;

  msg.data_length_code = 8;

  msg.data[0] =
    telemetrySequence;

  msg.data[1] =
    MSG_TEMP_SPEED;

  msg.data[2] =
    temperatureWarning ? 1 : 0;

  memcpy(
    &msg.data[3],
    &speed,
    sizeof(float));

  msg.data[7] = 0;

  CAN.println();
  CAN.println(
    "BACK -> PACKET 1");
  CAN.println("Packet 1 is sent");
  CAN.print("Temperature : ");
  CAN.println( temperatureWarning ? "WARNING" : "NORMAL");
  CAN.print( "Speed     : ");
  CAN.println( speed, 2);
  sendCAN(msg);
  ackStartTime = millis();
  telemetryState =
    WAIT_PACKET_1_ACK;
}


// =====================================================
// SEND PACKET 2
// =====================================================
void sendPacket2() {
  twai_message_t msg = {};

  msg.identifier =
    CAN_ID_PRESSURE_TRIP;

  msg.data_length_code = 8;

  uint16_t pressureScaled =
    (uint16_t)(pressure * 100.0f);

  uint16_t tripScaled =
    (uint16_t)(trip * 100.0f);

  msg.data[0] =
    telemetrySequence;

  msg.data[1] =
    MSG_PRESSURE_TRIP;

  msg.data[2] =
    pressureScaled & 0xFF;

  msg.data[3] =
    (pressureScaled >> 8) & 0xFF;


  msg.data[4] =
    tripScaled & 0xFF;

  msg.data[5] =
    (tripScaled >> 8) & 0xFF;


  msg.data[6] = 0;
  msg.data[7] = 0;
  CAN.println();
  CAN.println(
    "BACK -> PACKET 2");

  CAN.print(
    "Pressure : ");

  CAN.println(
    pressure,
    2);

  CAN.print("Trip     : ");

  CAN.println( trip, 2);
  sendCAN(msg);

  ackStartTime =
    millis();

  telemetryState =
    WAIT_PACKET_2_ACK;
}


// =====================================================
// SEND PACKET 3
// =====================================================
void sendPacket3() {
  twai_message_t msg = {};

  msg.identifier = CAN_ID_TOTAL_TRIP;
  msg.data_length_code = 8;

  msg.data[0] = telemetrySequence;
  msg.data[1] = MSG_TOTAL_TRIP;

  memcpy(
    &msg.data[2],
    &totalTrip,
    sizeof(float));

  uint32_t h2LeakValue =
    (uint32_t)(h2LeakPPM);

  msg.data[6] =
    (h2LeakValue >> 8) & 0xFF;

  msg.data[7] =
    h2LeakValue & 0xFF;

  CAN.println();
  CAN.println("BACK -> PACKET 3");

  CAN.print("Total Trip : ");
  CAN.println(totalTrip, 2);

  CAN.print("H2 Leak PPM: ");
  CAN.println(h2LeakPPM);

  CAN.print("RAW DATA   : ");

  for (int i = 0; i < 8; i++) {
    if (msg.data[i] < 0x10)
      CAN.print("0");

    CAN.print(msg.data[i], HEX);
    CAN.print(" ");
  }

  sendCAN(msg);

  ackStartTime = millis();
  telemetryState = WAIT_PACKET_3_ACK;
}


// =====================================================
// TELEMETRY STATE MACHINE
// =====================================================
void handleTelemetry() {
  switch (telemetryState) {
    // -----------------------------------------
    // Start new telemetry set
    // -----------------------------------------
    case SEND_PACKET_1:
      generateTelemetry();
      retryCount = 0;
      sendPacket1();
      break;


    // -----------------------------------------
    // Waiting for Packet 1 ACK
    // -----------------------------------------
    case WAIT_PACKET_1_ACK:

      if (millis() - ackStartTime > ACK_TIMEOUT_MS) {

        if (retryCount < MAX_RETRIES) {
          retryCount++;

          TELEMETRY.println(
            "Packet 1 ACK timeout - retry");

          sendPacket1();
        } else {
          TELEMETRY.println("Packet 1 failed");

          telemetryState =
            SEND_PACKET_2;

          retryCount = 0;
        }
      }

      break;


    // -----------------------------------------
    // Send Packet 2
    // -----------------------------------------
    case SEND_PACKET_2:

      retryCount = 0;

      sendPacket2();

      break;


    // -----------------------------------------
    // Waiting for Packet 2 ACK
    // -----------------------------------------
    case WAIT_PACKET_2_ACK:

      if (millis() - ackStartTime > ACK_TIMEOUT_MS) {
        if (retryCount < MAX_RETRIES) {
          retryCount++;

          TELEMETRY.println(
            "Packet 2 ACK timeout - retry");

          sendPacket2();
        } else {
          TELEMETRY.println(
            "Packet 2 failed");

          telemetryState =
            SEND_PACKET_3;

          retryCount = 0;
        }
      }

      break;


    // -----------------------------------------
    // Send Packet 3
    // -----------------------------------------
    case SEND_PACKET_3:

      retryCount = 0;

      sendPacket3();

      break;


    // -----------------------------------------
    // Waiting for Packet 3 ACK
    // -----------------------------------------
    case WAIT_PACKET_3_ACK:

      if (millis() - ackStartTime > ACK_TIMEOUT_MS) {
        if (retryCount < MAX_RETRIES) {
          retryCount++;

          TELEMETRY.println(
            "Packet 3 ACK timeout - retry");

          sendPacket3();
        } else {
          TELEMETRY.println(
            "Packet 3 failed");

          telemetryState =
            TELEMETRY_DONE;
        }
      }

      break;


    // -----------------------------------------
    // Complete
    // -----------------------------------------
    case TELEMETRY_DONE:

      if (millis() - telemetryStartTime > 2000) {
        telemetrySequence++;

        telemetryStartTime =
          millis();

        telemetryState =
          SEND_PACKET_1;
      }

      break;
  }
}


// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  rtd.begin();
  rtd.setReferenceResistor(RREF);
  rtd.setR0(R0);
  analogReadResolution(12);
  // initializeH2Sensor();
  pinMode(H2_CUTOFF, OUTPUT);  // h2_cutoff
  digitalWrite(H2_CUTOFF, LOW);

  pinMode(H2_DIN_PIN, INPUT);  //  MQ8
  digitalWrite(H2_DIN_PIN, LOW);

  setupTachometer();

  delay(1000);


  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX,
      CAN_RX,
      TWAI_MODE_NORMAL);


  twai_timing_config_t t_config =
    TWAI_TIMING_CONFIG_250KBITS();


  twai_filter_config_t f_config =
    TWAI_FILTER_CONFIG_ACCEPT_ALL();


  if (twai_driver_install(
        &g_config,
        &t_config,
        &f_config)
      != ESP_OK) {
    CAN.println(
      "TWAI installation failed");

    while (1);
  }


  if (twai_start() != ESP_OK) {
    CAN.println(
      "TWAI start failed");

    while (1);
  }
  CAN.println(
    "CAN started at 250 kbps");
  telemetryStartTime =
    millis();
}


// =====================================================
// LOOP
// =====================================================
void loop() {

  receiveCAN();
   readTachometer();
  handleTelemetry();

  delay(1);
}