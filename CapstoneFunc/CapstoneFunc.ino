//updated March 25 by Zack

#include <SPI.h>
#include <stdint.h>
#include "HX711.h"

#define DEBUG 0
#define TESTING 1

//Pins
#define SCK_SPI 13
#define MISO_SPI 12
#define MOSI_SPI 11

#define UART_FLAG 10
#define ADXL345_CS 9
#define BLUEFRUIT_CS 8
#define BLUEFRUIT_IRQ 7
#define DOUT_PIN 6
#define SCK_PIN 5
#define BLUEFRUIT_RST 4
#define CHARGING_IND_LED 3
#define CompressionSensorPin A1
#define VentilationSensorPin A0
#define ChargingDetectionPin A2

#define VoltageRef 4.991
#define ChargingThreshold 2.95

uint8_t dataArray[9]; //Formatted by representing 8..0 as MSB..LSB

/***************************************************************
 *  ADXL345 requires 3*10 = 30 bits                            *
 *  ADC Data requires 2*10 = 20 bits                           *
 *  Load Cell requires 22 bits                                 *
 *    Total bits = 72 bits = 9 bytes                           *
 *                                                             *
 * X = X acceleration, Y = Y acceleration, Z = Z acceleration, *
 * C = Compression, D = Air Bag Depth, F = Compression Force   *
 *                                                             *
 *      8          7          6          5          4          *
 * [XXXX XXXX, XXYY YYYY, YYYY ZZZZ, ZZZZ ZZCC, CCCC CCCC,     *
 *      3          2          1          0                     *
 *  DDDD DDDD, DDFF FFFF, FFFF FFFF, FFFF FFFF]                *
 *                                                             *
 ***************************************************************/
int rawXADXL345Data;
int rawYADXL345Data;
int rawZADXL345Data;

int rawCompressionData;
int rawVentilationData;

long rawForceData;

HX711 loadcell;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  SensorSetup();
  pinMode(UART_FLAG, OUTPUT);
  pinMode(9, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  pinMode(6, INPUT);
}

void loop() {
  int lungs = 0;
  int heart = 0;
 

  //Get data to send on each pass
  CollectSensorData();

  //Send data via dataArray global
  

  //Receive two bytes for audio

  //Send both bytes to pi
  sendAudioEncodingToPi(lungs, heart);
  
  //Check Battery
  chargingDetection();
  
}


void ADXL345setup () {
  //Power Setup
  digitalWrite(ADXL345_CS, HIGH);
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0x2D);
  SPI.transfer(0x00);
  digitalWrite(ADXL345_CS, HIGH);
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0x2D);
  SPI.transfer(0x10);
  digitalWrite(ADXL345_CS, HIGH);
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0x2D);
  SPI.transfer(0x08);
  digitalWrite(ADXL345_CS, HIGH);
  //Data Setup
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0x31);
  SPI.transfer(0x01);
  digitalWrite(ADXL345_CS, HIGH);
  //Act Setup
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0x27);
  SPI.transfer(0x44);
  digitalWrite(ADXL345_CS, HIGH);
}

void retrieveADXL345Data () {
  digitalWrite(ADXL345_CS, LOW);
  SPI.transfer(0xF2);
  rawXADXL345Data = SPI.transfer(0x00);
  rawXADXL345Data |= (SPI.transfer(0x00) & 0x03) << 8;

  rawYADXL345Data = SPI.transfer(0x00);
  rawYADXL345Data |= (SPI.transfer(0x00) & 0x03) << 8;

  rawZADXL345Data = SPI.transfer(0x00);
  rawZADXL345Data |= (SPI.transfer(0x00) & 0x03) << 8;
  digitalWrite(ADXL345_CS, HIGH);
}


void SensorSetup () {
  SPI.begin();
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  pinMode(ADXL345_CS, OUTPUT);
  pinMode(DOUT_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  ADXL345setup();
  loadcell.begin(DOUT_PIN, SCK_PIN, 32);
}

void CollectSensorData() {
  //Load Cell Reading
  if (loadcell.is_ready()) {
    rawForceData = loadcell.read();
    #if DEBUG
    Serial.print(rawForceData);
    Serial.print(" load cell data | ");
    #endif
  }

  //Acquire ADC Readings
  rawCompressionData = analogRead(CompressionSensorPin);
  rawVentilationData = analogRead(VentilationSensorPin);


  #if TESTING
    double ventVolt = (double)rawCompressionData * VoltageRef/1023.0;
    double compVolt = (double)rawVentilationData * VoltageRef/1023.0;
    double ventilation = 10.355/(ventVolt-0.0981) - 0.12;
    double compression = 9.5686/(compVolt-0.1999) - 0.12;
    Serial.print("Ventilation=");
    Serial.print(ventilation, 3);
    Serial.print(" cm and Compression=");
    Serial.print(compression, 3);
    Serial.print(" cm | ");
  #endif

  //Acquire ADXL345 readings
  retrieveADXL345Data();
  #if TESTING
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
    if (rawXADXL345Data & 0x200) {
      X = rawXADXL345Data - 1024;
    }
    else {
      X = rawXADXL345Data;
    }
    if (rawYADXL345Data & 0x200) {
      Y = rawYADXL345Data - 1024;
    }
    else {
      Y = rawYADXL345Data;
    }
    if (rawZADXL345Data & 0x200) {
      Z = rawZADXL345Data - 1024;
    }
    else {
      Z = rawZADXL345Data;
    }
    X /= 128.0;
    Y /= 128.0;
    Z /= 128.0;
    Serial.print("X is ");
    Serial.print(X, 3);
    Serial.print(", Y is ");
    Serial.print(Y, 3);
    Serial.print(", and Z is ");
    Serial.println(Z, 3);
  #endif
  #if DEBUG
    Serial.print(" Compression data ");
    Serial.print(rawCompressionData);
    Serial.print(" | Vent Data ");
    Serial.print(rawVentilationData);
    Serial.print(" | Acceleration X ");
    Serial.print(rawXADXL345Data, HEX);
    Serial.print(" | Acceleration Y ");
    Serial.print(rawYADXL345Data, HEX);
    Serial.print(" | Acceleration Z ");
    Serial.println(rawZADXL345Data, HEX);
  #endif
   
  dataArray[8] = (rawXADXL345Data >> 2);                                        //XXXX XXXX
  dataArray[7] = ((rawXADXL345Data & 0x03) << 6) | (rawYADXL345Data >> 4);      //XXYY YYYY
  dataArray[6] = ((rawYADXL345Data & 0x0F) << 4) | (rawZADXL345Data >> 6);      //YYYY ZZZZ
  dataArray[5] = ((rawZADXL345Data & 0x3F) << 2) | (rawCompressionData >> 8);   //ZZZZ ZZCC
  dataArray[4] = rawCompressionData & 0xFF;                                     //CCCC CCCC
  dataArray[3] = rawVentilationData >> 2;                                       //VVVV VVVV
  dataArray[2] = ((rawVentilationData & 0x03) << 6) | ((rawForceData >> 18) & 0x3F);  //VVFF FFFF
  dataArray[1] = (rawForceData >> 10) & 0xFF;
  dataArray[0] = (rawForceData >> 2) & 0xFF;
}

//Lungs are left channel and heart is right channel
void sendAudioEncodingToPi(byte lungs, byte heart) {
  digitalWrite(UART_FLAG, HIGH);
  Serial.write(lungs);
  Serial.write(heart);
  digitalWrite(UART_FLAG, LOW);
}

void chargingDetection() {
  int currentLevel = analogRead(ChargingDetectionPin);
  double convertedLevel = ((double)ChargingDetectionPin)/1023 * VoltageRef;
  if (convertedLevel < ChargingThreshold) {
    digitalWrite(CHARGING_IND_LED, HIGH);
  }
  else {
    digitalWrite(CHARGING_IND_LED, LOW);
  }
}
