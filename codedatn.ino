#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ThingerESP32.h>
#include <arduino_secrets.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Ticker.h>
Adafruit_ADS1115 ads;  
HardwareSerial DWIN(2);

#define MIN_ADD 0x52
#define SEC_ADD 0x53
#define TEMP_ADD 0x54
#define PRES_ADD 0x55
#define TEMP 35                           
#define QUAT 18
#define SAY 19
#define QUAY 4
#define PLASMA1 32
#define PLASMA2 33
#define XA 5
// Định nghĩa L298
#define IN_H2O2_1 25
#define IN_H2O2_2 26
#define IN_FECL3_1 2
#define IN_FECL3_2 15
#define EN_H2O2 12
#define EN_FECL3 13
OneWire oneWire(TEMP);
DallasTemperature temperature(&oneWire);

unsigned char Buffer[9];
unsigned char Minute[8] = {0x5a, 0xa5, 0x05, 0x82, MIN_ADD , 0x00, 0x00, 0x00};
unsigned char Second[8] = {0x5a, 0xa5, 0x05, 0x82, SEC_ADD, 0x00, 0x00, 0x00};
unsigned char Temperature[8] = {0x5a, 0xa5, 0x05, 0x82, TEMP_ADD , 0x00, 0x00, 0x00};
unsigned char Pressure[8] = {0x5a, 0xa5, 0x05, 0x82, PRES_ADD, 0x00, 0x00, 0x00};
int minutes;  
int flowRate;  
float t;
float p;
int remainingSeconds;

Ticker countdownTimer;
Ticker sayOffTimer;
Ticker xaOffTimer;

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void setup() {
  Serial.begin(115200); 
  thing.add_wifi(SSID, SSID_PASSWORD);
  DWIN.begin(115200, SERIAL_8N1, 16, 17); // RX -> 16, TX -> 17
  temperature.begin();
  pinMode(QUAT, OUTPUT);
  pinMode(SAY, OUTPUT);
  pinMode(QUAY, OUTPUT);
  pinMode(PLASMA1, OUTPUT);
  pinMode(PLASMA2, OUTPUT);
  pinMode(XA, OUTPUT);
  pinMode(IN_H2O2_1, OUTPUT);
  pinMode(IN_H2O2_2, OUTPUT);
  pinMode(EN_H2O2, OUTPUT);
  pinMode(IN_FECL3_1, OUTPUT);
  pinMode(IN_FECL3_2, OUTPUT);
  pinMode(EN_FECL3, OUTPUT);
  digitalWrite(QUAT, LOW);
  digitalWrite(SAY, LOW);
  digitalWrite(QUAY, LOW);
  digitalWrite(PLASMA1, LOW);
  digitalWrite(PLASMA2, LOW);
  digitalWrite(XA, LOW);
  digitalWrite(IN_H2O2_1, LOW);
  digitalWrite(IN_H2O2_2, LOW);
  digitalWrite(IN_FECL3_1, LOW);
  digitalWrite(IN_FECL3_2, LOW);
  
  thing["XA"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(XA);
    }
    else{
      digitalWrite(XA, in ? HIGH : LOW);
    }
  };
  thing["QUAT"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(QUAT);
    }
    else{
      digitalWrite(QUAT, in ? HIGH : LOW);
    }
  };

  thing["SAY"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(SAY);
    }
    else{
      digitalWrite(SAY, in ? HIGH : LOW);
    }
  };

  thing["QUAY"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(QUAY);
    }
    else{
      digitalWrite(QUAY, in ? HIGH : LOW);
    }
  };

  thing["PLASMA1"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(PLASMA1);
    }
    else{
      digitalWrite(PLASMA1, in ? HIGH : LOW);
    }
  };

  thing["PLASMA2"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(PLASMA2);
    }
    else{
      digitalWrite(PLASMA2, in ? HIGH : LOW);
    }
  };
  thing["H2O2_Pump"] << [](pson &in){
    if(in.is_empty()){
        in = digitalRead(IN_H2O2_1);
    }
    else{
        digitalWrite(IN_H2O2_1, in ? HIGH : LOW);
        analogWrite(EN_H2O2, in ? 255 : 0); // Bật (255) hoặc Tắt (0) EN_H2O2 tương ứng với giá trị của in
    }
  };
  thing["FeCl3_Pump"] << [](pson &in){
    if(in.is_empty()){
        in = digitalRead(IN_FECL3_1);
    }
    else{
        digitalWrite(IN_FECL3_1, in ? HIGH : LOW);
        analogWrite(EN_FECL3, in ? 255 : 0); // Bật (255) hoặc Tắt (0) EN_FECL3 tương ứng với giá trị của in
    }
  };
  thing["Pressure"] >> [](pson &out){
    out = p; 
  };

  thing["Temperature"] >> [](pson &out){
    out = t; 
  };
}

void loop() {
  sensor_write();
  delay(10);
  control_dwin();
  delay(100);
  thing.handle();
}
float getTemperature() {
  temperature.requestTemperatures(); 
  float tempC = temperature.getTempCByIndex(0);
  return tempC;
}

float getPressure() {
  int16_t adc0 = ads.readADC_SingleEnded(0);  // Đọc giá trị từ kênh A0 của ADS1115
  float voltage = adc0 * 0.1875 / 1000;  // ADS1115 có độ phân giải 0.1875mV/bit
  float pressure = (voltage - 0.5) * (10.0 / 4.0);
  return pressure;
}

void sensor_write() {
  t = getTemperature();
  p = getPressure();
  Temperature[6] = highByte((int)t);
  Temperature[7] = lowByte((int)t);
  DWIN.write(Temperature, 8);

  Pressure[6] = highByte((int)p);
  Pressure[7] = lowByte((int)p);
  DWIN.write(Pressure, 8);

  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" bar");
  Serial.println();
  delay(1000);
}
void readData() {
  uint8_t command[6];

  // Read minutes from 0x5000
  command[0] = 0x5A;
  command[1] = 0xA5;
  command[2] = 0x03; // Length
  command[3] = 0x83; // Read command
  command[4] = 0x50;
  command[5] = 0x00;
  DWIN.write(command, 6);
  delay(10);
  if (DWIN.available() >= 8) {
    for (int i = 0; i < 6; i++) DWIN.read();
    minutes = (DWIN.read() << 8) | DWIN.read();
  }
  command[4] = 0x51;
  command[5] = 0x00;
  DWIN.write(command, 6);
  delay(10);
  if (DWIN.available() >= 8) {
    for (int i = 0; i < 6; i++) DWIN.read(); 
    flowRate = (DWIN.read() << 8) | DWIN.read();
  }
  remainingSeconds = minutes * 60;
  Serial.print("Minutes: ");
  Serial.println(minutes);
  Serial.print("Flow Rate: ");
  Serial.println(flowRate);
}
void countdown() {
  if (remainingSeconds > 0) {
    remainingSeconds--;
    uint16_t minutesLeft = remainingSeconds / 60;
    uint16_t secondsLeft = remainingSeconds % 60;
    sendCountdownToDWIN(minutesLeft, secondsLeft);
  } else {
    countdownTimer.detach(); // Dừng bộ đếm thời gian khi hoàn thành
    analogWrite(EN_H2O2, 0);
    analogWrite(EN_FECL3, 0);
    digitalWrite(XA, HIGH);
    xaOffTimer.once(60, xaOff);
    digitalWrite(SAY, HIGH);
    sayOffTimer.once(60, sayOff); // Đặt bộ đếm thời gian để tắt SAY sau 60 giây
    changePage(3);
  }
}

void sendCountdownToDWIN(uint16_t minutes, uint16_t seconds) {
  Minute[6] = highByte(minutes);
  Minute[7] = lowByte(minutes);
  Second[6] = highByte(seconds);
  Second[7] = lowByte(seconds);
  DWIN.write(Minute, 8);
  DWIN.write(Second, 8);
  Serial.print("Countdown: ");
  Serial.print(minutes);
  Serial.print(":");
  if (seconds < 10) Serial.print("0");
  Serial.println(seconds);
}

void startProcess() {
  digitalWrite(QUAT, HIGH);
  digitalWrite(PLASMA1, HIGH);
  digitalWrite(PLASMA2, HIGH);
  digitalWrite(IN_H2O2_1, HIGH);
  digitalWrite(IN_H2O2_2, LOW);
  analogWrite(EN_H2O2, 255);
  digitalWrite(IN_FECL3_1, HIGH);
  digitalWrite(IN_FECL3_2, LOW);
  analogWrite(EN_FECL3, 255);
  digitalWrite(XA, HIGH);
  xaOffTimer.once(60, xaOff); // Đặt bộ đếm thời gian để tắt XA sau 60 giây
  analogWrite(EN_H2O2, flowRate);
  analogWrite(EN_FECL3, flowRate);
  countdownTimer.attach(1, countdown); // Gọi hàm countdown mỗi giây
}

void xaOff() {
  digitalWrite(XA, LOW);
}

void sayOff() {
  digitalWrite(SAY, LOW);
}


void off(){
  digitalWrite(QUAT, LOW);
  digitalWrite(SAY, LOW);
  digitalWrite(QUAY, LOW);
  digitalWrite(PLASMA1, LOW);
  digitalWrite(PLASMA2, LOW);
  digitalWrite(XA, LOW);
  digitalWrite(IN_H2O2_1, LOW);
  digitalWrite(IN_H2O2_2, LOW);
  digitalWrite(IN_FECL3_1, LOW);
  digitalWrite(IN_FECL3_2, LOW);
}
void stop() {
  digitalWrite(SAY, LOW);
  digitalWrite(QUAY, LOW);
  digitalWrite(PLASMA1, LOW);
  digitalWrite(PLASMA2, LOW);
  digitalWrite(XA, LOW);
  digitalWrite(IN_H2O2_1, LOW);
  digitalWrite(IN_H2O2_2, LOW);
  digitalWrite(IN_FECL3_1, LOW);
  digitalWrite(IN_FECL3_2, LOW);
}
void changePage(uint8_t pageID) {
    uint8_t changePage[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x04, 0x00, 0x00};
    changePage[7] = lowByte(pageID);
    DWIN.write(changePage, 8);
}

void control_dwin()
{
  if (DWIN.available())
  {
    for (int i = 0; i <= 8; i++) //this loop will store whole frame in buffer array.
    {
      Buffer[i] = DWIN.read();
    }
 
    if (Buffer[0] == 0X5A)
    {
      uint16_t addr = (Buffer[4] << 8) | Buffer[5];
      switch (addr)
      {
        case 0x6500:   // Start auto
          if (Buffer[8] == 0x01) {
          startProcess();
          }
          break;

        case 0x6400: // off
          if (Buffer[8] == 0x01) {
            off();
          }
          break;

        case 0x6600:  //stop
          if (Buffer[8] == 0x01) {
            stop();
          }
          break;
// manual
        case 0x6510:   
          if (Buffer[8] == 0x01) {
            digitalWrite(PLASMA1, HIGH);
          }
          else{
            digitalWrite(PLASMA1, LOW);
          }
          break;

        case 0x6520:   
          if (Buffer[8] == 0x01) {
            digitalWrite(PLASMA2, HIGH);
          }
          else{
            digitalWrite(PLASMA2, LOW);
          }
          break;

        case 0x6530:   
          if (Buffer[8] == 0x01) {
            digitalWrite(IN_H2O2_1, HIGH);
            digitalWrite(IN_H2O2_2, LOW);
            analogWrite(EN_H2O2, 255);
          }
          else{
            digitalWrite(IN_H2O2_1, LOW);
            digitalWrite(IN_H2O2_2, LOW);
          }
          break;
        case 0x6540:   
          if (Buffer[8] == 0x01) {
            digitalWrite(IN_FECL3_1, HIGH);
            digitalWrite(IN_FECL3_2, LOW);
            analogWrite(EN_FECL3, 255);
          }
          else{
            digitalWrite(IN_FECL3_1, LOW);
            digitalWrite(IN_FECL3_2, LOW);
          }
          break;
        case 0x6550:   
          if (Buffer[8] == 0x01) {
            digitalWrite(QUAT, HIGH);
          }
          else{
            digitalWrite(QUAT, LOW);
          }
          break;
        case 0x6560:   
          if (Buffer[8] == 0x01) {
            digitalWrite(QUAY, HIGH);
          }
          else{
            digitalWrite(QUAY, LOW);
          }
          break;
        case 0x6570:   // sấy
          if (Buffer[8] == 0x01) {
            digitalWrite(SAY, HIGH);
          }
          else{
            digitalWrite(SAY, LOW);
          }
          break;
        case 0x6580:   // xả
          if (Buffer[8] == 0x01) {
            digitalWrite(XA, HIGH);
          }
          else{
            digitalWrite(XA, LOW);
          }
          break;
        default:
          Serial.println("No valid data..");
      }
    }
  }
}
