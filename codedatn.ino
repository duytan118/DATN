#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ThingerESP32.h>
#include <arduino_secrets.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  // Tạo đối tượng ADS1115

// Định nghĩa UART port
HardwareSerial DWIN(2);

// Định nghĩa địa chỉ và các pin

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
unsigned long previousMillis = 0;
const long interval = 1000; // 1 giây
int totalSeconds;
int remainingMinutes;
int remainingSeconds;
uint16_t minutes = 0;
uint16_t seconds = 0;
uint16_t flowRate = 0; 

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void setup() {
  Serial.begin(115200); 
  thing.add_wifi(SSID, SSID_PASSWORD);
  DWIN.begin(115200, SERIAL_8N1, 16, 17); // RX -> 16, TX -> 17
  Timer1.initialize(interval * 1000);
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
    }
  };
  thing["FeCl3_Pump"] << [](pson &in){
    if(in.is_empty()){
      in = digitalRead(IN_FECL3_1);
    }
    else{
      digitalWrite(IN_FECL3_1, in ? HIGH : LOW);
    }
  };
  thing["Pressure"] >> [](pson &out){
    int16_t adc0 = ads.readADC_SingleEnded(0);
    float voltage = adc0 * 0.1875 / 1000;  // ADS1115 có độ phân giải 0.1875mV/bit
    float p = ((voltage - 0.5) * (10.0 / 4.5))*10;
    out = p; // Read and send pressure sensor data
  };

  thing["Temperature"] >> [](pson &out){
    temperature.requestTemperatures();
    float t = temperature.getTempCByIndex(0);
    out = t; 
  };
}

void loop() {
  requestDataFromDWIN(0x5000);
  if (Buffer[0] == 0x5A && Buffer[1] == 0xA5) {
    minutes = (Buffer[6] << 8) | Buffer[7]; // Lưu giá trị từ Buffer[6] và Buffer[7]
    Serial.print("Minutes (0x5000): ");
    Serial.println(minutes);
  }
  // Yêu cầu và lưu giá trị lưu lượng từ địa chỉ 0x5100
  requestDataFromDWIN(0x5100);
  if (Buffer[0] == 0x5A && Buffer[1] == 0xA5) {
    flowRate = (Buffer[6] << 8) | Buffer[7]; // Lưu giá trị từ Buffer[6] và Buffer[7]
    Serial.print("Flow Rate (0x5100): ");
    Serial.println(flowRate);
  }
  delay(10); 
  sensor_write();
  delay(10);
  control_dwin();
  delay(100);
  thing.handle();
}
void updateTime() {
  if (seconds == 0) {
    if (minutes > 0) {
      minutes--;
      seconds = 59;
    }
  } else {
    seconds--;
  }

  // Cập nhật màn hình với giá trị đếm ngược hiện tại
  Minute[7] = minutes;
  DWIN.write(Minute, 8);
  Second[7] = seconds;
  DWIN.write(Second, 8);
}

void sensor_write() {
  // Gửi dữ liệu nhiệt độ tới DWIN
  temperature.requestTemperatures();
  float t = temperature.getTempCByIndex(0);
  Temperature[6] = highByte((int)t);
  Temperature[7] = lowByte((int)t);
  DWIN.write(Temperature, 8);
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000;  // ADS1115 có độ phân giải 0.1875mV/bit
  float p = ((voltage - 0.5) * (10.0 / 4.5))*10;
  // Gửi dữ liệu áp suất tới DWIN
  Pressure[6] = highByte((int)p);
  Pressure[7] = lowByte((int)p);
  DWIN.write(Pressure, 8);

  // In dữ liệu lên Serial Monitor
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(p);
  Serial.println(" bar");
  Serial.println();
  delay(100);
}
void changePage(uint8_t pageID) {
    uint8_t changePage[8] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x04, 0x00, 0x00};
    changePage[7] = lowByte(pageID);
    DWIN.write(changePage, 8);
}

void requestDataFromDWIN(uint16_t address) {
  uint8_t command[] = {0x5A, 0xA5, 0x03, 0x83, (uint8_t)(address >> 8), (uint8_t)(address & 0xFF)};

  // Gửi lệnh đến DWIN qua UART
  DWIN.write(command, sizeof(command));

  // Đợi phản hồi từ DWIN
  delay(50); // Đợi một chút để DWIN phản hồi

  // Kiểm tra nếu có dữ liệu trả về
  if (DWIN.available() > 0) {
    for (int i = 0; i < 9; i++) {
      Buffer[i] = DWIN.read();
    }
  }
}
void startProcess() {
  temperature.requestTemperatures();
  float t = temperature.getTempCByIndex(0);
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000;  // ADS1115 có độ phân giải 0.1875mV/bit
  float p = ((voltage - 0.5) * (10.0 / 4.5))*10;
  // Đuổi khí
  Serial.println("Start process: Bật bơm khí H2O2 và FeCl3, và xả");
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
  delay(60000);
  digitalWrite(XA, LOW);
  analogWrite(EN_H2O2, flowRate);
  analogWrite(EN_FECL3, flowRate);
  if(p>7)
  delay(minutes*60000);
  analogWrite(EN_H2O2, 0);
  analogWrite(EN_FECL3, 0);
  


  
  // Gọi hàm cập nhật thời gian đếm ngược
  updateTime();
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
        case 0x6500:   
          if (Buffer[8] == 0x01) {
            
          }
          break;

        case 0x6400:
          if (Buffer[8] == 0x01) {
            off();
          }
          break;

        case 0x6600:  
          if (Buffer[8] == 0x01) {
            stop();
          }
          break;

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