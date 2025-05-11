/*

Current ifs:

- need to fix the cooling code, calculations, etc
- need to add the rest of the sensors for page 2

*/
#include <mcp_can.h>  // necessary for CAN Protocol communication commands
#include <SPI.h>

#define CS_Pin 10
#define INTRPT_Pin 9  // Interrupt pin

struct CANMessage {
  unsigned long id;
  unsigned char len;
  unsigned char buf[8];
};

#define BUFFER_SIZE 10  // Circular buffer size
CANMessage canBuffer[BUFFER_SIZE];
volatile int bufferHead = 0;
volatile int bufferTail = 0;

MCP_CAN CAN(CS_Pin);

unsigned long last300Update = 0;  // 0.3 seconds
unsigned long last1000Update = 0;  // 1.0 seconds
unsigned long last5000Update = 0;  // 5.0 seconds
const unsigned long interval300 = 300;
const unsigned long interval1000 = 1000;
const unsigned long interval5000 = 5000;

unsigned int rpm, rpm1, rpm2, rpm3dig, gear, coolInTemp, coolOutTemp, batteryVoltage, fuelUsed;
volatile bool canFlag = false;


// Page Values
const int pagePin = A9; // Analog pin for Page Dial
unsigned int lastPage = -1;

void setup() {
  pinMode(CS_Pin, OUTPUT);
  pinMode(INTRPT_Pin, INPUT);
  Serial1.begin(9600);

  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    attachInterrupt(digitalPinToInterrupt(INTRPT_Pin), canISR, FALLING);
  } else {
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  delay(1000);

  // assign testing page's data points
  sendToNextion("nameB2", "Battery", false); sendToNextion("nameB3", "CoolIn", false); sendToNextion("nameB4", "CoolOut", false);
  sendToNextion("nameC1", "FuelUsed", false); sendToNextion("nameC2", "", false); sendToNextion("nameC3", "", false); sendToNextion("nameC4", "", false);
  sendToNextion("nameD1", "", false); sendToNextion("nameD2", "", false); sendToNextion("nameD3", "", false); sendToNextion("nameD4", "", false);
}

void loop() {
  // Change Pages w/ dial
  float pageVolt = analogRead(pagePin) * (3.3 / 1023.0); // from 0 to 1023 --> from 0.0 to 3.3
  unsigned int currentPage = (pageVolt < 1.65) ? 0 : 1; // page0: <1.65  page2>=1.65
  if (currentPage != lastPage) { 
    Serial1.print("page" + String(currentPage)); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    lastPage = currentPage;
  }

  // read CAN messages if interrupt has triggered
  if (canFlag) {
    canFlag = false;
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
      CANMessage msg;
      CAN.readMsgBuf(&msg.id, &msg.len, msg.buf);
      int nextHead = (bufferHead + 1) % BUFFER_SIZE;
      if (nextHead != bufferTail) {
        canBuffer[bufferHead] = msg;
        bufferHead = nextHead;
      }
    }
  }

  processCANMessages();
  unsigned long currentMillis = millis();
  if (currentMillis - last300Update >= interval300) { // 300 - RPM, gear
    sendRPM();
    sendGear();
    last300Update = currentMillis;
  }
  if (currentMillis - last1000Update >= interval1000) { // 1000 - coolant
    sendCoolantTemp();
    last1000Update = currentMillis;
  }
  if (currentMillis - last5000Update >= interval5000) { // 5000 - battery, fuel
    sendBattery();
    sendFuel();
    last5000Update = currentMillis;
  }
}

void canISR() {
  canFlag = true;
}


//-------------------SET VARIABLES FROM CAN PACKET-------------------------------------------
void processCANMessages() {
  while (bufferTail != bufferHead) {
    CANMessage msg = canBuffer[bufferTail];
    bufferTail = (bufferTail + 1) % BUFFER_SIZE;
    handleCANMessage(msg);
  }
}

void handleCANMessage(CANMessage msg) {
  switch (msg.id) {
    case 0x102:
      rpm = extractFloatFromBuffer(msg.buf) / 6;
      gear = msg.buf[7];
      break;
    case 0x103:
      coolInTemp = extractFloatFromBuffer(msg.buf);
      coolOutTemp = extractFloatFromBuffer(msg.buf + 4);
      break;
    case 0x104:
      batteryVoltage = extractFloatFromBuffer(msg.buf);
      fuelUsed = extractFloatFromBuffer(msg.buf + 4);
      break;
  }
}

float extractFloatFromBuffer(unsigned char* buf) {
  union {
    uint32_t bits;
    float number;
  } data;
  data.bits = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
  return data.number;
}

//--------------------SENDING VALUES TO NEXTION---------------------------------------------
void sendRPM() {
  rpm3dig = rpm / 100;
  if (rpm3dig <= 100) {
    rpm1 = rpm3dig;
    rpm2 = 0;
  } else if (rpm3dig <= 135) {
    rpm1 = 100;
    rpm2 = (rpm3dig % 100) * 100 / 35;
  } else {
    rpm1 = 100;
    rpm2 = 100;
  }
  sendToNextion("rpmP1", rpm, false);
  sendToNextion("rpmP2", rpm, false);
  sendToNextion("rpm1", rpm1, true);
  sendToNextion("rpm2", rpm2, true);
}
void sendCoolantTemp() {
  sendToNextion("b3", coolInTemp, false);
  sendToNextion("b4", coolOutTemp, false);
  if (coolInTemp > 99.0) {
    sendToNextion("a2", "OVERHEATING", false);
    Serial1.print("warning.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  } else { 
    sendToNextion("a2", "", false);
    Serial1.print("warning.aph=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  }
}
void sendBattery() {
  sendToNextion("batteryVoltage", batteryVoltage, false);
}
void sendFuel() {
  sendToNextion("c1", fuelUsed, false);
}
void sendGear() {
  sendToNextion("gearP1", gear, true);
  sendToNextion("gearP2", gear, true);
}

void sendToNextion(const String& objectName, const String& value, bool isNumeric) {
  const unsigned long timeout = 100;  // timeout in milliseconds
  const int bytesNeeded = objectName.length() + value.length() + (isNumeric ? 6 : 10); // estimate of bytes to send

  unsigned long start = millis();
  while (Serial1.availableForWrite() < bytesNeeded) {
    if (millis() - start > timeout) return;  // give up if buffer is stuck
  }
  
  Serial1.print(objectName + (isNumeric ? ".val=" : ".txt=\"") + value + (isNumeric ? "" : "\""));
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}
