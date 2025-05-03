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

unsigned long lastRPMUpdate = 0;
unsigned long lastCoolantUpdate = 0;
unsigned long lastBatteryUpdate = 0;
unsigned long lastGearUpdate = 0;
const unsigned long rpmInterval = 500;
const unsigned long coolantInterval = 2000;
const unsigned long batteryInterval = 5000;
const unsigned long gearInterval = 500;

unsigned int rpm, rpm1, rpm2, rpm3dig, gear, coolInTemp, coolOutTemp, batteryVoltage, fuelUsed;
bool overheating;

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
}

void loop() {
  // Change Pages w/ dial
  float pageVolt = analogRead(pagePin) * (3.3 / 1023.0); // from 0 to 1023 --> from 0.0 to 3.3
  unsigned int currentPage = (pageVolt < 1.65) ? 0 : 1; // page0: <1.65  page2>=1.65
  if (currentPage != lastPage) { 
    Serial1.print("page" + String(currentPage)); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    lastPage = currentPage;
  }

  processCANMessages();
  unsigned long currentMillis = millis();
  if (currentMillis - lastRPMUpdate >= rpmInterval) { //RPM
    sendRPM();
    lastRPMUpdate = currentMillis;
  }
  if (currentMillis - lastCoolantUpdate >= coolantInterval) {
    sendCoolantTemp();
    lastCoolantUpdate = currentMillis;
  }
  if (currentMillis - lastBatteryUpdate >= batteryInterval) {
    sendBatteryFuel();
    lastBatteryUpdate = currentMillis;
  }
  if (currentMillis - lastGearUpdate >= gearInterval) {
    sendGear();
    lastGearUpdate = currentMillis;
  }
}

void canISR() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CANMessage msg;
    CAN.readMsgBuf(&msg.id, &msg.len, msg.buf);
    int nextHead = (bufferHead + 1) % BUFFER_SIZE;
    if (nextHead != bufferTail) {
      canBuffer[bufferHead] = msg;
      bufferHead = nextHead;
    } 
  }
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
  if (msg.id == 0x102) {
    rpm = extractFloatFromBuffer(msg.buf) / 6;
    gear = msg.buf[7];
  } else if (msg.id == 0x103) {
    coolInTemp = extractFloatFromBuffer(msg.buf);
    coolOutTemp = extractFloatFromBuffer(msg.buf + 4);
    if (coolInTemp > 102) {
      overheating = true;
    }
  } else if (msg.id == 0x104) {
    batteryVoltage = extractFloatFromBuffer(msg.buf) * 100;
    fuelUsed = extractFloatFromBuffer(msg.buf + 4) * 100;
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
  sendToNextion("rpm", rpm, true);
  sendToNextion("rpm1", rpm1, true);
  sendToNextion("rpm2", rpm2, true);
}

void sendCoolantTemp() {
  sendToNextion("coolInTemp", coolInTemp, true);
  sendToNextion("coolOutTemp", coolOutTemp, true);
  if (overheating) {
    sendToNextion("overheating", "!!!!!!!!Overheating!!!!!!!!", false);
  }
}

void sendBatteryFuel() {
  sendToNextion("batteryVoltage", batteryVoltage, true);
  sendToNextion("fuelUsed", fuelUsed, true);
}

void sendGear() {
  sendToNextion("gear", gear, true);
}

void sendToNextion(const String& objectName, const String& value, bool isNumeric) {
  Serial1.print(objectName + (isNumeric ? ".val=" : ".txt=\"") + value + (isNumeric ? "" : "\""));
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}
