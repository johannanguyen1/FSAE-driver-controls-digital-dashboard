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
const unsigned long rpmInterval = 500;
const unsigned long coolantInterval = 2000;
const unsigned long batteryInterval = 3000;

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
  processCANMessages();
  unsigned long currentMillis = millis();
  if (currentMillis - lastRPMUpdate >= rpmInterval) {
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
    overheating = coolInTemp > 102;
  } else if (msg.id == 0x104) {
    batteryVoltage = extractFloatFromBuffer(msg.buf);
    fuelUsed = extractFloatFromBuffer(msg.buf + 4);
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

void sendRPM() {
  rpm3dig = rpm/100; // change RPM from 13500 to 135
  int rpm1, rpm2;
  
  if (rpm3dig <= 100) { // GREEN: 0-100 RPM
    rpm1 = rpm3dig; 
    rpm2 = 0; 
  } else if (rpm3dig > 100) { // BLUE(Shift): 101-135 RPM
    rpm1 = 100;
    rpm2 = (rpm3dig % 100) * 100 / 35;
      // Math explained: 
      // (rpm3dig % 100) = 135 - 100
      // y = 135 - 100   and x = ratio from y out of 35, to out of 100
      // (y/35) = (x/100)
      // 100(y/35) = x = rpm2
      // NOTE: This will not work if the Arduino ends up recieving out-of-bound rpm values from MOTEC/CAN (out of bound is b<0 or b>135)
  }
  sendToNextion("rpm1", String(rpm1), true);
  sendToNextion("rpm2", String(rpm2), true);
  sendToNextion("gear", String(gear), true);
}

void sendCoolantTemp() {
  sendToNextion("coolInTemp", String(coolInTemp), true);
  sendToNextion("coolOutTemp", String(coolOutTemp), true);
}

void sendBatteryFuel() {
  sendToNextion("batteryVoltage", String(batteryVoltage), true);
  sendToNextion("fuelUsed", String(fuelUsed), true);
}

void sendToNextion(const String& objectName, const String& value, bool isNumeric) {
  // Serial1.print(objectName + (isNumeric ? ".val=" : ".txt=\"") + value + (isNumeric ? "" : "\""));
  // Serial1.write(0xFF);
  // Serial1.write(0xFF);
  // Serial1.write(0xFF);

  printf("%s: %s\n", objectName.c_str(), value.c_str());
}
