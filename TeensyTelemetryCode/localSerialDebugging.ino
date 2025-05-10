#include <mcp_can.h>
#include <SPI.h>

#define CS_Pin 10
#define INTRPT_Pin 9

struct CANMessage {
  unsigned long id;
  unsigned char len;
  unsigned char buf[8];
};

#define BUFFER_SIZE 10
CANMessage canBuffer[BUFFER_SIZE];
volatile int bufferHead = 0;
volatile int bufferTail = 0;

MCP_CAN CAN(CS_Pin);

unsigned long last300Update = 0;
unsigned long last1000Update = 0;
unsigned long last5000Update = 0;
const unsigned long interval300 = 300;
const unsigned long interval1000 = 1000;
const unsigned long interval5000 = 5000;

unsigned int rpm, rpm1, rpm2, rpm3dig, gear, coolInTemp, coolOutTemp, batteryVoltage, fuelUsed;

const int pagePin = A9;
unsigned int lastPage = -1;

void setup() {
  pinMode(CS_Pin, OUTPUT);
  pinMode(INTRPT_Pin, INPUT);
  Serial.begin(9600);

  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    attachInterrupt(digitalPinToInterrupt(INTRPT_Pin), canISR, FALLING);
    Serial.println("CAN init OK");
  } else {
    Serial.println("CAN init FAIL");
    while (1);
  }

  CAN.setMode(MCP_NORMAL);
  delay(1000);

  Serial.println("=== TEST PAGE SETUP ===");
  Serial.println("Battery | CoolIn | CoolOut");
  Serial.println("FuelUsed");
}

void loop() {
  float pageVolt = analogRead(pagePin) * (3.3 / 1023.0);
  unsigned int currentPage = (pageVolt < 1.65) ? 0 : 1;
  if (currentPage != lastPage) {
    Serial.print("Changing to page "); Serial.println(currentPage);
    lastPage = currentPage;
  }

  processCANMessages();
  unsigned long currentMillis = millis();
  if (currentMillis - last300Update >= interval300) {
    sendRPM();
    sendGear();
    last300Update = currentMillis;
  }
  if (currentMillis - last1000Update >= interval1000) {
    sendCoolantTemp();
    last1000Update = currentMillis;
  }
  if (currentMillis - last5000Update >= interval5000) {
    sendBattery();
    sendFuel();
    last5000Update = currentMillis;
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

//--------------------Simulated Output via Serial (for testing)-----------------------------
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
  Serial.print("[RPM] RPM: "); Serial.print(rpm);
  Serial.print(" | rpm1: "); Serial.print(rpm1);
  Serial.print(" | rpm2: "); Serial.println(rpm2);
}

void sendCoolantTemp() {
  Serial.print("[Coolant] In: "); Serial.print(coolInTemp);
  Serial.print(" | Out: "); Serial.println(coolOutTemp);
  if (coolInTemp > 99.0) {
    Serial.println("WARNING: OVERHEATING");
  }
}

void sendBattery() {
  Serial.print("[Battery] Voltage: ");
  Serial.println(batteryVoltage);
}

void sendFuel() {
  Serial.print("[Fuel] Used: ");
  Serial.println(fuelUsed);
}

void sendGear() {
  Serial.print("[Gear] Current Gear: ");
  Serial.println(gear);
}
