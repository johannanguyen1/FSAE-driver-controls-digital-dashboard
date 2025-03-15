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
const unsigned long batteryInterval = 5000;

unsigned int rpm, rpm1, rpm2, rpm3dig, gear, coolInTemp, coolOutTemp, batteryVoltage, fuelUsed;
bool overheating;


void setup() {
  pinMode(CS_Pin, OUTPUT);
  pinMode(INTRPT_Pin, INPUT);
  Serial1.begin(9600);

  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    attachInterrupt(digitalPinToInterrupt(INTRPT_Pin), canISR, FALLING);
  } else {
    sendErrorCase("#3 CAN Module not initialized");
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
      else {
        sendErrorCase("#1 Buffer overflow, dropped"); //does this NEED to be an error message that pauses everything?
      }
    }
  } else {
    sendErrorCase("#2 CAN message not available"); //does this NEED to be an error message that pauses everything?
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
  } else { //double check if this even needs to be an error case, or simply ignored
    //sendErrorCase("#4 message ids not found");
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
  rpm3dig = rpm/100; // change RPM from 13500 to 135
  // NOTE: If out of bounds, meaning a value is sent less than 0 or greater than 100, nextion will recieve the val as 0 for the progress bar in question
  if (rpm3dig <= 100) { // GREEN: 0-100 RPM, Out of Bound ==> when less than 0, just sends 0 for both progress bars
    rpm1 = rpm3dig; 
    rpm2 = 0; 
  } else if (135 >= rpm3dig || rpm3dig > 100) { // RED(Shift): 101-135 RPM
    rpm1 = 100;
    rpm2 = (rpm3dig % 100) * 100 / 35;
      // Math explained: 
      // (rpm3dig % 100) = 135 - 100 = 1 through 35
      // (y/35) = (x/100)
      // y = 135 - 100   and   x = ratio from y out of 35, to out of 100 for the progress bar
      // 100(y/35) = x = rpm2
  } else if (rpm3dig > 135) { // Out of Bound ==> when RPM > 135, maxes out both progress bars
    rpm1, rpm2 = 100; 
  }
  sendToNextion("rpm", rpm, true);
  sendToNextion("rpm1", rpm1, true);
  sendToNextion("rpm2", rpm2, true);
  sendToNextion("gear", gear, true);
}
void sendCoolantTemp() {
  sendToNextion("coolInTemp", coolInTemp, true);
  sendToNextion("coolOutTemp", coolOutTemp, true);
  if (overheating) {
    sendToNextion("overheating", "Overheating!", false);
  }
}
void sendBatteryFuel() {
  sendToNextion("batteryVoltage", batteryVoltage, true);
  sendToNextion("fuelUsed", fuelUsed, true);
}

void sendToNextion(const String& objectName, const String& value, bool isNumeric) {
  Serial1.print(objectName + (isNumeric ? ".val=" : ".txt=\"") + value + (isNumeric ? "" : "\""));
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

//--------------------SOMETHING FAILED, DISPLAY ERROR MESSAGE ON DASH--------------------------------------
void sendErrorCase(const String& message) { // message must be 30 characters or LESS
  sendToNextion("rpm1", 0, true);
  sendToNextion("rpm2". 0, true);
  sendToNextion("rpm", 0, true);
  sendToNextion("gear", 0, true);
  sendToNextion("batteryVoltage", 0, true);
  sendToNextion("coolInTemp", 0, true);
  sendToNextion("coolOutTemp", 0, true);
  sendToNextion("fuelUsed", 0, true);
  sentToNextion("overheating", message, false);
}
