#include <mcp_can.h>  // necessary for CAN Protocol communication commands
#include <SPI.h>      // necessary for serial communication between the SPI devices and the MicroController

// Define the pins between the MCP2515 Board and the MicroController
#define CS_Pin 10
#define INTRPT_Pin 9  // or 0

// Variables for data metrics
unsigned int engnSpeed;  // RPM will be stored here as an integer
unsigned int gear;


// Variables for tracking last sent values
static String lastRPM, lastGear;

// Timing variables
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 200; // Update every 200ms

// Flag to indicate CAN initialization
bool can_initialized = false;

// Create an instance of MCP_CAN with the CS pin
MCP_CAN CAN(CS_Pin);

void setup() {
  // Input and Output pin setups
  pinMode(CS_Pin, OUTPUT);
  pinMode(INTRPT_Pin, INPUT);

  // Initialize Serial for debugging
  Serial1.begin(9600);

  // Start the CAN bus at 250 kbps (using MCP_CAN's specific start command)
  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    can_initialized = true;
  } else {
    while (1);  // Stop if CAN initialization fails
  }

  // Set CAN mode to normal operation
  CAN.setMode(MCP_NORMAL);
  delay(1000);
}

void loop() {
  // Check for new CAN messages
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    receiveCANMessage();
  }

  // CAN re-initialization logic (if needed)
  if (!can_initialized) {
    if (millis() - lastUpdateTime > updateInterval) {
      if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
        can_initialized = true;
      }
      lastUpdateTime = millis();
    }
  }
}


void receiveCANMessage() {
  unsigned long canId;
  unsigned char len = 0;
  unsigned char buf[8];

  CAN.readMsgBuf(&canId, &len, buf);

  if (canId == 0x102) {  // Assuming 0x102 is the CAN ID for RPM and Gear data
    engnSpeed = extractFloatFromBuffer(buf) / 6;  // Convert to RPM

    gear = buf[7];  // Extract gear value

    // Send data to Nextion
    sendToNextionIfChanged("rpm", String(engnSpeed), lastRPM, true);
    sendToNextionIfChanged("gear", String(gear), lastGear, true);
  }
}

// Helper function to convert CAN buffer to float
float extractFloatFromBuffer(unsigned char* buf) {
  union {
    uint32_t bits;
    float number;
  } data;

  data.bits = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
  return data.number;
}

void sendToNextion(const String& objectName, const String& value, bool isNumeric) {
  if (isNumeric) {
    Serial1.print(objectName + ".val=");  // Command for setting numeric value
    Serial1.print(value);                // Send the value as a number
  } else {
    Serial1.print(objectName + ".txt=\"");  // Command for setting text value
    Serial1.print(value);                   // Send the value as a string
    Serial1.print("\"");
  }
  // End of command sequence (required by Nextion protocol)
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void sendToNextionIfChanged(const String& objectName, const String& value, String& lastValue, bool isNumeric) {
  if (value != lastValue) {  // Only update if the value has changed
    sendToNextion(objectName, value, isNumeric);
    lastValue = value;  // Update the cached value
  }
}



