#include <Wire.h>

#define I2C_ADDRESS 0x10

//--------------------------------------------------
// ADC Pins
//--------------------------------------------------

#define CELL1_PIN A0
#define CELL2_PIN A1
#define CELL3_PIN A2
#define TEMP_PIN  A3

//--------------------------------------------------
// Balance MOSFET Pins
//--------------------------------------------------

#define BAL1_PIN 5
#define BAL2_PIN 6
#define BAL3_PIN 7

//--------------------------------------------------
// Divider Ratios
//--------------------------------------------------

#define NODE1_SCALE 0.767f
#define NODE2_SCALE 0.600f
#define NODE3_SCALE 0.357f

//--------------------------------------------------
// Measurement Packet
//--------------------------------------------------

struct __attribute__((packed)) Packet_t
{
    uint8_t start;

    uint8_t module_id;

    uint16_t cell1_mv;
    uint16_t cell2_mv;
    uint16_t cell3_mv;

    uint16_t temp_x10;

    uint8_t fault;

    uint8_t checksum;
};

Packet_t txPacket;

//--------------------------------------------------
// Balance Command Packet
//--------------------------------------------------

struct __attribute__((packed)) BalanceCmd_t
{
    uint8_t start;

    uint8_t balance_mask;
};

BalanceCmd_t rxCmd;

//--------------------------------------------------
// Checksum
//--------------------------------------------------

uint8_t calculateChecksum(uint8_t *data,
                          uint8_t length)
{
    uint8_t sum = 0;

    for(uint8_t i=0;i<length;i++)
    {
        sum += data[i];
    }

    return sum;
}

//--------------------------------------------------
// Voltage Read
//--------------------------------------------------

float readNodeVoltage(uint8_t pin,
                      float scale)
{
    uint16_t adc = analogRead(pin);

    float adcVoltage =
        (adc * 5.0f) / 1023.0f;

    return adcVoltage / scale;
}

//--------------------------------------------------
// Temperature Read
//--------------------------------------------------

uint16_t readTemperature()
{
    return analogRead(TEMP_PIN);
}

//--------------------------------------------------
// STM32 Read Request
//--------------------------------------------------

void requestEvent()
{
    Wire.write(
        (uint8_t*)&txPacket,
        sizeof(Packet_t));
}

//--------------------------------------------------
// STM32 Write Command
//--------------------------------------------------

void receiveEvent(int bytes)
{
    if(bytes == sizeof(BalanceCmd_t))
    {
        Wire.readBytes(
            (char*)&rxCmd,
            sizeof(BalanceCmd_t));
    }
}

//--------------------------------------------------
// Setup
//--------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(BAL1_PIN, OUTPUT);
    pinMode(BAL2_PIN, OUTPUT);
    pinMode(BAL3_PIN, OUTPUT);

    digitalWrite(BAL1_PIN, LOW);
    digitalWrite(BAL2_PIN, LOW);
    digitalWrite(BAL3_PIN, LOW);

    txPacket.start = 0xAA;
    txPacket.module_id = 1;

    Wire.begin(I2C_ADDRESS);

    Wire.onRequest(requestEvent);

    Wire.onReceive(receiveEvent);
}

//--------------------------------------------------
// Loop
//--------------------------------------------------

void loop()
{
    //------------------------------------------
    // Voltage Measurement
    //------------------------------------------

    float node1 =
        readNodeVoltage(
            CELL1_PIN,
            NODE1_SCALE);

    float node2 =
        readNodeVoltage(
            CELL2_PIN,
            NODE2_SCALE);

    float node3 =
        readNodeVoltage(
            CELL3_PIN,
            NODE3_SCALE);

    float cell1 = node1;

    float cell2 = node2 - node1;

    float cell3 = node3 - node2;

    txPacket.cell1_mv =
        (uint16_t)(cell1 * 1000);

    txPacket.cell2_mv =
        (uint16_t)(cell2 * 1000);

    txPacket.cell3_mv =
        (uint16_t)(cell3 * 1000);

    //------------------------------------------
    // Temperature
    //------------------------------------------

    txPacket.temp_x10 =
        readTemperature();

    //------------------------------------------
    // Fault Byte
    //------------------------------------------

    txPacket.fault = 0;

    //------------------------------------------
    // Checksum
    //------------------------------------------

    txPacket.checksum =
        calculateChecksum(
            (uint8_t*)&txPacket,
            sizeof(Packet_t)-1);

    //------------------------------------------
    // Execute Balancing Command
    //------------------------------------------

    if(rxCmd.start == 0xAA)
    {
        digitalWrite(
            BAL1_PIN,
            (rxCmd.balance_mask & 0x01));

        digitalWrite(
            BAL2_PIN,
            (rxCmd.balance_mask & 0x02));

        digitalWrite(
            BAL3_PIN,
            (rxCmd.balance_mask & 0x04));
    }

    //------------------------------------------
    // Debug Print
    //------------------------------------------

    Serial.print("C1=");
    Serial.print(txPacket.cell1_mv);

    Serial.print(" C2=");
    Serial.print(txPacket.cell2_mv);

    Serial.print(" C3=");
    Serial.print(txPacket.cell3_mv);

    Serial.print(" Mask=");
    Serial.println(rxCmd.balance_mask,
                   BIN);

    delay(100);
}