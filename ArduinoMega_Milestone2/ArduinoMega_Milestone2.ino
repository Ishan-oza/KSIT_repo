/*
 * ==============================================================================
 * MULTI-DEVICE SIMULATOR: Concurrent Four-Device Field Unit
 * Platform: Arduino Mega 2560
 * ==============================================================================
 * ALL-IN-ONE STANDALONE FILE (Zero external .h header files required)
 * REPRESENTS FOUR LOGICAL FIELD DEVICES OVER TWO PHYSICAL INTERFACES:
 *   - Device 1 [d1Data()]: RS-232 (Serial1, 9600 baud) - Continuous Stream (Configurable 1000ms)
 *   - Device 2 [d2Data()]: RS-232 (Serial1, 9600 baud) - Periodic (4s, 2x Large Data)
 *   - Device 3 [d3Data()]: RS-485 (Serial2, 19200 baud) - Fast Polled / Key 3 / Fault Slave
 *   - Device 4 [d4Data()]: RS-485 (Serial2, 19200 baud) - 10-Second Polled / Key 4 Slave
 *
 * SERIAL MONITOR COMMANDS FOR ISOLATED TOT & AUTO-RECOVERY:
 *   - Enter [1]: Pause/Resume Device 1 ONLY -> Gateway shows DEV 1 TOT & Auto-Recovery
 *   - Enter [2]: Pause/Resume Device 2 ONLY -> Gateway shows DEV 2 TOT & Auto-Recovery
 *   - Enter [3]: Pause/Resume Device 3 ONLY -> Gateway shows DEV 3 Poll TOT & Auto-Recovery
 *   - Enter [4]: Pause/Resume Device 4 ONLY -> Gateway shows DEV 4 Poll TOT & Auto-Recovery
 *
 * DYNAMIC SPEED & RATE ADJUSTMENT:
 *   - Enter [S]: Slow Mode (Dev 1 = 2000ms / 2.0s - extended telemetry pace)
 *   - Enter [M]: Medium Mode (Dev 1 = 1000ms / 1.0s - standard readable)
 *   - Enter [F]: Fast Mode (Dev 1 = 300ms - high-rate stress test)
 *   - Enter [+]: Increase speed (decrease interval by 200ms)
 *   - Enter [-]: Decrease speed (increase interval by 500ms)
 *
 * DATA RECOVERY & RECONSTRUCTION TESTS:
 *   - Enter [G]: Force Sequence Gap (+2) on all devices (Tests Average Recovery Engine)
 *   - Enter [C]: Corrupt Token 'P_R' (missing W) on Dev 2 (Tests Schema Auto-Repair via P = V * I)
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==============================================================================
// 0. CONFIGURABLE TIMING PARAMETERS (CHANGE SPEEDS HERE AT TOP)
// ==============================================================================
#define DEFAULT_DEV1_INTERVAL_MS   1000   // 1.0s readable stream (Prevents terminal scrolling too fast!)
#define DEFAULT_DEV2_INTERVAL_MS   4000   // 4.0s large periodic burst
#define LCD_INTERVAL_MS            400    // Mega LCD refresh interval (400ms)

unsigned long dev1IntervalMs = DEFAULT_DEV1_INTERVAL_MS;
unsigned long dev2IntervalMs = DEFAULT_DEV2_INTERVAL_MS;

// Individual Device Pause Flags for Isolated TOT Testing
bool dev1Paused = false;
bool dev2Paused = false;
bool dev3Paused = false;
bool dev4Paused = false;

// Token Corruption Test Flag for Dev 2 (Tests character & field recovery: P_R -> PWR[R])
bool corruptNextDev2Token = false;

// Numerical Value Corruption Test Flag for Dev 1 (Tests missing numerical value: T=?C -> T=xx.xC[R])
bool corruptNextDev1Value = false;

// ==============================================================================
// 1. INTEGRATED PACKET PROTOCOL DEFINITIONS (MILESTONE 2)
// ==============================================================================
#define FRAME_SOF           0xAA
#define FRAME_EOF           0x55
#define MAX_PAYLOAD_SIZE    96

// Interface IDs
#define IFACE_RS232         0x01
#define IFACE_RS485         0x02

// Logical Device IDs
#define DEV_ID_GATEWAY      0x00
#define DEV_ID_1            0x01  // RS-232 Continuous
#define DEV_ID_2            0x02  // RS-232 Periodic Large
#define DEV_ID_3            0x03  // RS-485 Fast Polled / Key 3 / Fault Slave
#define DEV_ID_4            0x04  // RS-485 10-Second Polled / Key 4 Slave

// Message Types
#define MSG_TYPE_DATA       0x10
#define MSG_TYPE_POLL       0x20
#define MSG_TYPE_ACK        0x30

// Inter-byte UART Time-Out Timer thresholds
#define TOT_RS232_BYTE_MS   250   // 9600 baud margin
#define TOT_RS485_BYTE_MS   180   // 19200 baud margin

// Frame Parser States
enum ParserStateM2 {
    STATE_WAIT_SOF = 0,
    STATE_IFACE_ID,
    STATE_DEV_ID,
    STATE_MSG_TYPE,
    STATE_SEQ_NUM,
    STATE_PAYLOAD_LEN,
    STATE_PAYLOAD,
    STATE_CRC,
    STATE_EOF
};

// Data Structure for a Validated Milestone 2 Frame
struct DataFrameM2 {
    uint8_t interface_id;              // 0x01 (RS232) or 0x02 (RS485)
    uint8_t device_id;                 // DEV_ID_1..4 or DEV_ID_GATEWAY
    uint8_t msg_type;                  // MSG_TYPE_DATA, MSG_TYPE_POLL, MSG_TYPE_ACK
    uint8_t seq_num;                   // Sequence number 0-255
    uint8_t payload_len;               // Length of data (0-96)
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Raw payload buffer
    uint8_t crc8;                      // Received CRC-8
};

// Interface Statistics Counters
struct ProtocolStatsM2 {
    uint32_t tx_packets;
    uint32_t rx_valid_packets;
    uint32_t rx_crc_errors;
    uint32_t rx_malformed_errors;
    uint32_t tot_byte_timeouts;
};

// Function: Calculate Dallas/Maxim CRC-8 (X^8 + X^5 + X^4 + 1 = 0x31)
inline uint8_t calculateCRC8_M2(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Class: Non-blocking Stream Frame Parser for Milestone 2
class FrameParserM2 {
private:
    ParserStateM2 state;
    DataFrameM2 current_frame;
    uint8_t payload_index;
    uint32_t last_byte_time;
    uint32_t byte_tot_ms;
    ProtocolStatsM2 stats;

public:
    FrameParserM2(uint32_t tot_ms = 200) {
        byte_tot_ms = tot_ms;
        reset();
        memset(&stats, 0, sizeof(ProtocolStatsM2));
    }

    void reset() {
        state = STATE_WAIT_SOF;
        payload_index = 0;
        last_byte_time = 0;
        memset(&current_frame, 0, sizeof(DataFrameM2));
    }

    ProtocolStatsM2 getStats() const { return stats; }
    void recordTx() { stats.tx_packets++; }

    void checkTimeout() {
        if (state != STATE_WAIT_SOF && (millis() - last_byte_time > byte_tot_ms)) {
            stats.tot_byte_timeouts++;
            reset();
        }
    }

    bool processByte(uint8_t byte, DataFrameM2 &out_frame) {
        uint32_t now = millis();

        if (state != STATE_WAIT_SOF && (now - last_byte_time > byte_tot_ms)) {
            stats.tot_byte_timeouts++;
            reset();
        }
        last_byte_time = now;

        switch (state) {
            case STATE_WAIT_SOF:
                if (byte == FRAME_SOF) {
                    reset();
                    last_byte_time = now;
                    state = STATE_IFACE_ID;
                }
                break;

            case STATE_IFACE_ID:
                if (byte == IFACE_RS232 || byte == IFACE_RS485) {
                    current_frame.interface_id = byte;
                    state = STATE_DEV_ID;
                } else {
                    stats.rx_malformed_errors++;
                    reset();
                }
                break;

            case STATE_DEV_ID:
                if (byte <= DEV_ID_4 || byte == DEV_ID_GATEWAY) {
                    current_frame.device_id = byte;
                    state = STATE_MSG_TYPE;
                } else {
                    stats.rx_malformed_errors++;
                    reset();
                }
                break;

            case STATE_MSG_TYPE:
                if (byte == MSG_TYPE_DATA || byte == MSG_TYPE_POLL || byte == MSG_TYPE_ACK) {
                    current_frame.msg_type = byte;
                    state = STATE_SEQ_NUM;
                } else {
                    stats.rx_malformed_errors++;
                    reset();
                }
                break;

            case STATE_SEQ_NUM:
                current_frame.seq_num = byte;
                state = STATE_PAYLOAD_LEN;
                break;

            case STATE_PAYLOAD_LEN:
                if (byte <= MAX_PAYLOAD_SIZE) {
                    current_frame.payload_len = byte;
                    payload_index = 0;
                    if (current_frame.payload_len == 0) {
                        state = STATE_CRC;
                    } else {
                        state = STATE_PAYLOAD;
                    }
                } else {
                    stats.rx_malformed_errors++;
                    reset();
                }
                break;

            case STATE_PAYLOAD:
                current_frame.payload[payload_index++] = byte;
                if (payload_index >= current_frame.payload_len) {
                    state = STATE_CRC;
                }
                break;

            case STATE_CRC:
                current_frame.crc8 = byte;
                state = STATE_EOF;
                break;

            case STATE_EOF:
                if (byte == FRAME_EOF) {
                    uint8_t crc_buffer[MAX_PAYLOAD_SIZE + 5];
                    crc_buffer[0] = current_frame.interface_id;
                    crc_buffer[1] = current_frame.device_id;
                    crc_buffer[2] = current_frame.msg_type;
                    crc_buffer[3] = current_frame.seq_num;
                    crc_buffer[4] = current_frame.payload_len;
                    for (uint8_t i = 0; i < current_frame.payload_len; i++) {
                        crc_buffer[5 + i] = current_frame.payload[i];
                    }

                    uint8_t expected_crc = calculateCRC8_M2(crc_buffer, 5 + current_frame.payload_len);

                    if (expected_crc == current_frame.crc8) {
                        stats.rx_valid_packets++;
                        out_frame = current_frame;
                        reset();
                        return true;
                    } else {
                        stats.rx_crc_errors++;
                        reset();
                    }
                } else {
                    stats.rx_malformed_errors++;
                    reset();
                }
                break;
        }

        return false;
    }
};

// Function: Build and transmit a serialized framed packet over a Stream
inline void sendDataFrameM2(Stream &port, uint8_t iface_id, uint8_t dev_id, uint8_t msg_type,
                            uint8_t seq_num, const char *payload_str, bool corrupt_crc = false) {
    uint8_t len = 0;
    if (payload_str != nullptr) {
        len = strlen(payload_str);
        if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;
    }

    uint8_t crc_buffer[MAX_PAYLOAD_SIZE + 5];
    crc_buffer[0] = iface_id;
    crc_buffer[1] = dev_id;
    crc_buffer[2] = msg_type;
    crc_buffer[3] = seq_num;
    crc_buffer[4] = len;
    for (uint8_t i = 0; i < len; i++) {
        crc_buffer[5 + i] = (uint8_t)payload_str[i];
    }

    uint8_t crc = calculateCRC8_M2(crc_buffer, 5 + len);
    if (corrupt_crc) {
        crc ^= 0xFF; // Invert CRC to simulate checksum failure
    }

    port.write(FRAME_SOF);
    port.write(iface_id);
    port.write(dev_id);
    port.write(msg_type);
    port.write(seq_num);
    port.write(len);
    for (uint8_t i = 0; i < len; i++) {
        port.write((uint8_t)payload_str[i]);
    }
    port.write(crc);
    port.write(FRAME_EOF);
}

// ==============================================================================
// 2. HARDWARE PIN & BAUD CONFIGURATION
// ==============================================================================
#define RS485_DE_RE_PIN       2        // Pin 2 connects to DE & RE via Level Shifter
#define RS232_BAUD_RATE       9600     // Serial1: Pins 18 (TX1), 19 (RX1)
#define RS485_BAUD_RATE       19200    // Serial2: Pins 16 (TX2), 17 (RX2)
#define DEBUG_BAUD_RATE       115200   // USB Serial Monitor

// 20x4 I2C LCD Configuration
#define LCD_I2C_ADDRESS       0x27
#define LCD_COLUMNS           20
#define LCD_ROWS              4

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Stream Parsers
FrameParserM2 rs232Parser(TOT_RS232_BYTE_MS);
FrameParserM2 rs485Parser(TOT_RS485_BYTE_MS);

// Sequence Counters
uint8_t dev1_seq = 1;
uint8_t dev2_seq = 1;
uint8_t dev3_seq = 1;
uint8_t dev4_seq = 1;

// Scheduling Timers
unsigned long lastDev1Tx = 0;
unsigned long lastDev2Tx = 0;
unsigned long lastLcdUpdate = 0;

// Forward Declarations
void d1Data();
void d2Data();
void d3Data();
void d4Data();
void printMenu();
void handleSerialCLI();
void updateLcd();

// ==============================================================================
// 3. MODULAR DEVICE TELEMETRY TRANSMISSION FUNCTIONS
// ==============================================================================

/**
 * Device 1 [d1Data]: Continuous transmission over RS-232
 * Generates and transmits temperature and barometric pressure data.
 */
void d1Data() {
    static float simTemp  = 25.4;
    static float simPress = 1013.2;

    simTemp = 25.0 + (float)(dev1_seq % 30) * 0.2;
    simPress = 1010.0 + (float)(dev1_seq % 15) * 0.5;

    int tWhole = (int)simTemp;
    int tFrac  = abs((int)(simTemp * 10.0f) % 10);
    int pWhole = (int)simPress;

    char payload[MAX_PAYLOAD_SIZE];
    if (corruptNextDev1Value) {
        corruptNextDev1Value = false;
        snprintf(payload, sizeof(payload), "D1:T=?C,P=%dhPa", pWhole);
        Serial.println(F("\n! [TEST] Injected missing numerical value 'T=?C' on Device 1!"));
        Serial.println(F("  Watch ESP32-S3 Serial Monitor: Averages past 2 frames and tags with 'T=xx.xC[R]'!"));
    } else {
        snprintf(payload, sizeof(payload), "D1:T=%d.%dC,P=%dhPa", tWhole, tFrac, pWhole);
    }

    sendDataFrameM2(Serial1, IFACE_RS232, DEV_ID_1, MSG_TYPE_DATA, dev1_seq++, payload, false);
    rs232Parser.recordTx();
}

/**
 * Device 2 [d2Data]: Periodic transmission over RS-232
 * Transmits a large comprehensive payload (twice larger than D1).
 */
void d2Data() {
    static float simBattV = 12.55;
    static float simCurrA = 2.15;

    simBattV = 12.40 + (float)(dev2_seq % 20) * 0.02;
    simCurrA = 1.80  + (float)(dev2_seq % 10) * 0.10;

    int vWhole = (int)simBattV;
    int vFrac  = abs((int)(simBattV * 100.0f) % 100);
    int iWhole = (int)simCurrA;
    int iFrac  = abs((int)(simCurrA * 100.0f) % 100);
    float pwrVal = simBattV * simCurrA;
    int pwrWhole = (int)pwrVal;
    int pwrFrac  = abs((int)(pwrVal * 10.0f) % 10);

    char payload[MAX_PAYLOAD_SIZE];

    // If corrupted token test is active, transmit P_R (with W lost) and missing value ?W
    if (corruptNextDev2Token) {
        corruptNextDev2Token = false;
        snprintf(payload, sizeof(payload),
                 "D2_LARGE:V=%d.%02dV,I=%d.%02dA,P_R=?W,SOC=94%%,CAP=120Ah,ST=RUN_OK",
                 vWhole, vFrac, iWhole, iFrac);
        Serial.println(F("\n! [TEST] Injected corrupted token 'P_R' (W missing) & missing value '?W' on Dev 2!"));
        Serial.println(F("  Watch ESP32-S3 Serial Monitor: Restores 'PWR[R]' and calculates power with '[R]'!"));
    } else {
        snprintf(payload, sizeof(payload),
                 "D2_LARGE:V=%d.%02dV,I=%d.%02dA,PWR=%d.%dW,SOC=94%%,CAP=120Ah,ST=RUN_OK",
                 vWhole, vFrac, iWhole, iFrac, pwrWhole, pwrFrac);
    }

    sendDataFrameM2(Serial1, IFACE_RS232, DEV_ID_2, MSG_TYPE_DATA, dev2_seq++, payload, false);
    rs232Parser.recordTx();

    Serial.print(F(">>> [RS-232 TX] Dev 2 Large Data Sent (Seq:#"));
    Serial.print(dev2_seq - 1);
    Serial.print(F(", Len:"));
    Serial.print(strlen(payload));
    Serial.print(F("B - 2x D1): "));
    Serial.println(payload);
}

/**
 * Device 3 [d3Data]: Polled slave response over RS-485
 * Triggered on demand when polled by Gateway (Key '3' or on any Fault/Timeout).
 */
void d3Data() {
    static uint16_t simRpm = 1750;
    static uint8_t simLoad = 45;

    simRpm = 1700 + (dev3_seq * 15) % 400;
    simLoad = 40 + (dev3_seq % 30);

    char payload[MAX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload), "D3:RPM=%u,LD=%u%%", simRpm, simLoad);

    delayMicroseconds(250);
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(100);

    sendDataFrameM2(Serial2, IFACE_RS485, DEV_ID_3, MSG_TYPE_DATA, dev3_seq++, payload, false);
    Serial2.flush();
    delayMicroseconds(100);

    digitalWrite(RS485_DE_RE_PIN, LOW);
    rs485Parser.recordTx();

    Serial.print(F(">>> [RS-485 RESP] Dev 3 answered Poll (Seq:#"));
    Serial.print(dev3_seq - 1);
    Serial.print(F(" Data: "));
    Serial.print(payload);
    Serial.println(F(")"));
}

/**
 * Device 4 [d4Data]: Polled slave response over RS-485
 * Triggered periodically every 10 seconds by Gateway or on demand with Key '4'.
 */
void d4Data() {
    static float simFlow  = 3.42;
    static uint8_t simTank = 72;

    simFlow = 3.0 + (float)(dev4_seq % 20) * 0.08;
    simTank = 70 + (dev4_seq % 25);

    int flwWhole = (int)simFlow;
    int flwFrac  = abs((int)(simFlow * 100.0f) % 100);

    char payload[MAX_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload), "D4:FLW=%d.%02dL/m,TK=%u%%", flwWhole, flwFrac, simTank);

    delayMicroseconds(250);
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(100);

    sendDataFrameM2(Serial2, IFACE_RS485, DEV_ID_4, MSG_TYPE_DATA, dev4_seq++, payload, false);
    Serial2.flush();
    delayMicroseconds(100);

    digitalWrite(RS485_DE_RE_PIN, LOW);
    rs485Parser.recordTx();

    Serial.print(F(">>> [RS-485 RESP] Dev 4 answered Poll (Seq:#"));
    Serial.print(dev4_seq - 1);
    Serial.print(F(" Data: "));
    Serial.print(payload);
    Serial.println(F(")"));
}

// ==============================================================================
// 4. MAIN SETUP & LOOP
// ==============================================================================
void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    delay(500);

    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    Serial1.begin(RS232_BAUD_RATE);
    Serial2.begin(RS485_BAUD_RATE);

    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("[MEGA] 4-DEV EMULATR"));
    lcd.setCursor(0, 1);
    lcd.print(F("D1:#001     D2:#001 "));
    lcd.setCursor(0, 2);
    lcd.print(F("D3:#001     D4:#001 "));
    lcd.setCursor(0, 3);
    lcd.print(F("STATUS: READY (OK)  "));

    Serial.println(F("\n===================================================================="));
    Serial.println(F("     ARDUINO MEGA 2560 - 4-DEVICE MODULAR FIELD SIMULATOR (M2)      "));
    Serial.println(F("===================================================================="));
    Serial.println(F("Keyboard Device Commands:"));
    Serial.println(F("  • Enter '1' : Pause / Resume Device 1 (Isolates D1 TOT & Auto-Recovery)"));
    Serial.println(F("  • Enter '2' : Pause / Resume Device 2 (Isolates D2 TOT & Auto-Recovery)"));
    Serial.println(F("  • Enter '3' : Pause / Resume Device 3 (Isolates D3 Poll TOT & Auto-Recovery)"));
    Serial.println(F("  • Enter '4' : Pause / Resume Device 4 (Isolates D4 Poll TOT & Auto-Recovery)"));
    Serial.println(F("Data Recovery Tests:"));
    Serial.println(F("  • Enter 'G' : Force Sequence Gap (+2) on all devices (Tests Average Recovery with [R])"));
    Serial.println(F("  • Enter 'C' : Corrupt Token 'P_R=?W' on Dev 2 (Tests 'PWR[R]' & Numerical '[R]')"));
    Serial.println(F("  • Enter 'V' : Inject Missing Value 'T=?C' on Dev 1 (Tests Dev 1 [R] Average)"));
    Serial.println(F("Dynamic Speed Keys: [S]low (2s), [M]edium (1s), [F]ast (300ms), [+], [-]"));
    printMenu();

    unsigned long now = millis();
    lastDev1Tx = now;
    lastDev2Tx = now;
}

void loop() {
    unsigned long now = millis();

    // 0. Process Serial Diagnostic Commands
    handleSerialCLI();

    // 1. Device 1: RS-232 Continuous Transmission
    if (!dev1Paused && (now - lastDev1Tx >= dev1IntervalMs)) {
        lastDev1Tx = now;
        d1Data();
    }

    // 2. Device 2: RS-232 Periodic Transmission every 4.0s (Data twice larger than D1)
    if (!dev2Paused && (now - lastDev2Tx >= dev2IntervalMs)) {
        lastDev2Tx = now;
        d2Data();
    }

    // 3. Devices 3 & 4: RS-485 Slave Listener & Responder
    while (Serial2.available() > 0) {
        uint8_t b = Serial2.read();
        DataFrameM2 rx_frame;
        if (rs485Parser.processByte(b, rx_frame)) {
            if (rx_frame.msg_type == MSG_TYPE_POLL) {
                if (rx_frame.device_id == DEV_ID_3) {
                    if (!dev3Paused) {
                        d3Data();
                    } else {
                        Serial.println(F("! [TEST] Device 3 is PAUSED -> ignoring poll request (DEV 3 TOT)!"));
                    }
                } else if (rx_frame.device_id == DEV_ID_4) {
                    if (!dev4Paused) {
                        d4Data();
                    } else {
                        Serial.println(F("! [TEST] Device 4 is PAUSED -> ignoring poll request (DEV 4 TOT)!"));
                    }
                }
            }
        }
    }
    rs485Parser.checkTimeout();

    // 4. Ingest ACKs from Gateway over RS-232
    while (Serial1.available() > 0) {
        uint8_t b = Serial1.read();
        DataFrameM2 rx_frame;
        rs232Parser.processByte(b, rx_frame);
    }
    rs232Parser.checkTimeout();

    // 5. Update 20x4 LCD (No cont, pol, per - sequence numbers front and center!)
    if (now - lastLcdUpdate >= LCD_INTERVAL_MS) {
        lastLcdUpdate = now;
        updateLcd();
    }
}

// ==============================================================================
// 5. SERIAL DIAGNOSTIC INTERACTION & LCD DISPLAY
// ==============================================================================
void handleSerialCLI() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case '1':
                dev1Paused = !dev1Paused;
                if (dev1Paused) {
                    Serial.println(F("\n! [TEST] Device 1 PAUSED!"));
                    Serial.println(F("  Dev 2 continues streaming! Watch ESP32-S3 show ONLY Dev 1 TOT (>2500ms)!"));
                } else {
                    Serial.println(F("\n* [TEST] Device 1 RESUMED!"));
                    Serial.println(F("  Watch ESP32-S3: LCD will flash [* AUTO-RECOVERY * DEV 1]!"));
                }
                break;

            case '2':
                dev2Paused = !dev2Paused;
                if (dev2Paused) {
                    Serial.println(F("\n! [TEST] Device 2 PAUSED!"));
                    Serial.println(F("  Dev 1 continues streaming! Watch ESP32-S3 show ONLY Dev 2 TOT (>8000ms)!"));
                } else {
                    Serial.println(F("\n* [TEST] Device 2 RESUMED!"));
                    Serial.println(F("  Watch ESP32-S3: LCD will flash [* AUTO-RECOVERY * DEV 2]!"));
                }
                break;

            case '3':
                dev3Paused = !dev3Paused;
                if (dev3Paused) {
                    Serial.println(F("\n! [TEST] Device 3 PAUSED! (Will ignore RS-485 polls -> Triggers DEV 3 TOT on Gateway)"));
                } else {
                    Serial.println(F("\n* [TEST] Device 3 RESUMED! (Will answer RS-485 polls -> Triggers AUTO-RECOVERY)"));
                }
                break;

            case '4':
                dev4Paused = !dev4Paused;
                if (dev4Paused) {
                    Serial.println(F("\n! [TEST] Device 4 PAUSED! (Will ignore 10s polls -> Triggers DEV 4 TOT on Gateway)"));
                } else {
                    Serial.println(F("\n* [TEST] Device 4 RESUMED! (Will answer 10s polls -> Triggers AUTO-RECOVERY)"));
                }
                break;

            case 'c':
            case 'C':
                corruptNextDev2Token = true;
                Serial.println(F("\n! [TEST] Next Dev 2 frame will have corrupted token 'P_R=?W' (missing W and value)!"));
                Serial.println(F("  Watch ESP32-S3 Serial Monitor: Restores 'PWR[R]' and numerical average with '[R]'!"));
                break;

            case 'v':
            case 'V':
                corruptNextDev1Value = true;
                Serial.println(F("\n! [TEST] Next Dev 1 frame will have missing numerical value 'T=?C'!"));
                Serial.println(F("  Watch ESP32-S3 Serial Monitor: Averages past 2 frames and tags with 'T=xx.xC[R]'!"));
                break;

            case 's':
            case 'S':
                dev1IntervalMs = 2000;
                Serial.println(F("\n>>> [SPEED: SLOW MODE (2000ms)] Dev 1 now streams once every 2 seconds (Easy to read!)"));
                break;

            case 'm':
            case 'M':
                dev1IntervalMs = 1000;
                Serial.println(F("\n>>> [SPEED: MEDIUM MODE (1000ms)] Dev 1 now streams once every 1 second (Default)"));
                break;

            case 'f':
            case 'F':
                dev1IntervalMs = 300;
                Serial.println(F("\n>>> [SPEED: FAST MODE (300ms)] Dev 1 now streams at high data rate (Stress test)"));
                break;

            case '+':
                if (dev1IntervalMs > 200) dev1IntervalMs -= 200;
                Serial.print(F("\n>>> [SPEED UP] Dev 1 interval decreased to: "));
                Serial.print(dev1IntervalMs);
                Serial.println(F("ms"));
                break;

            case '-':
                dev1IntervalMs += 500;
                Serial.print(F("\n>>> [SLOW DOWN] Dev 1 interval increased to: "));
                Serial.print(dev1IntervalMs);
                Serial.println(F("ms"));
                break;

            case 'g':
            case 'G':
                dev1_seq += 2;
                dev2_seq += 2;
                dev3_seq += 2;
                dev4_seq += 2;
                Serial.println(F("\n! [FAULT INJECT] Forced SEQUENCE GAP (+2) on all devices!"));
                Serial.println(F("  Watch ESP32-S3: Data Recovery Engine will detect gap and reconstruct missing packet for all devices!"));
                break;

            case '?':
            case 'h':
            case 'H':
                printMenu();
                break;
        }
    }
}

void printMenu() {
    Serial.println(F("\n----------------- ARDUINO MEGA COMMAND MENU -------------------"));
    Serial.println(F("  ISOLATED TOT & AUTO-RECOVERY TESTS:"));
    Serial.println(F("    Press [1] : Pause/Resume Device 1 (Shows isolated DEV 1 TOT & Auto-Recovery)"));
    Serial.println(F("    Press [2] : Pause/Resume Device 2 (Shows isolated DEV 2 TOT & Auto-Recovery)"));
    Serial.println(F("    Press [3] : Pause/Resume Device 3 (Shows isolated DEV 3 Poll TOT & Auto-Recovery)"));
    Serial.println(F("    Press [4] : Pause/Resume Device 4 (Shows isolated DEV 4 Poll TOT & Auto-Recovery)"));
    Serial.println(F("  DATA RECOVERY & FAULT INJECTION:"));
    Serial.println(F("    Press [G] : Force Sequence Gap (+2) on all devices -> Tests Average Recovery with [R]"));
    Serial.println(F("    Press [C] : Corrupt Token 'P_R=?W' on Dev 2 -> Tests 'PWR[R]' & Numerical '[R]'"));
    Serial.println(F("    Press [V] : Inject Missing Value 'T=?C' on Dev 1 -> Tests Dev 1 [R] Average"));
    Serial.println(F("  DYNAMIC SPEED & RATE CONTROLS:"));
    Serial.println(F("    Press [S] : SLOW Mode (2000ms - extended interval)"));
    Serial.println(F("    Press [M] : MEDIUM Mode (1000ms - standard readable default)"));
    Serial.println(F("    Press [F] : FAST Mode (300ms - high-speed stress test)"));
    Serial.println(F("    Press [+] : Speed up Dev 1 by 200ms"));
    Serial.println(F("    Press [-] : Slow down Dev 1 by 500ms"));
    Serial.println(F("  Press [?]   : Re-print this menu"));
    Serial.println(F("---------------------------------------------------------------\n"));
}

void updateLcd() {
    char r0[21], r1[21], r2[21], r3[21];
    snprintf(r0, sizeof(r0), "[MEGA] SPEED:%4lums", dev1IntervalMs);

    if (dev1Paused) snprintf(r1, sizeof(r1), "D1:PAUSED   D2:#%-5u", dev2_seq);
    else if (dev2Paused) snprintf(r1, sizeof(r1), "D1:#%-5u   D2:PAUSED", dev1_seq);
    else snprintf(r1, sizeof(r1), "D1:#%-5u   D2:#%-5u", dev1_seq, dev2_seq);

    if (dev3Paused) snprintf(r2, sizeof(r2), "D3:PAUSED   D4:#%-5u", dev4_seq);
    else if (dev4Paused) snprintf(r2, sizeof(r2), "D3:#%-5u   D4:PAUSED", dev3_seq);
    else snprintf(r2, sizeof(r2), "D3:#%-5u   D4:#%-5u", dev3_seq, dev4_seq);

    if (dev1Paused) snprintf(r3, sizeof(r3), "! DEV 1 PAUSED (TOT)! ");
    else if (dev2Paused) snprintf(r3, sizeof(r3), "! DEV 2 PAUSED (TOT)! ");
    else if (dev3Paused) snprintf(r3, sizeof(r3), "! DEV 3 PAUSED (TOT)! ");
    else if (dev4Paused) snprintf(r3, sizeof(r3), "! DEV 4 PAUSED (TOT)! ");
    else snprintf(r3, sizeof(r3), "STATUS: ALL RUNNING  ");

    lcd.setCursor(0, 0); lcd.print(r0);
    lcd.setCursor(0, 1); lcd.print(r1);
    lcd.setCursor(0, 2); lcd.print(r2);
    lcd.setCursor(0, 3); lcd.print(r3);
}
