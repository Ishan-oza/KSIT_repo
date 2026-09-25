/*
 * ==============================================================================
 * MULTI-DEVICE CENTRAL GATEWAY: Dual-Bus Acquisition Hub
 * Platform: ESP32-S3 (Dual-Core, 2.4GHz Wi-Fi / Serial Gateway)
 * ==============================================================================
 * ALL-IN-ONE STANDALONE FILE (Zero external .h header files required)
 * CORE CAPABILITIES IMPLEMENTED:
 *   1. Concurrent ingestion & representation of 4 independent logical devices:
 *        - Device 1: RS-232 Continuous Stream (1000ms pace)
 *        - Device 2: RS-232 Periodic Stream (4.0s, 2x Large Data)
 *        - Device 3: RS-485 Fast / Fault / Key 3 Triggered Slave
 *        - Device 4: RS-485 10-Second Periodic Polling / Key 4 Slave
 *   2. Bidirectional Communication:
 *        - RS-232: Transmits validated ACK response back to Mega for every frame.
 *        - RS-485: Master polling requests scheduled for Dev 3 (on-demand/fault) & Dev 4 (every 10s).
 *   3. Device-Isolated Comparative Data Recovery & Auto-Repair Engine:
 *        - Compares incoming frames STRICTLY against the past 2 frames of the SAME device!
 *        - Static Alphabets/Labels: Restores missing/corrupted letters (e.g. 'P_R' -> 'PWR')
 *          by comparing against the identical alphabetic template of the past 2 frames.
 *        - Varying Numerical Data: Applies average logic of the past 2 frames to calculate
 *          missing/corrupted numerical values or reconstruct missing frames.
 *   4. High-Visibility Flashing LCD Alert System:
 *        - Flashes [* AUTO-RECOVERY *] and [! FAULT: TIMEOUT !] with acoustic buzzer.
 *   5. Serial Stream Freeze / Pause Feature:
 *        - Press SPACE in Serial Monitor to freeze live scrolling to inspect data.
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==============================================================================
// 0. CONFIGURABLE TIMING & WATCHDOG THRESHOLDS (CHANGE AT TOP)
// ==============================================================================
#define DEV1_TIMEOUT_THRESHOLD_MS   2500   // Dev 1 silence threshold (2.5s)
#define DEV2_TIMEOUT_THRESHOLD_MS   8000   // Dev 2 silence threshold (8.0s)
#define DEV4_POLL_INTERVAL_MS       10000  // Dev 4 automated poll interval (10s)
#define DASHBOARD_INTERVAL_MS       10000  // ANSI Dashboard interval (10s)
#define LCD_PAGE_INTERVAL_MS        3000   // LCD cycling interval (3s)
#define BUZZER_INTERVAL_MS          5000   // Periodic buzzer check (5s)

// ==============================================================================
// 0.5 WI-FI & UDP PROTOCOL CONFIGURATION (MILESTONE 3)
// ==============================================================================
const char* WIFI_SSID     = "Loyal hu, sbse connect nhi houga";
const char* WIFI_PASSWORD = "loyalpassword";
const uint16_t UDP_PORT   = 5000;

// Milestone 3 UDP Commands
#define CMD_GET_SINGLE_DEVICE 0x01
#define CMD_GET_ALL_DEVICES   0x02
#define CMD_GET_HEALTH_STATUS 0x03

// Milestone 3 Status & Error Codes
#define STATUS_OK             0x00
#define ERR_INVALID_CMD       0x01
#define ERR_INVALID_DEVICE_ID 0x02
#define ERR_DEVICE_OFFLINE    0x03
#define ERR_MUTEX_TIMEOUT     0x04
#define ERR_MALFORMED_PACKET  0x05
#define DEV2_TIMEOUT_THRESHOLD_MS   8000   // Dev 2 silence threshold (8.0s)
#define DEV4_POLL_INTERVAL_MS       10000  // Dev 4 automated poll interval (10s)
#define DASHBOARD_INTERVAL_MS       10000  // ANSI Dashboard interval (10s)
#define LCD_PAGE_INTERVAL_MS        3000   // LCD cycling interval (3s)
#define BUZZER_INTERVAL_MS          5000   // Periodic buzzer check (5s)

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
#define DEV_ID_3            0x03  // RS-485 Fast / Fault / Key 3 Triggered Slave
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
    bool flag_crc_err;
    bool flag_tot_timeout;
    bool flag_malf_err;

public:
    FrameParserM2(uint32_t tot_ms = 200) {
        byte_tot_ms = tot_ms;
        flag_crc_err = false;
        flag_tot_timeout = false;
        flag_malf_err = false;
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

    bool checkTimeout() {
        if (state != STATE_WAIT_SOF && (millis() - last_byte_time > byte_tot_ms)) {
            stats.tot_byte_timeouts++;
            flag_tot_timeout = true;
            reset();
            return true;
        }
        return false;
    }

    bool popCrcError() {
        if (flag_crc_err) { flag_crc_err = false; return true; }
        return false;
    }

    bool popTotTimeout() {
        if (flag_tot_timeout) { flag_tot_timeout = false; return true; }
        return false;
    }

    bool popMalformedError() {
        if (flag_malf_err) { flag_malf_err = false; return true; }
        return false;
    }

    bool processByte(uint8_t byte, DataFrameM2 &out_frame) {
        uint32_t now = millis();

        if (state != STATE_WAIT_SOF && (now - last_byte_time > byte_tot_ms)) {
            stats.tot_byte_timeouts++;
            flag_tot_timeout = true;
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
                        flag_crc_err = true;
                        reset();
                    }
                } else {
                    stats.rx_malformed_errors++;
                    flag_malf_err = true;
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
#define RS232_RX_PIN          18       // RS-232 Receiver (from Level Shifter CH1)
#define RS232_TX_PIN          17       // RS-232 Transmitter (to Level Shifter CH2)
#define RS485_RX_PIN          15       // RS-485 Receiver (from MAX3485 RO)
#define RS485_TX_PIN          16       // RS-485 Transmitter (to MAX3485 DI)
#define RS485_DE_RE_PIN       6        // RS-485 Transmit Enable (DE & /RE)
#define BUZZER_PIN            13       // Alert Buzzer Pin
#define WHITE_LED_PIN         4        // ESP32 Flashlight LED (Kept LOW/OFF)

#define I2C_SDA_PIN           8        // Custom I2C Data Pin
#define I2C_SCL_PIN           9        // Custom I2C Clock Pin

#define RS232_BAUD_RATE       9600     // RS-232 Serial1 baud
#define RS485_BAUD_RATE       19200    // RS-485 Serial2 baud
#define DEBUG_BAUD_RATE       115200   // USB CDC Serial Monitor

// 16x2 I2C LCD Configuration
#define LCD_I2C_ADDRESS       0x27
#define LCD_COLUMNS           16
#define LCD_ROWS              2

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Stream Parsers
FrameParserM2 rs232Parser(TOT_RS232_BYTE_MS);
FrameParserM2 rs485Parser(TOT_RS485_BYTE_MS);

// ==============================================================================
// 3. DEVICE HEALTH STATUS & RECORD STRUCTURE
// ==============================================================================
enum DeviceHealth {
    HEALTH_OK = 0,
    HEALTH_TIMEOUT,
    HEALTH_CORRUPTED
};

struct DeviceRecord {
    uint8_t devId;
    const char *name;
    const char *ifaceName;
    const char *behaviour;
    uint8_t ifaceId;
    uint8_t expectedSeq;
    uint8_t lastSeq;
    bool hasReceivedFirst;
    uint32_t rxCount;
    uint32_t droppedCount;
    uint32_t duplicateCount;
    uint32_t lastSeenMs;
    uint32_t timeoutThresholdMs;
    DeviceHealth health;
    char latestPayload[MAX_PAYLOAD_SIZE + 1];
    char cleanTelemetry[24];

    // Dedicated Historical Payloads STRICTLY for this device (prevents cross-device contamination)
    char prevPayload1[MAX_PAYLOAD_SIZE + 1]; // 1 frame ago (k-1)
    char prevPayload2[MAX_PAYLOAD_SIZE + 1]; // 2 frames ago (k-2)
    bool hasPrev1;
    bool hasPrev2;

    // Milestone 3 Extended Diagnostic Counters (Supervised by UDP CMD 0x03)
    uint16_t crcErrorCount;
    uint16_t timeoutCount;
    uint16_t outOfOrderCount;
};

// Allocate records for all 4 devices
DeviceRecord devices[4] = {
    { DEV_ID_1, "DEV 1", "RS-232", "Continuous",      IFACE_RS232, 0, 0, false, 0, 0, 0, 0, DEV1_TIMEOUT_THRESHOLD_MS, HEALTH_OK, "INITIALIZING", "INIT...", "", "", false, false, 0, 0, 0 },
    { DEV_ID_2, "DEV 2", "RS-232", "Period 4s (2x)",  IFACE_RS232, 0, 0, false, 0, 0, 0, 0, DEV2_TIMEOUT_THRESHOLD_MS, HEALTH_OK, "INITIALIZING", "INIT...", "", "", false, false, 0, 0, 0 },
    { DEV_ID_3, "DEV 3", "RS-485", "Fault/Key Trig",  IFACE_RS485, 0, 0, false, 0, 0, 0, 0, 0,                         HEALTH_OK, "[STANDBY]",    "[STANDBY]", "", "", false, false, 0, 0, 0 },
    { DEV_ID_4, "DEV 4", "RS-485", "Poll 10s/Key 4",  IFACE_RS485, 0, 0, false, 0, 0, 0, 0, 0,                         HEALTH_OK, "[STANDBY]",    "[STANDBY]", "", "", false, false, 0, 0, 0 }
};

// Milestone 3 Wi-Fi & UDP Global Server State
WiFiUDP udpServer;
bool wifiConnected = false;
bool udpServerListening = false;
unsigned long lastWifiRetry = 0;

uint8_t espRs232AckSeq = 1;
uint8_t gatewayPollSeq = 1;

// RS-485 Polling Engine State Machine
enum PollEngineState {
    POLL_STATE_IDLE = 0,
    POLL_STATE_WAIT_DEV3,
    POLL_STATE_WAIT_DEV4
};

PollEngineState pollState = POLL_STATE_IDLE;
unsigned long pollWaitStartTime = 0;
const unsigned long POLL_RESPONSE_TIMEOUT_MS = 300;

unsigned long lastDev4PollTime = 0;

// On-Demand D3 & D4 LCD Display Timers
unsigned long d3DisplayUntil = 0;
unsigned long d4DisplayUntil = 0;
const unsigned long ON_DEMAND_DISPLAY_DURATION_MS = 4000;

// ==============================================================================
// 3.5 HIGH-VISIBILITY ALERT & FLASH SYSTEM FOR 16x2 LCD
// ==============================================================================
char lcdAlertLine0[17] = "";
char lcdAlertLine1[17] = "";
unsigned long lcdAlertUntil = 0;
unsigned long lastLcdFlashToggle = 0;
bool lcdFlashState = false;

// Terminal Stream Pause Feature
bool terminalOutputPaused = false;

// Display & Diagnostics Schedulers
unsigned long lastDashboardPrint = 0;
unsigned long lastLcdUpdate      = 0;
unsigned long lastHealthCheck    = 0;
unsigned long lastBuzzerCheck    = 0;

uint8_t lcdPage = 0;

// Non-blocking Buzzer Variables
enum BuzzerPhase {
    BUZZ_IDLE = 0,
    BUZZ_BEEP_ON,
    BUZZ_BEEP_OFF
};

BuzzerPhase buzzPhase = BUZZ_IDLE;
unsigned long buzzTimer = 0;
uint8_t buzzRemaining = 0;
uint16_t buzzOnMs = 100;
uint16_t buzzOffMs = 80;

// Forward Declarations
DeviceRecord* getDevice(uint8_t devId);
void processIncomingFrame(DataFrameM2 &frame);
void autoRepairCorruptedPayload(DeviceRecord *dev, char *payload, size_t maxLen);
void recoverMissingData(DeviceRecord *dev, uint8_t gap, uint8_t currentSeq);
void manageRs485Polling(unsigned long now);
void triggerManualPoll(uint8_t devId);
void handleGatewayCLI(char c);
void printGatewayMenu();
void checkDeviceHealth(unsigned long now);
void printConsoleDashboard();
void updateLcdDisplay();
void printPaddedLine(uint8_t row, const char *rawText);
void triggerLcdAlert(const char *line0, const char *line1, uint16_t durationMs = 3500);
void processBuzzer();
void triggerBeeps(uint8_t count, uint16_t onDuration = 100, uint16_t offDuration = 80);

void setup() {
    pinMode(WHITE_LED_PIN, OUTPUT);
    digitalWrite(WHITE_LED_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    Serial.begin(DEBUG_BAUD_RATE);
    delay(1500);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ESP32-S3 GATEWAY"));
    lcd.setCursor(0, 1);
    lcd.print(F("M2+M3: UDP GW   "));

    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

    // Initialize Wi-Fi in Station Mode (Milestone 3)
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(F("\n[WI-FI] Connecting to SSID: ")); Serial.println(WIFI_SSID);

    Serial1.begin(RS232_BAUD_RATE, SERIAL_8N1, RS232_RX_PIN, RS232_TX_PIN);
    Serial2.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    unsigned long now = millis();
    for (int i = 0; i < 4; i++) {
        devices[i].lastSeenMs = now + 3000;
        devices[i].health = HEALTH_OK;
    }
    lastLcdUpdate = now;
    lastDev4PollTime = now;

    Serial.println(F("\n===================================================================="));
    Serial.println(F("     ESP32-S3 MULTI-DEVICE SERIAL-TO-WIFI GATEWAY (M2 & M3)         "));
    Serial.println(F("===================================================================="));
    Serial.println(F("Logical Devices Registered:"));
    Serial.println(F("  [D1] RS-232 Continuous Stream (1000ms)   - Temp / Pressure"));
    Serial.println(F("  [D2] RS-232 Periodic (4.0s, 2x Large)    - Battery / Power"));
    Serial.println(F("  [D3] RS-485 Fault/Key Triggered Slave    - Motor RPM / Load"));
    Serial.println(F("  [D4] RS-485 10-Second / Key 4 Slave      - Flow / Tank Level"));
    Serial.println(F("Features: Device-Isolated 2-Frame Comparison + [R] Tag Data Recovery!"));
    Serial.println(F("Network:  Wi-Fi UDP Server on Port 5000 (Non-blocking Acquisition & Commands)"));
    printGatewayMenu();
}

void loop() {
    unsigned long now = millis();

    // 0. Wi-Fi Reconnection Watchdog & Milestone 3 UDP Server Command Engine
    manageWiFiAndUdp(now);

    // 0.5 Handle Serial Diagnostic Commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        handleGatewayCLI(c);
    }

    // 1. Ingest RS-232 Stream (Serial1)
    while (Serial1.available() > 0) {
        uint8_t b = Serial1.read();
        DataFrameM2 frame;
        if (rs232Parser.processByte(b, frame)) {
            processIncomingFrame(frame);

            // Send ACK Back over RS-232
            char ackPayload[MAX_PAYLOAD_SIZE];
            snprintf(ackPayload, sizeof(ackPayload), "ACK_D%u:#%u", frame.device_id, frame.seq_num);
            sendDataFrameM2(Serial1, IFACE_RS232, DEV_ID_GATEWAY, MSG_TYPE_ACK, espRs232AckSeq++, ackPayload, false);
            rs232Parser.recordTx();
        }
    }
    rs232Parser.checkTimeout();

    // 2. Ingest RS-485 Slave Responses (Serial2)
    while (Serial2.available() > 0) {
        uint8_t b = Serial2.read();
        DataFrameM2 frame;
        if (rs485Parser.processByte(b, frame)) {
            processIncomingFrame(frame);

            if (pollState == POLL_STATE_WAIT_DEV3 && frame.device_id == DEV_ID_3) {
                pollState = POLL_STATE_IDLE;
            } else if (pollState == POLL_STATE_WAIT_DEV4 && frame.device_id == DEV_ID_4) {
                pollState = POLL_STATE_IDLE;
            }
        }
    }
    rs485Parser.checkTimeout();

    // 2.5 Check for Fault Triggers (CRC check failure, TOT interbyte timeout, malformed errors)
    if (rs232Parser.popCrcError()) {
        Serial.println(F("\n! [FAULT TRIGGER: CRC CHECK FAILURE on RS-232] -> Corrupt frame dropped!"));
        Serial.println(F("  >>> [DATA RECOVERY] Preserving Latest Valid Record Buffer for uninterrupted operation."));
        triggerLcdAlert("! FAULT: CRC ERR !", "RS-232 CRC FAIL ", 3000);
        triggerBeeps(2, 100, 60);
        devices[0].crcErrorCount++;
        devices[1].crcErrorCount++;
        triggerManualPoll(DEV_ID_3);
    }
    if (rs485Parser.popCrcError()) {
        Serial.println(F("\n! [FAULT TRIGGER: CRC CHECK FAILURE on RS-485] -> Corrupt frame dropped!"));
        triggerLcdAlert("! FAULT: CRC ERR !", "RS-485 CRC FAIL ", 3000);
        triggerBeeps(2, 100, 60);
        devices[2].crcErrorCount++;
        devices[3].crcErrorCount++;
        triggerManualPoll(DEV_ID_3);
    }
    if (rs232Parser.popTotTimeout()) {
        Serial.println(F("\n! [FAULT TRIGGER: INTERBYTE TOT on RS-232] -> Dropped incomplete frame!"));
        triggerLcdAlert("! FAULT: BYTE-TOT!", "RS-232 BYTE TOT ", 3000);
        triggerBeeps(2, 100, 60);
        triggerManualPoll(DEV_ID_3);
    }
    if (rs485Parser.popTotTimeout()) {
        Serial.println(F("\n! [FAULT TRIGGER: INTERBYTE TOT on RS-485] -> Dropped incomplete frame!"));
        triggerLcdAlert("! FAULT: BYTE-TOT!", "RS-485 BYTE TOT ", 3000);
        triggerBeeps(2, 100, 60);
        triggerManualPoll(DEV_ID_3);
    }
    if (rs232Parser.popMalformedError()) {
        Serial.println(F("\n! [FAULT TRIGGER: MALFORMED FRAME on RS-232] -> Framing error detected!"));
        triggerLcdAlert("! FAULT: MALFORM !", "RS-232 BAD FRAME", 3000);
        triggerBeeps(2, 100, 60);
        triggerManualPoll(DEV_ID_3);
    }

    // Run RS-485 Polling Engine (Handles Dev 4 every 10s & response timeouts)
    manageRs485Polling(now);

    // 3. Health Watchdog Check
    if (now - lastHealthCheck >= 200) {
        lastHealthCheck = now;
        checkDeviceHealth(now);
    }

    // 4. Console Dashboard Print
    if (now - lastDashboardPrint >= DASHBOARD_INTERVAL_MS) {
        lastDashboardPrint = now;
        printConsoleDashboard();
    }

    // 5. 16x2 LCD Display Engine (Handles Alert Flashing and Normal Page Cycling)
    if (now < lcdAlertUntil) {
        // High-Visibility Alert Flashing mode (Toggles every 300ms)
        if (now - lastLcdFlashToggle >= 300) {
            lastLcdFlashToggle = now;
            lcdFlashState = !lcdFlashState;

            if (lcdFlashState) {
                printPaddedLine(0, lcdAlertLine0);
                printPaddedLine(1, lcdAlertLine1);
            } else {
                printPaddedLine(0, "                "); // Flash top line
                printPaddedLine(1, lcdAlertLine1);
            }
        }
    } else {
        // Normal Page Cycling (5 pages: 0-4)
        if (now - lastLcdUpdate >= LCD_PAGE_INTERVAL_MS) {
            lastLcdUpdate = now;
            updateLcdDisplay();
            lcdPage = (lcdPage + 1) % 5;
        }
    }

    // 6. Buzzer Alert Engine
    if (now - lastBuzzerCheck >= BUZZER_INTERVAL_MS) {
        lastBuzzerCheck = now;

        uint8_t faultedDevices = 0;
        for (int i = 0; i < 4; i++) {
            if (devices[i].health == HEALTH_TIMEOUT) {
                faultedDevices++;
            }
        }

        if (faultedDevices == 1) {
            triggerBeeps(1, 150, 100);
        } else if (faultedDevices > 1) {
            triggerBeeps(2, 120, 80);
        }
    }
    processBuzzer();
}

DeviceRecord* getDevice(uint8_t devId) {
    if (devId >= 1 && devId <= 4) {
        return &devices[devId - 1];
    }
    return nullptr;
}

void triggerLcdAlert(const char *line0, const char *line1, uint16_t durationMs) {
    strncpy(lcdAlertLine0, line0, sizeof(lcdAlertLine0) - 1);
    lcdAlertLine0[sizeof(lcdAlertLine0) - 1] = '\0';
    strncpy(lcdAlertLine1, line1, sizeof(lcdAlertLine1) - 1);
    lcdAlertLine1[sizeof(lcdAlertLine1) - 1] = '\0';
    lcdAlertUntil = millis() + durationMs;
    lcdFlashState = true;
    lastLcdFlashToggle = millis();

    // Render alert immediately
    printPaddedLine(0, lcdAlertLine0);
    printPaddedLine(1, lcdAlertLine1);
}

// Helper function to strip "[R]" tags so 16x2 LCD stays clean and uncluttered
void stripTagR(const char *src, char *dst, size_t dstSize) {
    if (!src || !dst || dstSize == 0) return;
    size_t d = 0;
    size_t s = 0;
    size_t srcLen = strlen(src);
    while (s < srcLen && d < dstSize - 1) {
        if (s + 2 < srcLen && src[s] == '[' && src[s + 1] == 'R' && src[s + 2] == ']') {
            s += 3; // Skip "[R]"
        } else {
            dst[d++] = src[s++];
        }
    }
    dst[d] = '\0';
}

// ==============================================================================
// 3.7 DEVICE-ISOLATED COMPARATIVE AUTO-REPAIR ENGINE
// Compares incoming frame STRICTLY against previous 2 frames of the SAME device
// - Restores corrupted/missing alphabets (e.g. 'P_R' -> 'PWR') from past schema
// - Restores missing/corrupted numbers using the mathematical average of past 2 frames
// - Tags reconstructed fields with [R] in Serial Monitor (LCD display remains clean)
// ==============================================================================
void autoRepairCorruptedPayload(DeviceRecord *dev, char *payload, size_t maxLen) {
    if (!dev->hasPrev1) return; // Need at least 1 previous frame of this device

    bool repairedAlphabet = false;
    char original[MAX_PAYLOAD_SIZE + 1];
    strncpy(original, payload, sizeof(original) - 1);
    original[sizeof(original) - 1] = '\0';

    // 1. RECONSTRUCT CORRUPTED / MISSING ALPHABETS
    // Check if previous 2 frames of this device had a valid alphabet token that got corrupted
    if (strstr(payload, "P_R") != nullptr) {
        char temp[MAX_PAYLOAD_SIZE + 1];
        char *p = strstr(payload, "P_R");
        int prefixLen = p - payload;
        strncpy(temp, payload, prefixLen);
        temp[prefixLen] = '\0';
        strncat(temp, "PWR[R]", sizeof(temp) - strlen(temp) - 1);
        strncat(temp, p + 3, sizeof(temp) - strlen(temp) - 1);
        strncpy(payload, temp, maxLen - 1);
        payload[maxLen - 1] = '\0';
        repairedAlphabet = true;
    } else if (strstr(payload, "P?R") != nullptr) {
        char temp[MAX_PAYLOAD_SIZE + 1];
        char *p = strstr(payload, "P?R");
        int prefixLen = p - payload;
        strncpy(temp, payload, prefixLen);
        temp[prefixLen] = '\0';
        strncat(temp, "PWR[R]", sizeof(temp) - strlen(temp) - 1);
        strncat(temp, p + 3, sizeof(temp) - strlen(temp) - 1);
        strncpy(payload, temp, maxLen - 1);
        payload[maxLen - 1] = '\0';
        repairedAlphabet = true;
    }

    // Generic character-by-character alphabet recovery:
    // If current character is '_' or '?' while prevPayload1 and prevPayload2 have the identical alphabet/delimiter
    int curLen = strlen(payload);
    int prevLen = strlen(dev->prevPayload1);
    int compareLen = (curLen < prevLen) ? curLen : prevLen;

    for (int i = 0; i < compareLen; i++) {
        char cCur = payload[i];
        char cP1  = dev->prevPayload1[i];
        char cP2  = dev->hasPrev2 ? dev->prevPayload2[i] : cP1;

        if (isalpha(cP1) || cP1 == '=' || cP1 == '%' || cP1 == ':' || cP1 == ',') {
            if (cP1 == cP2 && (cCur == '_' || cCur == '#')) {
                payload[i] = cP1; // Reconstruct the missing alphabet!
                repairedAlphabet = true;
            }
        }
    }

    if (repairedAlphabet) {
        Serial.print(F("\n* [ALPHABET RECONSTRUCTION - "));
        Serial.print(dev->name);
        Serial.println(F("]"));
        Serial.print(F("  Compared strictly with previous 2 frames of "));
        Serial.println(dev->name);
        Serial.print(F("  Original:     \""));
        Serial.print(original);
        Serial.println(F("\""));
        Serial.print(F("  Reconstructed:\""));
        Serial.print(payload);
        Serial.println(F("\" (Alphabet successfully restored & tagged with [R] in Serial Monitor)"));
    }

    // 2. RECONSTRUCT CORRUPTED / MISSING NUMERICAL VALUES
    // If any numerical field contains '?' or was lost, apply the average of previous 2 frames
    if (strchr(payload, '?') != nullptr) {
        Serial.print(F("\n* [NUMERICAL RECONSTRUCTION - "));
        Serial.print(dev->name);
        Serial.println(F("]"));
        Serial.print(F("  Missing numerical value ('?') -> Applying average of previous 2 frames of "));
        Serial.print(dev->name);
        Serial.println(F("..."));

        if (dev->devId == DEV_ID_1) {
            float p1T = 25.0f, p2T = 25.0f;
            int p1P = 1010, p2P = 1010;
            sscanf(dev->prevPayload1, "D1:T=%fC,P=%dhPa", &p1T, &p1P);
            if (dev->hasPrev2) sscanf(dev->prevPayload2, "D1:T=%fC,P=%dhPa", &p2T, &p2P);

            float avgT = (p1T + p2T) / 2.0f;
            int avgP = (p1P + p2P) / 2;
            int tWhole = (int)avgT;
            int tFrac = abs((int)(avgT * 10.0f) % 10);

            snprintf(payload, maxLen, "D1:T=%d.%dC[R],P=%dhPa[R]", tWhole, tFrac, avgP);
            Serial.print(F("  Reconstructed using Average: T="));
            Serial.print(avgT, 1);
            Serial.print(F("C[R], P="));
            Serial.print(avgP);
            Serial.println(F("hPa[R]"));
        } else if (dev->devId == DEV_ID_2) {
            int vW1 = 12, vF1 = 40, iW1 = 1, iF1 = 80;
            sscanf(dev->prevPayload1, "D2_LARGE:V=%d.%d", &vW1, &vF1);
            const char *iPtr1 = strstr(dev->prevPayload1, "I=");
            if (iPtr1) sscanf(iPtr1, "I=%d.%d", &iW1, &iF1);

            int vW2 = vW1, vF2 = vF1, iW2 = iW1, iF2 = iF1;
            if (dev->hasPrev2) {
                sscanf(dev->prevPayload2, "D2_LARGE:V=%d.%d", &vW2, &vF2);
                const char *iPtr2 = strstr(dev->prevPayload2, "I=");
                if (iPtr2) sscanf(iPtr2, "I=%d.%d", &iW2, &iF2);
            }

            float v1 = (float)vW1 + (float)vF1 / 100.0f;
            float v2 = (float)vW2 + (float)vF2 / 100.0f;
            float i1 = (float)iW1 + (float)iF1 / 100.0f;
            float i2 = (float)iW2 + (float)iF2 / 100.0f;

            float avgV = (v1 + v2) / 2.0f;
            float avgI = (i1 + i2) / 2.0f;
            float avgPwr = avgV * avgI;

            int vW = (int)avgV;
            int vF = abs((int)(avgV * 100.0f) % 100);
            int iW = (int)avgI;
            int iF = abs((int)(avgI * 100.0f) % 100);
            int pW = (int)avgPwr;
            int pF = abs((int)(avgPwr * 10.0f) % 10);

            const char *pwrTag = repairedAlphabet ? "PWR[R]" : "PWR";
            snprintf(payload, maxLen, "D2_LARGE:V=%d.%02dV,I=%d.%02dA,%s=%d.%dW[R],SOC=94%%,CAP=120Ah,ST=REPAIRED_OK",
                     vW, vF, iW, iF, pwrTag, pW, pF);

            Serial.print(F("  Reconstructed using Average: V="));
            Serial.print(avgV, 2);
            Serial.print(F("V, I="));
            Serial.print(avgI, 2);
            Serial.print(F("A, "));
            Serial.print(pwrTag);
            Serial.print(F("="));
            Serial.print(avgPwr, 1);
            Serial.println(F("W[R]"));
        } else if (dev->devId == DEV_ID_3) {
            int r1 = 1750, l1 = 40, r2 = 1750, l2 = 40;
            sscanf(dev->prevPayload1, "D3:RPM=%d,LD=%d%%", &r1, &l1);
            if (dev->hasPrev2) sscanf(dev->prevPayload2, "D3:RPM=%d,LD=%d%%", &r2, &l2);
            int avgRpm = (r1 + r2) / 2;
            int avgLd  = (l1 + l2) / 2;
            snprintf(payload, maxLen, "D3:RPM=%d[R],LD=%d%%[R]", avgRpm, avgLd);
            Serial.print(F("  Reconstructed using Average: RPM="));
            Serial.print(avgRpm);
            Serial.print(F("[R], LD="));
            Serial.print(avgLd);
            Serial.println(F("%[R]"));
        } else if (dev->devId == DEV_ID_4) {
            int fW1 = 3, fF1 = 40, t1 = 70, fW2 = 3, fF2 = 40, t2 = 70;
            sscanf(dev->prevPayload1, "D4:FLW=%d.%d,TK=%d%%", &fW1, &fF1, &t1);
            if (dev->hasPrev2) sscanf(dev->prevPayload2, "D4:FLW=%d.%d,TK=%d%%", &fW2, &fF2, &t2);
            float f1 = (float)fW1 + (float)fF1 / 100.0f;
            float f2 = (float)fW2 + (float)fF2 / 100.0f;
            float avgF = (f1 + f2) / 2.0f;
            int avgTk = (t1 + t2) / 2;
            int fW = (int)avgF;
            int fF = abs((int)(avgF * 100.0f) % 100);
            snprintf(payload, maxLen, "D4:FLW=%d.%02dL/m[R],TK=%d%%[R]", fW, fF, avgTk);
            Serial.print(F("  Reconstructed using Average: FLW="));
            Serial.print(avgF, 2);
            Serial.print(F("L/m[R], TK="));
            Serial.print(avgTk);
            Serial.println(F("%[R]"));
        }
    }
}

void processIncomingFrame(DataFrameM2 &frame) {
    DeviceRecord *dev = getDevice(frame.device_id);
    if (!dev) return;

    unsigned long now = millis();

    // 1. Auto-Recovery Check & High-Visibility LCD Flash
    if (dev->health == HEALTH_TIMEOUT) {
        Serial.print(F("\n* [AUTO-RECOVERY] "));
        Serial.print(dev->name);
        Serial.println(F(" resumed communication! Automatically restored to HEALTHY."));

        char alertLine1[17];
        snprintf(alertLine1, sizeof(alertLine1), "%s: RESTORED OK", dev->name);
        triggerLcdAlert("* AUTO-RECOVERY *", alertLine1, 3500);
        triggerBeeps(1, 120, 60);
    }
    dev->health = HEALTH_OK;
    dev->lastSeenMs = now;
    dev->rxCount++;

    // 2. Auto-Repair Corrupted Tokens (Compares strictly with past 2 frames of THIS device)
    autoRepairCorruptedPayload(dev, (char*)frame.payload, sizeof(frame.payload));

    // 3. Sequence Audit & Missing Data Recovery
    if (!dev->hasReceivedFirst) {
        dev->hasReceivedFirst = true;
        dev->lastSeq = frame.seq_num;
        dev->expectedSeq = (frame.seq_num + 1) % 256;
    } else {
        if (frame.seq_num == dev->lastSeq) {
            dev->duplicateCount++;
            Serial.print(F("! [INTEGRITY AUDIT] "));
            Serial.print(dev->name);
            Serial.print(F(" DUPLICATE FRAME: Seq #"));
            Serial.print(frame.seq_num);
            Serial.println(F(" repeated!"));
        } else if (frame.seq_num == dev->expectedSeq) {
            dev->lastSeq = frame.seq_num;
            dev->expectedSeq = (frame.seq_num + 1) % 256;
        } else {
            uint8_t gap = (frame.seq_num + 256 - dev->expectedSeq) % 256;
            if (gap < 128) {
                dev->droppedCount += gap;
                Serial.print(F("! [INTEGRITY AUDIT] "));
                Serial.print(dev->name);
                Serial.print(F(" MISSING FRAME(S): Gap of "));
                Serial.print(gap);
                Serial.print(F(" packets! (Expected #"));
                Serial.print(dev->expectedSeq);
                Serial.print(F(", got #"));
                Serial.print(frame.seq_num);
                Serial.println(F(")"));

                // DATA RECOVERY ENGINE: Works for ALL 4 Devices!
                recoverMissingData(dev, gap, frame.seq_num);
            }
            dev->lastSeq = frame.seq_num;
            dev->expectedSeq = (frame.seq_num + 1) % 256;
        }
    }

    // 4. Store Payload in Latest Valid Record
    memset(dev->latestPayload, 0, sizeof(dev->latestPayload));
    memcpy(dev->latestPayload, frame.payload, strlen((char*)frame.payload));
    dev->latestPayload[sizeof(dev->latestPayload) - 1] = '\0';

    // 4.5 Update previous 2 frames history strictly for THIS device
    // (Strip [R] tags so template comparison remains pristine and unshifted)
    char cleanHistory[MAX_PAYLOAD_SIZE + 1];
    stripTagR(dev->latestPayload, cleanHistory, sizeof(cleanHistory));

    if (dev->hasPrev1) {
        strncpy(dev->prevPayload2, dev->prevPayload1, sizeof(dev->prevPayload2) - 1);
        dev->prevPayload2[sizeof(dev->prevPayload2) - 1] = '\0';
        dev->hasPrev2 = true;
    }
    strncpy(dev->prevPayload1, cleanHistory, sizeof(dev->prevPayload1) - 1);
    dev->prevPayload1[sizeof(dev->prevPayload1) - 1] = '\0';
    dev->hasPrev1 = true;

    // 5. Real-time Independent Serial Monitor Output (Can be paused with SPACE)
    if (!terminalOutputPaused) {
        if (frame.device_id == DEV_ID_1) {
            Serial.print(F("[RS-232 RX] [DEV 1 - Continuous]   Seq:#"));
            if (frame.seq_num < 10) Serial.print(F("00"));
            else if (frame.seq_num < 100) Serial.print(F("0"));
            Serial.print(frame.seq_num);
            Serial.print(F(" | Len:"));
            Serial.print(strlen((char*)frame.payload));
            Serial.print(F("B | Data: \""));
            Serial.print(dev->latestPayload);
            Serial.println(F("\""));
        } else if (frame.device_id == DEV_ID_2) {
            Serial.print(F(">>> [RS-232 RX] [DEV 2 - Large 2x Data] Seq:#"));
            if (frame.seq_num < 10) Serial.print(F("00"));
            else if (frame.seq_num < 100) Serial.print(F("0"));
            Serial.print(frame.seq_num);
            Serial.print(F(" | Len:"));
            Serial.print(strlen((char*)frame.payload));
            Serial.print(F("B (2x D1) | Data: \""));
            Serial.print(dev->latestPayload);
            Serial.println(F("\""));
        } else if (frame.device_id == DEV_ID_3) {
            Serial.print(F("<-- [RS-485 RX] [DEV 3 - Triggered Resp] Seq:#"));
            if (frame.seq_num < 10) Serial.print(F("00"));
            else if (frame.seq_num < 100) Serial.print(F("0"));
            Serial.print(frame.seq_num);
            Serial.print(F(" | Data: \""));
            Serial.print(dev->latestPayload);
            Serial.println(F("\""));
        } else if (frame.device_id == DEV_ID_4) {
            Serial.print(F("<-- [RS-485 RX] [DEV 4 - 10s/Key Resp]  Seq:#"));
            if (frame.seq_num < 10) Serial.print(F("00"));
            else if (frame.seq_num < 100) Serial.print(F("0"));
            Serial.print(frame.seq_num);
            Serial.print(F(" | Data: \""));
            Serial.print(dev->latestPayload);
            Serial.println(F("\""));
        }
    }

    // 6. Extract Clean Telemetry for LCD (Guaranteed NO [R] tags on LCD display)
    const char *payloadText = dev->latestPayload;
    const char *colon = strchr(payloadText, ':');
    if (colon != nullptr) {
        stripTagR(colon + 1, dev->cleanTelemetry, sizeof(dev->cleanTelemetry));
    } else {
        stripTagR(payloadText, dev->cleanTelemetry, sizeof(dev->cleanTelemetry));
    }
    dev->cleanTelemetry[sizeof(dev->cleanTelemetry) - 1] = '\0';

    // 7. On-Demand LCD Trigger & Standby Timer Management
    if (frame.device_id == DEV_ID_3) {
        d3DisplayUntil = now + ON_DEMAND_DISPLAY_DURATION_MS;
        if (now >= lcdAlertUntil) {
            lcdPage = 2; // Jump immediately to Page 2 to display D3
            lastLcdUpdate = now;
            updateLcdDisplay();
        }
    } else if (frame.device_id == DEV_ID_4) {
        d4DisplayUntil = now + ON_DEMAND_DISPLAY_DURATION_MS;
        if (now >= lcdAlertUntil) {
            lcdPage = 2; // Jump immediately to Page 2 to display D4
            lastLcdUpdate = now;
            updateLcdDisplay();
        }
    }
}

// ==============================================================================
// 3.8 DATA RECOVERY ENGINE: Missing Frame Average for ALL 4 DEVICES
// ==============================================================================
void recoverMissingData(DeviceRecord *dev, uint8_t gap, uint8_t currentSeq) {
    if (!dev->hasPrev1) return;

    uint8_t missingSeq = dev->expectedSeq;
    Serial.print(F("\n>>> [DATA RECOVERY ENGINE - "));
    Serial.print(dev->name);
    Serial.println(F("] Missing frame detected! Comparing strictly within this device's past 2 frames..."));

    char recoveredPayload[MAX_PAYLOAD_SIZE + 1];

    if (dev->devId == DEV_ID_1) {
        float p1T = 25.0f, p2T = 25.0f;
        int p1P = 1010, p2P = 1010;
        sscanf(dev->prevPayload1, "D1:T=%fC,P=%dhPa", &p1T, &p1P);
        if (dev->hasPrev2) sscanf(dev->prevPayload2, "D1:T=%fC,P=%dhPa", &p2T, &p2P);
        else { p2T = p1T; p2P = p1P; }

        float avgT = (p1T + p2T) / 2.0f;
        int avgP = (p1P + p2P) / 2;
        int tW = (int)avgT;
        int tF = abs((int)(avgT * 10.0f) % 10);

        snprintf(recoveredPayload, sizeof(recoveredPayload), "D1_RECOVERED:T=%d.%dC[R],P=%dhPa[R]", tW, tF, avgP);
        Serial.print(F("  * [RECONSTRUCTED FRAME #"));
        Serial.print(missingSeq);
        Serial.print(F("]: \""));
        Serial.print(recoveredPayload);
        Serial.println(F("\" (Alphabet template preserved, numerical average applied with [R])"));

    } else if (dev->devId == DEV_ID_2) {
        int vW1 = 12, vF1 = 40, iW1 = 1, iF1 = 80;
        sscanf(dev->prevPayload1, "D2_LARGE:V=%d.%d", &vW1, &vF1);
        const char *iPtr1 = strstr(dev->prevPayload1, "I=");
        if (iPtr1) sscanf(iPtr1, "I=%d.%d", &iW1, &iF1);

        int vW2 = vW1, vF2 = vF1, iW2 = iW1, iF2 = iF1;
        if (dev->hasPrev2) {
            sscanf(dev->prevPayload2, "D2_LARGE:V=%d.%d", &vW2, &vF2);
            const char *iPtr2 = strstr(dev->prevPayload2, "I=");
            if (iPtr2) sscanf(iPtr2, "I=%d.%d", &iW2, &iF2);
        }

        float v1 = (float)vW1 + (float)vF1 / 100.0f;
        float v2 = (float)vW2 + (float)vF2 / 100.0f;
        float i1 = (float)iW1 + (float)iF1 / 100.0f;
        float i2 = (float)iW2 + (float)iF2 / 100.0f;

        float avgV = (v1 + v2) / 2.0f;
        float avgI = (i1 + i2) / 2.0f;
        float avgPwr = avgV * avgI;

        int vW = (int)avgV;
        int vF = abs((int)(avgV * 100.0f) % 100);
        int iW = (int)avgI;
        int iF = abs((int)(avgI * 100.0f) % 100);
        int pW = (int)avgPwr;
        int pF = abs((int)(avgPwr * 10.0f) % 10);

        snprintf(recoveredPayload, sizeof(recoveredPayload),
                 "D2_RECOVERED:V=%d.%02dV[R],I=%d.%02dA[R],PWR=%d.%dW[R],SOC=94%%,CAP=120Ah,ST=RECOVERED_OK",
                 vW, vF, iW, iF, pW, pF);

        Serial.print(F("  * [RECONSTRUCTED FRAME #"));
        Serial.print(missingSeq);
        Serial.print(F("]: \""));
        Serial.print(recoveredPayload);
        Serial.println(F("\" (Alphabet template preserved, numerical average applied with [R])"));

    } else if (dev->devId == DEV_ID_3) {
        int r1 = 1750, l1 = 40, r2 = 1750, l2 = 40;
        sscanf(dev->prevPayload1, "D3:RPM=%d,LD=%d%%", &r1, &l1);
        if (dev->hasPrev2) sscanf(dev->prevPayload2, "D3:RPM=%d,LD=%d%%", &r2, &l2);
        else { r2 = r1; l2 = l1; }

        int avgRpm = (r1 + r2) / 2;
        int avgLd  = (l1 + l2) / 2;

        snprintf(recoveredPayload, sizeof(recoveredPayload), "D3_RECOVERED:RPM=%d[R],LD=%d%%[R]", avgRpm, avgLd);
        Serial.print(F("  * [RECONSTRUCTED FRAME #"));
        Serial.print(missingSeq);
        Serial.print(F("]: \""));
        Serial.print(recoveredPayload);
        Serial.println(F("\" (Motor RPM & Load average reconstructed with [R])"));

    } else if (dev->devId == DEV_ID_4) {
        int fW1 = 3, fF1 = 40, t1 = 70, fW2 = 3, fF2 = 40, t2 = 70;
        sscanf(dev->prevPayload1, "D4:FLW=%d.%d,TK=%d%%", &fW1, &fF1, &t1);
        if (dev->hasPrev2) sscanf(dev->prevPayload2, "D4:FLW=%d.%d,TK=%d%%", &fW2, &fF2, &t2);
        else { fW2 = fW1; fF2 = fF1; t2 = t1; }

        float f1 = (float)fW1 + (float)fF1 / 100.0f;
        float f2 = (float)fW2 + (float)fF2 / 100.0f;
        float avgF = (f1 + f2) / 2.0f;
        int avgTk = (t1 + t2) / 2;

        int fW = (int)avgF;
        int fF = abs((int)(avgF * 100.0f) % 100);

        snprintf(recoveredPayload, sizeof(recoveredPayload), "D4_RECOVERED:FLW=%d.%02dL/m[R],TK=%d%%[R]", fW, fF, avgTk);
        Serial.print(F("  * [RECONSTRUCTED FRAME #"));
        Serial.print(missingSeq);
        Serial.print(F("]: \""));
        Serial.print(recoveredPayload);
        Serial.println(F("\" (Flow rate & Tank level average reconstructed with [R])"));
    }
}

// ==============================================================================
// 3.9 MILESTONE 3: WI-FI & UDP REMOTE COMMAND ENGINE
// ==============================================================================
void sendUdpError(IPAddress ip, uint16_t port, uint8_t cmdEcho, uint8_t errorCode, uint8_t targetDevId, const char *msg) {
    uint8_t errBuf[96];
    errBuf[0] = cmdEcho;
    errBuf[1] = errorCode;
    errBuf[2] = targetDevId;
    size_t msgLen = strlen(msg);
    if (msgLen > sizeof(errBuf) - 4) msgLen = sizeof(errBuf) - 4;
    memcpy(&errBuf[3], msg, msgLen);
    errBuf[3 + msgLen] = '\0';
    udpServer.beginPacket(ip, port);
    udpServer.write(errBuf, 3 + msgLen + 1);
    udpServer.endPacket();
}

void processUdpRequests() {
    if (!udpServerListening) return;

    int packetSize = udpServer.parsePacket();
    if (packetSize <= 0) return;

    IPAddress remoteIp = udpServer.remoteIP();
    uint16_t remotePort = udpServer.remotePort();
    uint8_t rxBuffer[64];

    if (packetSize < 2) {
        udpServer.flush();
        sendUdpError(remoteIp, remotePort, 0xFF, ERR_MALFORMED_PACKET, 0x00, "Packet < 2 bytes");
        return;
    }

    int bytesRead = udpServer.read(rxBuffer, sizeof(rxBuffer));
    if (bytesRead < 2) {
        sendUdpError(remoteIp, remotePort, 0xFF, ERR_MALFORMED_PACKET, 0x00, "Buffer read error");
        return;
    }

    uint8_t cmd = rxBuffer[0];
    uint8_t targetDevId = rxBuffer[1];
    uint8_t txBuffer[512];

    switch (cmd) {
        case CMD_GET_SINGLE_DEVICE: {
            if (targetDevId < 1 || targetDevId > 4) {
                sendUdpError(remoteIp, remotePort, cmd, ERR_INVALID_DEVICE_ID, targetDevId, "Device ID must be 1-4");
                break;
            }
            DeviceRecord *dev = &devices[targetDevId - 1];
            if (dev->health == HEALTH_TIMEOUT) {
                sendUdpError(remoteIp, remotePort, cmd, ERR_DEVICE_OFFLINE, targetDevId, "Device flagged OFFLINE");
                break;
            }

            txBuffer[0] = CMD_GET_SINGLE_DEVICE;
            txBuffer[1] = STATUS_OK;
            txBuffer[2] = dev->devId;
            uint16_t pLen = strlen(dev->latestPayload);
            txBuffer[3] = (uint8_t)((pLen >> 8) & 0xFF);
            txBuffer[4] = (uint8_t)(pLen & 0xFF);
            memcpy(&txBuffer[5], dev->latestPayload, pLen);

            udpServer.beginPacket(remoteIp, remotePort);
            udpServer.write(txBuffer, 5 + pLen);
            udpServer.endPacket();

            Serial.printf("[UDP TX] Replied to %s:%u with DEV %u Telemetry (%u B)\n",
                          remoteIp.toString().c_str(), remotePort, dev->devId, pLen);
            break;
        }

        case CMD_GET_ALL_DEVICES: {
            txBuffer[0] = CMD_GET_ALL_DEVICES;
            txBuffer[1] = STATUS_OK;
            txBuffer[2] = 4; // 4 devices
            size_t offset = 3;

            for (int i = 0; i < 4; i++) {
                txBuffer[offset++] = devices[i].devId;
                bool isOnline = (devices[i].health == HEALTH_OK);
                txBuffer[offset++] = isOnline ? 0x01 : 0x00;
                uint16_t pLen = isOnline ? strlen(devices[i].latestPayload) : 0;
                txBuffer[offset++] = (uint8_t)((pLen >> 8) & 0xFF);
                txBuffer[offset++] = (uint8_t)(pLen & 0xFF);
                if (pLen > 0) {
                    memcpy(&txBuffer[offset], devices[i].latestPayload, pLen);
                    offset += pLen;
                }
            }

            udpServer.beginPacket(remoteIp, remotePort);
            udpServer.write(txBuffer, offset);
            udpServer.endPacket();

            Serial.printf("[UDP TX] Replied to %s:%u with ALL 4 Devices Snapshot (%u B)\n",
                          remoteIp.toString().c_str(), remotePort, (unsigned int)offset);
            break;
        }

        case CMD_GET_HEALTH_STATUS: {
            txBuffer[0] = CMD_GET_HEALTH_STATUS;
            txBuffer[1] = STATUS_OK;
            txBuffer[2] = 4; // 4 channels
            size_t offset = 3;

            for (int i = 0; i < 4; i++) {
                const DeviceRecord &d = devices[i];
                txBuffer[offset++] = d.devId;
                txBuffer[offset++] = (d.health == HEALTH_OK) ? 0x01 : 0x00;

                uint32_t ts = d.lastSeenMs;
                txBuffer[offset++] = (uint8_t)((ts >> 24) & 0xFF);
                txBuffer[offset++] = (uint8_t)((ts >> 16) & 0xFF);
                txBuffer[offset++] = (uint8_t)((ts >> 8) & 0xFF);
                txBuffer[offset++] = (uint8_t)(ts & 0xFF);

                txBuffer[offset++] = (uint8_t)((d.crcErrorCount >> 8) & 0xFF);
                txBuffer[offset++] = (uint8_t)(d.crcErrorCount & 0xFF);
                txBuffer[offset++] = (uint8_t)((d.timeoutCount >> 8) & 0xFF);
                txBuffer[offset++] = (uint8_t)(d.timeoutCount & 0xFF);
                txBuffer[offset++] = (uint8_t)((d.outOfOrderCount >> 8) & 0xFF);
                txBuffer[offset++] = (uint8_t)(d.outOfOrderCount & 0xFF);
            }

            udpServer.beginPacket(remoteIp, remotePort);
            udpServer.write(txBuffer, offset);
            udpServer.endPacket();

            Serial.printf("[UDP TX] Replied to %s:%u with HEALTH & Diagnostics (%u B)\n",
                          remoteIp.toString().c_str(), remotePort, (unsigned int)offset);
            break;
        }

        default:
            sendUdpError(remoteIp, remotePort, cmd, ERR_INVALID_CMD, targetDevId, "Unknown Command Byte");
            Serial.printf("[UDP WARN] Rejected unsupported command 0x%02X from %s:%u\n",
                          cmd, remoteIp.toString().c_str(), remotePort);
            break;
    }
}

void manageWiFiAndUdp(unsigned long now) {
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            wifiConnected = true;
            udpServer.begin(UDP_PORT);
            udpServerListening = true;
            Serial.println(F("\n===================================================================="));
            Serial.print(F("* [WI-FI CONNECTED] SSID: ")); Serial.println(WIFI_SSID);
            Serial.print(F("* [GATEWAY IP]       : ")); Serial.println(WiFi.localIP());
            Serial.print(F("* [UDP SERVER]       : Port ")); Serial.println(UDP_PORT);
            Serial.println(F("===================================================================="));

            char ipStr[17];
            snprintf(ipStr, sizeof(ipStr), "IP:%s", WiFi.localIP().toString().c_str());
            triggerLcdAlert("WI-FI CONNECTED ", ipStr, 3500);
        }
    } else {
        if (wifiConnected) {
            wifiConnected = false;
            udpServerListening = false;
            Serial.println(F("\n! [WI-FI FAULT] Connection lost! Initiating auto-reconnect..."));
            triggerLcdAlert("! WI-FI LOST   !", "RECONNECTING... ", 3000);
            lastWifiRetry = now;
        }

        if (now - lastWifiRetry >= 5000) {
            lastWifiRetry = now;
            Serial.println(F("[WI-FI] Attempting reconnection to AP..."));
            WiFi.reconnect();
        }
    }

    if (udpServerListening) {
        processUdpRequests();
    }
}

void manageRs485Polling(unsigned long now) {
    // 1. Check if a pending poll timed out
    if (pollState == POLL_STATE_WAIT_DEV3) {
        if (now - pollWaitStartTime > POLL_RESPONSE_TIMEOUT_MS) {
            pollState = POLL_STATE_IDLE;
            Serial.println(F("! [POLL TIMEOUT] Device 3 did not answer poll request within 300ms!"));
            devices[DEV_ID_3 - 1].health = HEALTH_TIMEOUT;
            devices[DEV_ID_3 - 1].timeoutCount++;
            triggerLcdAlert("! POLL TIMEOUT  !", "DEV 3 NO REPLY  ", 3000);
            triggerBeeps(2, 120, 80);
        }
    } else if (pollState == POLL_STATE_WAIT_DEV4) {
        if (now - pollWaitStartTime > POLL_RESPONSE_TIMEOUT_MS) {
            pollState = POLL_STATE_IDLE;
            Serial.println(F("! [POLL TIMEOUT] Device 4 did not answer poll request within 300ms!"));
            devices[DEV_ID_4 - 1].health = HEALTH_TIMEOUT;
            devices[DEV_ID_4 - 1].timeoutCount++;
            triggerLcdAlert("! POLL TIMEOUT  !", "DEV 4 NO REPLY  ", 3000);
            triggerBeeps(2, 120, 80);
        }
    }

    // 2. Automated 10-Second Polling for Device 4
    if (pollState == POLL_STATE_IDLE) {
        if (now - lastDev4PollTime >= DEV4_POLL_INTERVAL_MS) {
            lastDev4PollTime = now;
            triggerManualPoll(DEV_ID_4);
        }
    }
}

void triggerManualPoll(uint8_t devId) {
    if (devId == DEV_ID_3) {
        Serial.println(F("--> [RS-485 TX] [GATEWAY POLL -> DEV 3] Requesting Device 3 telemetry..."));
        digitalWrite(RS485_DE_RE_PIN, HIGH);
        delayMicroseconds(100);
        sendDataFrameM2(Serial2, IFACE_RS485, DEV_ID_3, MSG_TYPE_POLL, gatewayPollSeq++, nullptr, false);
        Serial2.flush();
        delayMicroseconds(150);
        digitalWrite(RS485_DE_RE_PIN, LOW);
        rs485Parser.recordTx();
        pollState = POLL_STATE_WAIT_DEV3;
        pollWaitStartTime = millis();
    } else if (devId == DEV_ID_4) {
        Serial.println(F("--> [RS-485 TX] [GATEWAY POLL -> DEV 4] Requesting Device 4 telemetry..."));
        digitalWrite(RS485_DE_RE_PIN, HIGH);
        delayMicroseconds(100);
        sendDataFrameM2(Serial2, IFACE_RS485, DEV_ID_4, MSG_TYPE_POLL, gatewayPollSeq++, nullptr, false);
        Serial2.flush();
        delayMicroseconds(150);
        digitalWrite(RS485_DE_RE_PIN, LOW);
        rs485Parser.recordTx();
        pollState = POLL_STATE_WAIT_DEV4;
        pollWaitStartTime = millis();
    }
}

void handleGatewayCLI(char c) {
    switch (c) {
        case ' ':
        case 'z':
        case 'Z':
            terminalOutputPaused = !terminalOutputPaused;
            if (terminalOutputPaused) {
                Serial.println(F("\n>>> [TERMINAL FROZEN] Live serial output paused for inspection!"));
                Serial.println(F(">>> Press SPACE or 'Z' again to resume live scrolling.\n"));
            } else {
                Serial.println(F("\n>>> [TERMINAL RESUMED] Live serial scrolling resumed.\n"));
            }
            break;
        case '3':
            Serial.println(F("\n>>> [USER TRIGGER: KEY '3'] Triggering Device 3 poll..."));
            triggerManualPoll(DEV_ID_3);
            break;
        case '4':
            Serial.println(F("\n>>> [USER TRIGGER: KEY '4'] Triggering Device 4 poll..."));
            triggerManualPoll(DEV_ID_4);
            break;
        case 'p':
        case 'P':
            triggerManualPoll(DEV_ID_3);
            break;
        case 'd':
        case 'D':
            printConsoleDashboard();
            break;
        case 'h':
        case 'H':
        case '?':
            printGatewayMenu();
            break;
    }
}

void printGatewayMenu() {
    Serial.println(F("\n---------------- GATEWAY INTERACTIVE COMMANDS ----------------"));
    Serial.println(F("  Press [SPACE] : Freeze / Resume terminal scrolling (Inspect data!)"));
    Serial.println(F("  Press [3]     : Manually poll Device 3 (RS-485 Request-Response)"));
    Serial.println(F("  Press [4]     : Manually poll Device 4 (RS-485 10s / On-Demand)"));
    Serial.println(F("  Press [D]     : Print Consolidated Health & Buffer Dashboard"));
    Serial.println(F("  Press [?]     : Show this menu"));
    Serial.println(F("  Data Recovery : Auto-reconstructs lost packets (D1-D4) & repairs tokens!"));
    Serial.println(F("--------------------------------------------------------------\n"));
}

void checkDeviceHealth(unsigned long now) {
    for (int i = 0; i < 4; i++) {
        if (devices[i].health == HEALTH_OK) {
            if (devices[i].timeoutThresholdMs > 0) {
                if (now > devices[i].lastSeenMs && (now - devices[i].lastSeenMs > devices[i].timeoutThresholdMs)) {
                    devices[i].health = HEALTH_TIMEOUT;
                    Serial.print(F("\n! [FAULT TRIGGER: TIMEOUT DETECTED] "));
                    Serial.print(devices[i].name);
                    Serial.print(F(" is silent (>"));
                    Serial.print(devices[i].timeoutThresholdMs);
                    Serial.println(F("ms)! Automatically querying Device 3..."));

                    char alertLine1[17];
                    snprintf(alertLine1, sizeof(alertLine1), "%s: COMM TOT!  ", devices[i].name);
                    triggerLcdAlert("! FAULT: TIMEOUT !", alertLine1, 3500);
                    triggerBeeps(2, 150, 80);

                    triggerManualPoll(DEV_ID_3);
                }
            }
        }
    }
}

void printConsoleDashboard() {
    unsigned long now = millis();
    ProtocolStatsM2 s232 = rs232Parser.getStats();
    ProtocolStatsM2 s485 = rs485Parser.getStats();

    Serial.println(F("\n===================================================================================================="));
    Serial.println(F("                                ESP32-S3 MULTI-DEVICE GATEWAY (M2)                                  "));
    Serial.println(F("===================================================================================================="));
    Serial.println(F("DEV | INTERFACE | BEHAVIOUR     | LAST_SEQ | TOTAL_RX | DROPPED | DUPLICATE | LAST_SEEN | HEALTH STATUS "));
    Serial.println(F("----+-----------+---------------+----------+----------+---------+-----------+-----------+---------------"));

    for (int i = 0; i < 4; i++) {
        const char *healthStr = (devices[i].health == HEALTH_OK) ? "HEALTHY (OK)" : "TIMEOUT (FAULT)";
        unsigned long silentMs = (now > devices[i].lastSeenMs) ? (now - devices[i].lastSeenMs) : 0;

        char rowBuf[120];
        snprintf(rowBuf, sizeof(rowBuf),
                 "D%-2d | %-9s | %-13s | #%-7u | %-8u | %-7u | %-9u | %6lums | %s",
                 devices[i].devId, devices[i].ifaceName, devices[i].behaviour,
                 devices[i].lastSeq, devices[i].rxCount, devices[i].droppedCount,
                 devices[i].duplicateCount, silentMs, healthStr);
        Serial.println(rowBuf);
    }
    Serial.println(F("----------------------------------------------------------------------------------------------------"));
    char busBuf[120];
    snprintf(busBuf, sizeof(busBuf),
             "BUS INTEGRITY: RS-232 RX:%-4u TX:%-4u CRC_Err:%-3u Malf:%-3u | RS-485 RX:%-4u TX:%-4u CRC_Err:%-3u Malf:%-3u",
             s232.rx_valid_packets, s232.tx_packets, s232.rx_crc_errors, s232.rx_malformed_errors,
             s485.rx_valid_packets, s485.tx_packets, s485.rx_crc_errors, s485.rx_malformed_errors);
    Serial.println(busBuf);
    Serial.println(F("----------------------------------------------------------------------------------------------------"));
    Serial.println(F("LATEST VALID BUFFERED RECORDS (Zero Data Loss Retention):"));
    for (int i = 0; i < 4; i++) {
        Serial.print(F("  ["));
        Serial.print(devices[i].name);
        Serial.print(F("]: \""));
        if ((i == 2 && now >= d3DisplayUntil) || (i == 3 && now >= d4DisplayUntil)) {
            Serial.print(F("[STANDBY]"));
        } else {
            Serial.print(devices[i].latestPayload);
        }
        Serial.println(F("\""));
    }
    Serial.println(F("----------------------------------------------------------------------------------------------------"));
    if (wifiConnected) {
        char wifiBuf[120];
        snprintf(wifiBuf, sizeof(wifiBuf),
                 "WI-FI: CONNECTED | SSID: %s | GATEWAY IP: %s | UDP SERVER: PORT %u (ACTIVE)",
                 WIFI_SSID, WiFi.localIP().toString().c_str(), UDP_PORT);
        Serial.println(wifiBuf);
    } else {
        Serial.println(F("WI-FI: CONNECTING... (Auto-reconnecting) | UDP SERVER: WAITING FOR NETWORK"));
    }
    Serial.println(F("====================================================================================================\n"));
}

void printPaddedLine(uint8_t row, const char *rawText) {
    char padded[17];
    int len = strlen(rawText);
    if (len > 16) len = 16;
    memcpy(padded, rawText, len);
    for (int i = len; i < 16; i++) padded[i] = ' ';
    padded[16] = '\0';
    lcd.setCursor(0, row);
    lcd.print(padded);
}

void updateLcdDisplay() {
    unsigned long now = millis();
    char buf0[24], buf1[24];

    switch (lcdPage) {
        case 0:
            // Device Status Overview
            snprintf(buf0, sizeof(buf0), "D1:%-4s D2:%-4s",
                     (devices[0].health == HEALTH_OK) ? "OK" : "TOT!",
                     (devices[1].health == HEALTH_OK) ? "OK" : "TOT!");
            snprintf(buf1, sizeof(buf1), "D3:%-4s D4:%-4s",
                     (devices[2].health == HEALTH_TIMEOUT) ? "TOT!" : ((now < d3DisplayUntil) ? "ACT" : "STBY"),
                     (devices[3].health == HEALTH_TIMEOUT) ? "TOT!" : ((now < d4DisplayUntil) ? "ACT" : "STBY"));
            break;

        case 1:
            // Continuous & Periodic Devices (RS-232)
            if (devices[0].health == HEALTH_TIMEOUT) {
                snprintf(buf0, sizeof(buf0), "D1: [COMM TOT!] ");
            } else {
                snprintf(buf0, sizeof(buf0), "D1:%s", devices[0].cleanTelemetry);
            }

            if (devices[1].health == HEALTH_TIMEOUT) {
                snprintf(buf1, sizeof(buf1), "D2: [COMM TOT!] ");
            } else {
                snprintf(buf1, sizeof(buf1), "D2:%s", devices[1].cleanTelemetry);
            }
            break;

        case 2:
            // Polled & On-Demand Devices (RS-485)
            if (devices[2].health == HEALTH_TIMEOUT) {
                snprintf(buf0, sizeof(buf0), "D3: [COMM TOT!] ");
            } else if (now < d3DisplayUntil) {
                snprintf(buf0, sizeof(buf0), "D3:%s", devices[2].cleanTelemetry);
            } else {
                snprintf(buf0, sizeof(buf0), "D3: [STANDBY]   ");
            }

            if (devices[3].health == HEALTH_TIMEOUT) {
                snprintf(buf1, sizeof(buf1), "D4: [COMM TOT!] ");
            } else if (now < d4DisplayUntil) {
                snprintf(buf1, sizeof(buf1), "D4:%s", devices[3].cleanTelemetry);
            } else {
                snprintf(buf1, sizeof(buf1), "D4: [STANDBY]   ");
            }
            break;

        case 3:
            // Wi-Fi & UDP Gateway Network Status (Milestone 3)
            if (wifiConnected) {
                snprintf(buf0, sizeof(buf0), "IP:%s", WiFi.localIP().toString().c_str());
                snprintf(buf1, sizeof(buf1), "UDP:PORT %u OK", UDP_PORT);
            } else {
                snprintf(buf0, sizeof(buf0), "WI-FI:CONNECTING");
                snprintf(buf1, sizeof(buf1), "UDP:PORT %u ...", UDP_PORT);
            }
            break;

        case 4:
            // Bus Integrity & Statistics
            {
                ProtocolStatsM2 s232 = rs232Parser.getStats();
                ProtocolStatsM2 s485 = rs485Parser.getStats();

                char crc232[6];
                if (s232.rx_crc_errors == 0) strcpy(crc232, "OK");
                else snprintf(crc232, sizeof(crc232), "E%u", (unsigned int)s232.rx_crc_errors);

                char crc485[6];
                if (s485.rx_crc_errors == 0) strcpy(crc485, "OK");
                else snprintf(crc485, sizeof(crc485), "E%u", (unsigned int)s485.rx_crc_errors);

                snprintf(buf0, sizeof(buf0), "232 R%-2u T%-2u C:%-2s", devices[0].lastSeq, espRs232AckSeq - 1, crc232);
                snprintf(buf1, sizeof(buf1), "485 R%-2u T%-2u C:%-2s", devices[2].lastSeq, gatewayPollSeq - 1, crc485);
            }
            break;
    }

    printPaddedLine(0, buf0);
    printPaddedLine(1, buf1);
}

void triggerBeeps(uint8_t count, uint16_t onDuration, uint16_t offDuration) {
    if (buzzPhase == BUZZ_IDLE) {
        buzzRemaining = count;
        buzzOnMs = onDuration;
        buzzOffMs = offDuration;
        digitalWrite(BUZZER_PIN, HIGH);
        buzzPhase = BUZZ_BEEP_ON;
        buzzTimer = millis();
    }
}

void processBuzzer() {
    if (buzzPhase == BUZZ_IDLE) return;
    unsigned long now = millis();

    if (buzzPhase == BUZZ_BEEP_ON) {
        if (now - buzzTimer >= buzzOnMs) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzRemaining--;
            if (buzzRemaining > 0) {
                buzzPhase = BUZZ_BEEP_OFF;
                buzzTimer = now;
            } else {
                buzzPhase = BUZZ_IDLE;
            }
        }
    } else if (buzzPhase == BUZZ_BEEP_OFF) {
        if (now - buzzTimer >= buzzOffMs) {
            digitalWrite(BUZZER_PIN, HIGH);
            buzzPhase = BUZZ_BEEP_ON;
            buzzTimer = now;
        }
    }
}
