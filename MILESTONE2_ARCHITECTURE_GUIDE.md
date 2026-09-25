# Milestone 2: Concurrent Four-Device Communication and Data Management

## Executive Summary & Architectural Overview

Milestone 2 advances the dual-interface gateway into a **concurrent multi-device data management hub**. Using only the provided hardware resources (**Arduino Mega 2560** and **ESP32-S3**), the system emulates, multiplexes, validates, buffers, and displays telemetry from **four logical field devices** across two distinct physical communication buses:

1. **RS-232 Bus (`Serial1` @ 9600 baud)**:
   - **Device 1**: Continuous high-speed telemetry stream (transmits every 300 ms).
   - **Device 2**: Burst / Periodic diagnostic telemetry (transmits 3 frames in a burst every 2500 ms).
2. **RS-485 Bus (`Serial2` @ 19200 baud, Half-Duplex)**:
   - **Device 3**: Fast request-response polled slave (polled by Gateway every 500 ms).
   - **Device 4**: Slow request-response polled slave (polled by Gateway every 1200 ms).

```
 +-------------------------------------------------------------------------------+
 |                       ARDUINO MEGA 2560 (FIELD UNIT)                          |
 |                                                                               |
 |   [DEV 1: RS-232] Continuous Sensor Stream (every 300ms)                      |
 |   [DEV 2: RS-232] Periodic Burst Stream (3 frames every 2500ms)               |
 |   [DEV 3: RS-485] Slave Listener (Responds to Master Poll 0x03)               |
 |   [DEV 4: RS-485] Slave Listener (Responds to Master Poll 0x04)               |
 |   [DIAGNOSTIC CLI] Interactive Keyboard Fault Injector (via USB Serial)       |
 +-----------------------+-------------------------------+-----------------------+
                         | Serial1 (RS-232 @ 9600)       | Serial2 (RS-485 @ 19200)
                         v                               v
 +-----------------------+-------------------------------+-----------------------+
 |                         ESP32-S3 (CENTRAL GATEWAY)                            |
 |                                                                               |
 |   • Non-Blocking UART Ingestion: RS-232 stream unpacker                       |
 |   • RS-485 Polling Engine: Dynamic time-sliced scheduler (500ms & 1200ms)     |
 |   • 4x Independent Device Records & Ring Buffers                              |
 |   • Sequence Tracking: Missing, Duplicate, Out-of-Order frame detection      |
 |   • Integrity Validation: Dallas/Maxim CRC-8 (0x31) on every frame            |
 |   • Fault Isolation: Individual timeouts; failure of 1 device never affects 3 |
 |   • Auto-Recovery: Restores healthy status immediately upon reconnection      |
 |   • ANSI Console Dashboard: Live tabular health overview                      |
 |   • 16x2 LCD Display: 4-page cycling overview & live telemetry                |
 |   • Buzzer Engine: Audible diagnostic alert beeps                             |
 +-------------------------------------------------------------------------------+
```

---

## Data Frame Format (Milestone 2 Extension)

Every frame transmitted over both buses follows an industrial structured format:

| Byte Index | Field | Description | Possible Values |
| :--- | :--- | :--- | :--- |
| `0` | **SOF** | Start of Frame delimiter | `0xAA` |
| `1` | **IFACE_ID** | Physical interface identifier | `0x01` (RS-232), `0x02` (RS-485) |
| `2` | **DEV_ID** | Logical Device identifier | `0x01` (Dev 1), `0x02` (Dev 2), `0x03` (Dev 3), `0x04` (Dev 4), `0x00` (Gateway) |
| `3` | **MSG_TYPE** | Frame purpose | `0x10` (DATA), `0x20` (POLL Request), `0x30` (ACK) |
| `4` | **SEQ_NUM** | 8-bit rolling sequence counter | `0x00` – `0xFF` (tracked independently per device) |
| `5` | **PAYLOAD_LEN** | Number of bytes in payload | `0` – `64` |
| `6 .. 5+LEN` | **PAYLOAD** | ASCII telemetry string | e.g. `"D1:T=26.4C,P=1013hPa"` |
| `6+LEN` | **CRC-8** | Dallas/Maxim polynomial `0x31` | Calculated over bytes 1 through `5+LEN` |
| `7+LEN` | **EOF** | End of Frame delimiter | `0x55` |

---

## 4 Logical Device Specifications

### Device 1: RS-232 Continuous Stream
- **ID**: `0x01`
- **Interface**: RS-232 (`Serial1`, 9600 baud)
- **Behaviour**: Continuous transmission every 300 ms.
- **Simulated Sensors**: Ambient Temperature (`simTemp`) & Atmospheric Pressure (`simPress`).
- **Gateway Ingestion**: Evaluated as an incoming asynchronous stream. Sequence numbers strictly monotonic (`seq == expectedSeq`).
- **Timeout Threshold**: 1500 ms (5 missed continuous intervals).

### Device 2: RS-232 Burst / Periodic Transmission
- **ID**: `0x02`
- **Interface**: RS-232 (`Serial1`, 9600 baud)
- **Behaviour**: Burst transmission of 3 consecutive packets every 2500 ms.
- **Simulated Sensors**: Battery Voltage (`V`), Discharge Current (`I`), Total Power (`PWR`).
- **Gateway Ingestion**: The gateway handles 3 rapid packets in sequence without frame collision or buffer corruption.
- **Timeout Threshold**: 5500 ms (accommodates 2.5s cycle with margin).

### Device 3: RS-485 Polled Slave (Fast Rate)
- **ID**: `0x03`
- **Interface**: RS-485 (`Serial2`, 19200 baud, Half-Duplex)
- **Behaviour**: Master-polled request-response every 500 ms.
- **Simulated Sensors**: Industrial Motor RPM (`simRpm`) and Motor Load percentage (`simLoad`).
- **Bus Contention Management**: ESP32 drives DE/RE HIGH -> sends `MSG_TYPE_POLL` -> sets DE/RE LOW -> Mega receives poll -> drives DE/RE HIGH -> replies with `MSG_TYPE_DATA` -> flushes -> releases line.
- **Timeout Threshold**: 2000 ms (4 missed poll replies).

### Device 4: RS-485 Polled Slave (Slow / Different Rate)
- **ID**: `0x04`
- **Interface**: RS-485 (`Serial2`, 19200 baud, Half-Duplex)
- **Behaviour**: Master-polled request-response every 1200 ms.
- **Simulated Sensors**: Flow Rate (`simFlow`) and Storage Tank percentage (`simTank`).
- **Gateway Ingestion**: Co-exists on the shared RS-485 differential bus with Device 3 without collisions.
- **Timeout Threshold**: 3500 ms (3 missed poll replies).

---

## Fault Management & Integrity Assurance

| Condition | Detection Mechanism | Gateway Action & Recovery |
| :--- | :--- | :--- |
| **Missing Packets** | Sequence gap: `(seq - expectedSeq) > 0` | Increments `droppedCount`, logs gap, updates `expectedSeq`. |
| **Duplicate Packets**| `seq == lastSeq` | Increments `duplicateCount`, discards duplicate payload. |
| **CRC Corruption** | Dallas CRC-8 mismatch | Increments `rx_crc_errors`, aborts invalid frame. |
| **Device Timeout** | `(now - lastSeenMs) > timeoutThresholdMs` | Marks device `TIMEOUT (FAULT)`, triggers audible buzzer warning. |
| **Device Recovery** | First valid frame received after timeout | Automatically marks device `HEALTHY (OK)`, resets health state. |
| **Fault Isolation** | Non-blocking scheduling & independent buffers | If Dev 1 or Dev 3 fails, Dev 2 and Dev 4 remain 100% active. |
