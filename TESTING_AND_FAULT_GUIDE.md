# Multi-Device Gateway: Testing & Fault Injection Guide

This guide details how to upload, operate, and verify all multi-device gateway operational and fault injection test cases.

---

## 1. Zero Hardware Rewiring Required

The hardware setup is **100% identical to Milestone 1**:
- **Arduino Mega**:
  - `Serial1`: RS-232 on Pins 18 (TX1), 19 (RX1) @ 9600 baud.
  - `Serial2`: RS-485 on Pins 16 (TX2), 17 (RX2), Pin 2 (DE/RE) @ 19200 baud.
  - `I2C`: 20x4 LCD on Pins 20 (SDA), 21 (SCL).
- **ESP32-S3**:
  - `Serial1`: RS-232 on GPIO 18 (RX1), GPIO 17 (TX1) @ 9600 baud.
  - `Serial2`: RS-485 on GPIO 15 (RX2), GPIO 16 (TX2), GPIO 6 (DE/RE) @ 19200 baud.
  - `I2C`: 16x2 LCD on GPIO 8 (SDA), GPIO 9 (SCL).
  - `Buzzer`: GPIO 13.
  - `Flashlight LED`: GPIO 4 (kept LOW).

---

## 2. Flashing the Firmware

### A. Arduino Mega 2560
1. Open Arduino IDE.
2. Open `Milestone2_Gateway/ArduinoMega_Milestone2/ArduinoMega_Milestone2.ino`.
3. Set **Tools -> Board** to `Arduino Mega or Mega 2560`.
4. Set **Tools -> Port** to the Mega COM port.
5. Click **Upload**.
6. Open **Serial Monitor** at **115200 baud**.

### B. ESP32-S3 Gateway
1. Open Arduino IDE in a second window.
2. Open `Milestone2_Gateway/ESP32S3_Gateway_Milestone2/ESP32S3_Gateway_Milestone2.ino`.
3. Set **Tools -> Board** to `ESP32S3 Dev Module`.
4. Ensure **Tools -> USB CDC On Boot** is set to **`Enabled`**.
5. Set **Tools -> Port** to the ESP32 COM port.
6. Click **Upload**.
7. Open **Serial Monitor** at **115200 baud**.

---

## 3. Normal Healthy State (Baseline Verification)

Once both nodes are running:
1. **ESP32-S3 Serial Monitor**: Prints the ANSI multi-device table every 2.5 seconds:
   ```
   ====================================================================================================
                                   ESP32-S3 MULTI-DEVICE GATEWAY (M2)                                  
   ====================================================================================================
   DEV | INTERFACE | BEHAVIOUR     | LAST_SEQ | TOTAL_RX | DROPPED | DUPLICATE | LAST_SEEN | HEALTH STATUS 
   ----+-----------+---------------+----------+----------+---------+-----------+-----------+---------------
   D1  | RS-232    | Continuous    | #042     | 126      | 0       | 0         |      80ms | HEALTHY (OK)  
   D2  | RS-232    | Burst/Period  | #018     | 27       | 0       | 0         |    1200ms | HEALTHY (OK)  
   D3  | RS-485    | Poll (500ms)  | #088     | 88       | 0       | 0         |     490ms | HEALTHY (OK)  
   D4  | RS-485    | Poll (1200ms) | #037     | 37       | 0       | 0         |    1180ms | HEALTHY (OK)  
   ----------------------------------------------------------------------------------------------------
   BUS INTEGRITY REGS: RS-232 CRC_Errors: 0    Malformed: 0    | RS-485 CRC_Errors: 0    Malformed: 0   
   ----------------------------------------------------------------------------------------------------
   LATEST VALID BUFFERED RECORDS:
     [DEV 1]: "D1:T=26.4C,P=1013hPa"
     [DEV 2]: "D2_B3:PWR=30.2W,OK"
     [DEV 3]: "D3:RPM=1850,LD=45%"
     [DEV 4]: "D4:FLW=3.42L/m,TK=84%"
   ====================================================================================================
   ```
2. **ESP32-S3 16x2 LCD**:
   - Page 0: `D1:OK    D2:OK   ` / `D3:OK    D4:OK   `
   - Page 1: `D1: 26.4C 1013h` / `D2: 12.6V RUN  `
   - Page 2: `D3: 1850R LD45%` / `D4: 3.42L TK84%`
   - Page 3: `DROP:00  DUP:00 ` / `CRC:00   M2:GW  `

---

## 4. Fault Injection & Reliability Verification

All test cases can be triggered interactively by typing a single character in the **Arduino Mega Serial Monitor**:

### Test 1: Device Disconnect / Inactivity Timeout (`Press '1'`)
- **Action**: In the Mega Serial Monitor, type `1` and press Enter.
- **What happens**: Device 1 (RS-232 Continuous) immediately halts transmission.
- **Result on ESP32 Gateway**:
  - After 1500 ms, the Gateway flags: `! [FAULT DETECTED] DEV 1 Communication TIMEOUT! (Silent > 1500ms)`.
  - In the dashboard table, `D1` switches to `TIMEOUT (FAULT)`.
  - On the 16x2 LCD, Page 0 shows: `D1:TO!   D2:OK   `.
  - **FAULT ISOLATION PROOF**: Device 2, Device 3, and Device 4 continue receiving, polling, and displaying fresh telemetry without interruption!
  - Buzzer emits a single diagnostic beep every 5 seconds.

### Test 2: Automatic Recovery Without Restart (`Press '1' again`)
- **Action**: Type `1` again in the Mega Serial Monitor.
- **What happens**: Device 1 resumes transmitting.
- **Result on ESP32 Gateway**:
  - The Gateway immediately logs: `* [AUTO-RECOVERY] DEV 1 resumed communication! Automatically restored to HEALTHY.`
  - Health state transitions back to `HEALTHY (OK)`.
  - LCD returns to `D1:OK`.
  - **RECOVERY PROOF**: System recovered 100% autonomously without pressing reset or rebooting.

### Test 3: CRC Data Corruption (`Press '2'`)
- **Action**: In the Mega Serial Monitor, type `2` and press Enter.
- **What happens**: The next frame sent by Device 2 has its CRC byte deliberately inverted (`crc ^= 0xFF`).
- **Result on ESP32 Gateway**:
  - The frame parser rejects the frame before accepting it into the device buffer.
  - The `RS-232 CRC_Errors` counter ticks up by 1.
  - The latest valid data record remains protected and uncorrupted.

### Test 4: RS-485 Slave Failure & Bus Isolation (`Press '3'`)
- **Action**: In the Mega Serial Monitor, type `3` and press Enter.
- **What happens**: Device 3 stops replying to polling requests.
- **Result on ESP32 Gateway**:
  - Device 3 enters `TIMEOUT (FAULT)`.
  - **BUS ISOLATION PROOF**: The Gateway continues polling Device 4 every 1200 ms. Device 4 responds normally! The failure of one RS-485 multi-drop slave does NOT hang the bus or block communication with other slaves.

### Test 5: High-Rate Burst Flooding (`Press '4'`)
- **Action**: In the Mega Serial Monitor, type `4` and press Enter.
- **What happens**: Device 1 rapidly transmits 8 consecutive frames 20 ms apart.
- **Result on ESP32 Gateway**:
  - The non-blocking parser processes all 8 frames in rapid succession.
  - No buffer overflow occurs, and Device 3 and Device 4 continue their RS-485 polling seamlessly.

### Test 6: Missing / Dropped Frame Detection (`Press '5'`)
- **Action**: In the Mega Serial Monitor, type `5` and press Enter.
- **What happens**: Device 1 skips 5 sequence numbers.
- **Result on ESP32 Gateway**:
  - The Gateway logs: `! [INTEGRITY AUDIT] DEV 1 MISSING FRAME(S): Gap of 5 packets! (Expected #X, got #X+5)`.
  - The `DROPPED` column for `D1` increments by 5.

### Test 7: Duplicate / Repeated Frame Detection (`Press '6'`)
- **Action**: In the Mega Serial Monitor, type `6` and press Enter.
- **What happens**: Device 2 re-transmits its previous sequence number.
- **Result on ESP32 Gateway**:
  - The Gateway logs: `! [INTEGRITY AUDIT] DEV 2 DUPLICATE FRAME: Seq #X repeated!`.
  - The `DUPLICATE` counter increments by 1, and the duplicate packet is discarded.

### Return to Healthy State (`Press '0'`)
- Pressing `0` in the Mega Serial Monitor instantly clears all active faults and returns all 4 devices to 100% normal operation.
