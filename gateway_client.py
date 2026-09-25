#!/usr/bin/env python3
"""
===============================================================================
Project: ESP32-S3 Multi-Device Serial-to-Wi-Fi Gateway (Milestone 3 Validation)
Component: Remote UDP Inspection & Command Client
Protocol: Binary Request/Response (UDP Port 5000)
Target: ESP32-S3 Gateway running FreeRTOS UDP Task on Core 0
===============================================================================
"""

import socket
import struct
import sys
import time

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

# =============================================================================
# ANSI COLOR FORMATTING FOR TERMINAL OUTPUT
# =============================================================================
CLR_RESET   = "\033[0m"
CLR_BOLD    = "\033[1m"
CLR_DIM     = "\033[2m"
CLR_RED     = "\033[91m"
CLR_GREEN   = "\033[92m"
CLR_YELLOW  = "\033[93m"
CLR_BLUE    = "\033[94m"
CLR_MAGENTA = "\033[95m"
CLR_CYAN    = "\033[96m"
CLR_WHITE   = "\033[97m"

# =============================================================================
# PROTOCOL DEFINITIONS & CONSTANTS
# =============================================================================
GATEWAY_UDP_PORT   = 5000
SOCKET_TIMEOUT_SEC = 2.0

# Commands
CMD_GET_SINGLE_DEVICE = 0x01
CMD_GET_ALL_DEVICES   = 0x02
CMD_GET_HEALTH_STATUS = 0x03
CMD_INVALID_TEST      = 0xEE  # Deliberate invalid command code for testing rejection

# Status / Error Codes
STATUS_OK                 = 0x00
ERR_INVALID_CMD           = 0x01
ERR_INVALID_DEVICE_ID     = 0x02
ERR_DEVICE_OFFLINE        = 0x03
ERR_MUTEX_TIMEOUT         = 0x04
ERR_MALFORMED_PACKET      = 0x05

ERROR_CODE_MAP = {
    ERR_INVALID_CMD:       "ERR_INVALID_CMD (Command Code Not Recognized)",
    ERR_INVALID_DEVICE_ID: "ERR_INVALID_DEVICE_ID (Target Device ID Out of Bounds 1-4)",
    ERR_DEVICE_OFFLINE:    "ERR_DEVICE_OFFLINE (Device Flagged Offline / Ingestion Stalled)",
    ERR_MUTEX_TIMEOUT:     "ERR_MUTEX_TIMEOUT (Core 0 Timed Out Waiting for Shared Memory Mutex)",
    ERR_MALFORMED_PACKET:  "ERR_MALFORMED_PACKET (Packet Failed Length Boundary Check)"
}

# =============================================================================
# FORMATTING & DISPLAY HELPERS
# =============================================================================
def print_banner():
    banner = f"""
{CLR_CYAN}{CLR_BOLD}╔═════════════════════════════════════════════════════════════════════════════╗
║          ESP32-S3 MULTI-DEVICE SERIAL-TO-WI-FI UDP GATEWAY CLIENT          ║
║                      Milestone 3 Diagnostic & Validation Suite              ║
╚═════════════════════════════════════════════════════════════════════════════╝{CLR_RESET}
    """
    print(banner)

def hex_dump(data: bytes, prefix="       "):
    """Formats raw bytes as a side-by-side hex and ASCII dump for embedded inspection."""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join((chr(b) if 32 <= b <= 126 else ".") for b in chunk)
        lines.append(f"{prefix}{i:04X}  {hex_str:<48}  |{ascii_str}|")
    return "\n".join(lines)

def prompt_gateway_ip() -> str:
    """Prompts and validates the target ESP32-S3 IP address."""
    if len(sys.argv) > 1:
        arg_ip = sys.argv[1].strip()
        try:
            socket.inet_aton(arg_ip)
            print(f"{CLR_GREEN}Using Gateway IP from command line argument: {CLR_BOLD}{arg_ip}{CLR_RESET}")
            return arg_ip
        except socket.error:
            pass

    while True:
        try:
            user_input = input(f"{CLR_BOLD}Enter ESP32-S3 Gateway IP address{CLR_RESET} (e.g. 192.168.1.100): ").strip()
            if not user_input:
                print(f"{CLR_RED}IP address cannot be empty.{CLR_RESET}")
                continue
            # Validate IP format
            socket.inet_aton(user_input)
            return user_input
        except socket.error:
            print(f"{CLR_RED}Invalid IPv4 format. Please enter a valid address (e.g., 192.168.4.1).{CLR_RESET}")
        except KeyboardInterrupt:
            print(f"\n{CLR_YELLOW}Session aborted by user.{CLR_RESET}")
            sys.exit(0)

# =============================================================================
# NETWORK TRANSACTION ENGINE
# =============================================================================
def send_request(sock: socket.socket, target_ip: str, cmd: int, device_id: int):
    """
    Packs and transmits a 2-byte command packet, waits for a response with a 
    2.0s hard timeout, and dispatches the payload to the appropriate parser.
    """
    # Protocol Request Frame: [CMD (1B)][Target Dev ID (1B)]
    req_packet = struct.pack("!BB", cmd, device_id)

    print(f"\n{CLR_DIM}-------------------------------------------------------------------------------{CLR_RESET}")
    print(f"{CLR_BLUE}TX -> [{target_ip}:{GATEWAY_UDP_PORT}]{CLR_RESET} CMD: {CLR_BOLD}0x{cmd:02X}{CLR_RESET} | DeviceID: {CLR_BOLD}0x{device_id:02X}{CLR_RESET} ({len(req_packet)} bytes)")
    
    start_time = time.perf_counter()
    try:
        sock.sendto(req_packet, (target_ip, GATEWAY_UDP_PORT))
        raw_rx, server_addr = sock.recvfrom(1024)
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        print(f"{CLR_GREEN}RX <- [{server_addr[0]}:{server_addr[1]}]{CLR_RESET} Received {len(raw_rx)} bytes in {CLR_BOLD}{elapsed_ms:.2f} ms{CLR_RESET}")
        print(f"{CLR_DIM}Raw Hex Payload:\n{hex_dump(raw_rx)}{CLR_RESET}")
        
        # Parse and display unpacked data
        parse_response(raw_rx)

    except socket.timeout:
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        print(f"{CLR_RED}{CLR_BOLD}⚠️  [TIMEOUT / PACKET LOST]{CLR_RESET} No response from gateway after {elapsed_ms:.1f} ms.")
        print(f"{CLR_YELLOW}   Check: (1) Is ESP32-S3 connected to Wi-Fi? (2) Verify IP address: {target_ip} (3) Port 5000 reachable?{CLR_RESET}")
    except Exception as e:
        print(f"{CLR_RED}⚠️  Socket Exception: {e}{CLR_RESET}")

# =============================================================================
# PROTOCOL RESPONSE PARSERS
# =============================================================================
def parse_response(data: bytes):
    """Dispatches binary response frame according to the Milestone 3 specification."""
    if len(data) < 2:
        print(f"{CLR_RED}❌ Error: Response packet too short ({len(data)} bytes). Corrupted frame.{CLR_RESET}")
        return

    resp_cmd, status = struct.unpack("!BB", data[:2])

    # Check for error status
    if status != STATUS_OK:
        parse_error_response(resp_cmd, status, data)
        return

    # Route to appropriate payload parser
    if resp_cmd == CMD_GET_SINGLE_DEVICE:
        parse_single_device_response(data)
    elif resp_cmd == CMD_GET_ALL_DEVICES:
        parse_all_devices_response(data)
    elif resp_cmd == CMD_GET_HEALTH_STATUS:
        parse_health_status_response(data)
    else:
        print(f"{CLR_YELLOW}⚠️ Received OK status with unrecognized command echo: 0x{resp_cmd:02X}{CLR_RESET}")

def parse_error_response(resp_cmd: int, status_code: int, data: bytes):
    """Parses and renders an explicit Error Response packet."""
    target_id = data[2] if len(data) > 2 else 0x00
    err_str = "None"
    if len(data) > 3:
        err_str = data[3:].decode("ascii", errors="replace").rstrip("\x00")

    err_name = ERROR_CODE_MAP.get(status_code, "UNKNOWN_ERROR_CODE")

    print(f"\n{CLR_RED}{CLR_BOLD}╔══════════════════════════ GATEWAY ERROR REJECTION ══════════════════════════╗{CLR_RESET}")
    print(f"{CLR_RED}║ Command Echo     :{CLR_RESET} 0x{resp_cmd:02X}")
    print(f"{CLR_RED}║ Error Code       :{CLR_RESET} 0x{status_code:02X} -> {CLR_BOLD}{err_name}{CLR_RESET}")
    print(f"{CLR_RED}║ Target Device ID :{CLR_RESET} 0x{target_id:02X} ({target_id})")
    print(f"{CLR_RED}║ Gateway Message  :{CLR_RESET} \"{err_str}\"")
    print(f"{CLR_RED}{CLR_BOLD}╚═════════════════════════════════════════════════════════════════════════════╝{CLR_RESET}")

def parse_single_device_response(data: bytes):
    """
    Unpacks Single Device Data (CMD 0x01).
    Frame: [CMD(1B)][STATUS(1B)][DevID(1B)][Len(2B BE)][Payload(Len Bytes)]
    """
    if len(data) < 5:
        print(f"{CLR_RED}❌ Error: Truncated Single Device frame ({len(data)} bytes).{CLR_RESET}")
        return

    dev_id, p_len = struct.unpack("!BH", data[2:5])
    raw_payload = data[5:5+p_len]
    payload_str = raw_payload.decode("utf-8", errors="replace")

    print(f"\n{CLR_GREEN}{CLR_BOLD}┌──────────────────────── SINGLE DEVICE TELEMETRY ────────────────────────────┐{CLR_RESET}")
    print(f"│ {CLR_BOLD}Device ID       :{CLR_RESET} {dev_id:02d} (0x{dev_id:02X})")
    print(f"│ {CLR_BOLD}Status          :{CLR_RESET} {CLR_GREEN}{CLR_BOLD}● ONLINE{CLR_RESET}")
    print(f"│ {CLR_BOLD}Payload Length  :{CLR_RESET} {p_len} bytes")
    print(f"│ {CLR_BOLD}Payload Data    :{CLR_RESET} {CLR_CYAN}{payload_str}{CLR_RESET}")
    print(f"{CLR_GREEN}{CLR_BOLD}└─────────────────────────────────────────────────────────────────────────────┘{CLR_RESET}")

def parse_all_devices_response(data: bytes):
    """
    Unpacks All Devices Data (CMD 0x02).
    Frame: [CMD(1B)][STATUS(1B)][DeviceCount(1B)]
    Followed by per-device: [DevID(1B)][Online(1B)][Len(2B BE)][Payload(Len B)]
    """
    if len(data) < 3:
        print(f"{CLR_RED}❌ Error: Truncated All-Devices frame header.{CLR_RESET}")
        return

    device_count = data[2]
    offset = 3

    print(f"\n{CLR_CYAN}{CLR_BOLD}╔══════════════════════════ ALL DEVICES TELEMETRY SNAPSHOT ═══════════════════════════╗{CLR_RESET}")
    print(f"{CLR_CYAN}║ Total Registered Devices in Gateway: {device_count}{CLR_RESET}")
    print(f"{CLR_CYAN}╟──────┬─────────┬──────────────┬─────────────────────────────────────────────────────╢{CLR_RESET}")
    print(f"{CLR_CYAN}║ {CLR_BOLD}ID{CLR_RESET}   │ {CLR_BOLD}STATUS{CLR_RESET}  │ {CLR_BOLD}LENGTH (B){CLR_RESET}   │ {CLR_BOLD}PAYLOAD DATA{CLR_RESET}                                        {CLR_CYAN}║{CLR_RESET}")
    print(f"{CLR_CYAN}╟──────┼─────────┼──────────────┼─────────────────────────────────────────────────────╢{CLR_RESET}")

    for _ in range(device_count):
        if offset + 4 > len(data):
            print(f"{CLR_RED}║ Error: Unexpected end of buffer while unpacking device records.             ║{CLR_RESET}")
            break

        dev_id, is_online, p_len = struct.unpack("!BBH", data[offset:offset+4])
        offset += 4

        raw_payload = data[offset:offset+p_len]
        offset += p_len
        payload_str = raw_payload.decode("utf-8", errors="replace")

        status_badge = f"{CLR_GREEN}ONLINE {CLR_RESET}" if is_online else f"{CLR_RED}OFFLINE{CLR_RESET}"
        display_payload = payload_str if is_online else f"{CLR_DIM}[NO DATA - SENSOR UNREACHABLE]{CLR_RESET}"

        print(f"║ 0x{dev_id:02X} │ {status_badge} │ {p_len:<12} │ {display_payload:<51} ║")

    print(f"{CLR_CYAN}{CLR_BOLD}╚══════╧═════════╧══════════════╧═════════════════════════════════════════════════════╝{CLR_RESET}")

def parse_health_status_response(data: bytes):
    """
    Unpacks Health/Communication Status (CMD 0x03).
    Frame: [CMD(1B)][STATUS(1B)][DeviceCount(1B)]
    Followed by per-device: [DevID(1B)][Online(1B)][Timestamp_ms(4B BE)]
                             [CRCErrCount(2B BE)][TimeoutCount(2B BE)][OutOfOrderCount(2B BE)]
    """
    if len(data) < 3:
        print(f"{CLR_RED}❌ Error: Truncated Health Status frame header.{CLR_RESET}")
        return

    device_count = data[2]
    offset = 3

    print(f"\n{CLR_MAGENTA}{CLR_BOLD}╔═══════════════════════════════ SYSTEM HEALTH & DIAGNOSTICS ══════════════════════════════╗{CLR_RESET}")
    print(f"{CLR_MAGENTA}║ Total Supervised Channels: {device_count}{CLR_RESET}")
    print(f"{CLR_MAGENTA}╟──────┬──────────────┬───────────────────┬───────────┬───────────┬───────────────────────╢{CLR_RESET}")
    print(f"{CLR_MAGENTA}║ {CLR_BOLD}ID{CLR_RESET}   │ {CLR_BOLD}STATE{CLR_RESET}        │ {CLR_BOLD}LAST SEEN (ms){CLR_RESET}    │ {CLR_BOLD}CRC ERR{CLR_RESET}   │ {CLR_BOLD}TIMEOUT{CLR_RESET}   │ {CLR_BOLD}OUT-OF-ORDER{CLR_RESET}          {CLR_MAGENTA}║{CLR_RESET}")
    print(f"{CLR_MAGENTA}╟──────┼──────────────┼───────────────────┼───────────┼───────────┼───────────────────────╢{CLR_RESET}")

    for _ in range(device_count):
        if offset + 12 > len(data):
            print(f"{CLR_RED}║ Error: Unexpected buffer boundary in health frame.                                        ║{CLR_RESET}")
            break

        dev_id, is_online, ts_ms, crc_err, timeout_ct, ooo_ct = struct.unpack(
            "!BBIHHH", data[offset:offset+12])
        offset += 12

        state_badge = f"{CLR_GREEN}{CLR_BOLD}● HEALTHY{CLR_RESET}  " if is_online else f"{CLR_RED}{CLR_BOLD}✖ FAULT    {CLR_RESET}"

        print(f"║ 0x{dev_id:02X} │ {state_badge} │ {ts_ms:>15} │ {crc_err:<9} │ {timeout_ct:<9} │ {ooo_ct:<12}          ║")

    print(f"{CLR_MAGENTA}{CLR_BOLD}╚══════╧══════════════╧═══════════════════╧═══════════╧═══════════╧═══════════════════════╝{CLR_RESET}")

# =============================================================================
# INTERACTIVE TERMINAL MENU
# =============================================================================
def main():
    print_banner()
    gateway_ip = prompt_gateway_ip()

    # Create UDP client socket configured with strict 2.0s non-blocking timeout
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(SOCKET_TIMEOUT_SEC)

    print(f"\n{CLR_GREEN}✓ Socket initialized with {SOCKET_TIMEOUT_SEC}s timeout.{CLR_RESET}")
    print(f"{CLR_GREEN}✓ Target configured: {gateway_ip}:{GATEWAY_UDP_PORT}{CLR_RESET}")

    while True:
        try:
            print(f"\n{CLR_BOLD}══════════════════════════ TEST SELECTION MENU ══════════════════════════{CLR_RESET}")
            print(f"  {CLR_CYAN}[1]{CLR_RESET} Query Single Device Telemetry  (CMD 0x01: Dev 1, 2, 3, or 4)")
            print(f"  {CLR_CYAN}[2]{CLR_RESET} Query ALL Devices Telemetry    (CMD 0x02: Atomic 4-Channel Dump)")
            print(f"  {CLR_CYAN}[3]{CLR_RESET} Query System Health / Status   (CMD 0x03: Online States & Timestamps)")
            print(f"  {CLR_CYAN}[4]{CLR_RESET} Test Error Rejection           (CMD 0xEE: Invalid Command Injection)")
            print(f"  {CLR_CYAN}[5]{CLR_RESET} Test Out-of-Bounds Device ID   (CMD 0x01: Request Invalid Dev ID 99)")
            print(f"  {CLR_CYAN}[6]{CLR_RESET} Change Target Gateway IP")
            print(f"  {CLR_CYAN}[0]{CLR_RESET} Exit")
            print(f"{CLR_BOLD}─────────────────────────────────────────────────────────────────────────{CLR_RESET}")

            choice = input(f"{CLR_BOLD}Select Option [0-6]: {CLR_RESET}").strip()

            if choice == "1":
                dev_input = input(f"Enter Target Device ID (1-4, note: Dev 4 tests offline state): ").strip()
                try:
                    dev_id = int(dev_input)
                    send_request(sock, gateway_ip, CMD_GET_SINGLE_DEVICE, dev_id)
                except ValueError:
                    print(f"{CLR_RED}Invalid input. Please enter an integer (1-4).{CLR_RESET}")

            elif choice == "2":
                send_request(sock, gateway_ip, CMD_GET_ALL_DEVICES, 0x00)

            elif choice == "3":
                send_request(sock, gateway_ip, CMD_GET_HEALTH_STATUS, 0x00)

            elif choice == "4":
                print(f"{CLR_YELLOW}Sending intentionally invalid Command Byte (0xEE) to verify rejection...{CLR_RESET}")
                send_request(sock, gateway_ip, CMD_INVALID_TEST, 0x01)

            elif choice == "5":
                print(f"{CLR_YELLOW}Sending out-of-bounds Device ID (0x63 / 99) on CMD 0x01 to verify boundary checks...{CLR_RESET}")
                send_request(sock, gateway_ip, CMD_GET_SINGLE_DEVICE, 99)

            elif choice == "6":
                gateway_ip = prompt_gateway_ip()
                print(f"{CLR_GREEN}✓ Target updated to: {gateway_ip}:{GATEWAY_UDP_PORT}{CLR_RESET}")

            elif choice in ("0", "q", "exit"):
                print(f"\n{CLR_CYAN}Closing UDP socket and exiting.{CLR_RESET}")
                sock.close()
                sys.exit(0)

            else:
                print(f"{CLR_RED}Invalid option '{choice}'. Please select between 0 and 6.{CLR_RESET}")

        except KeyboardInterrupt:
            print(f"\n\n{CLR_YELLOW}Interrupt received. Exiting gracefully...{CLR_RESET}")
            sock.close()
            sys.exit(0)

if __name__ == "__main__":
    main()
