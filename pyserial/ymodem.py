#!/usr/bin/env python3
"""
YMODEM File Sender for STM32 Bootloader
Correct termination: 2x EOT + final empty header
"""

import serial
import time
import sys
import os

# ---------------- YMODEM CONSTANTS ----------------
SOH = 0x01        # 128-byte packet
STX = 0x02        # 1024-byte packet
EOT = 0x04
ACK = 0x06
NAK = 0x15
CAN = 0x18
CRC_REQ = ord('C')

# -------------------------------------------------

def fmt_resp(resp):
    return f"0x{resp:02X}" if resp is not None else "None"

def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def make_packet(block_num, data, use_1k=False):
    packet_size = 1024 if use_1k else 128
    data = data + b'\x00' * (packet_size - len(data))

    header = STX if use_1k else SOH
    pkt = bytes([
        header,
        block_num & 0xFF,
        (~block_num) & 0xFF
    ])

    pkt += data
    crc = crc16(data)
    pkt += bytes([crc >> 8, crc & 0xFF])

    return pkt

def wait_for_response(ser, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            return ser.read(1)[0]
    return None

def send_file_ymodem(port, baudrate, filename):

    if not os.path.exists(filename):
        print("❌ File not found")
        return False

    filesize = os.path.getsize(filename)
    basename = os.path.basename(filename)

    print(f"Opening {port} @ {baudrate}")
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )

    time.sleep(1)

    # Drain boot messages
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode(errors="ignore"))

    # ---------------- WAIT FOR 'C' ----------------
    print("Waiting for receiver 'C'...")
    start = time.time()
    while time.time() - start < 60:
        b = wait_for_response(ser, 1)
        if b == CRC_REQ:
            print("Receiver ready (CRC mode)")
            break
    else:
        print("❌ No 'C' received")
        ser.close()
        return False

    # ---------------- SEND HEADER (BLOCK 0) ----------------
    # ---------------- SEND HEADER (BLOCK 0) ----------------
    print("Sending header packet...")
    print(f"  Filename: {basename}")
    print(f"  Size    : {filesize} bytes")

    header_data = basename.encode() + b'\x00'
    header_data += str(filesize).encode() + b'\x00'

    pkt = make_packet(0, header_data, use_1k=False)

    for attempt in range(10):
        ser.write(pkt)
        response = wait_for_response(ser)

        if response == ACK:
            print("Header ACK received")
            response = wait_for_response(ser)
            if response == CRC_REQ:
                print("Receiver ready for data")
                break
            else:
                print(f"Unexpected after ACK: {fmt_resp(response)}")

        elif response == CRC_REQ:
            print("Header accepted (ACK implied by 'C')")
            break

        elif response == NAK:
            print("Header NAK, retrying...")
            continue

        else:
            print(f"Header error: {fmt_resp(response)}")

    else:
        print("❌ Header failed after retries")
        ser.close()
        return False


    # ---------------- SEND DATA BLOCKS ----------------
    print("Sending data blocks...")
    with open(filename, "rb") as f:
        block = 1
        sent = 0

        while True:
            data = f.read(1024)
            if not data:
                break

            use_1k = len(data) > 128
            pkt = make_packet(block, data, use_1k)

            for _ in range(10):
                ser.write(pkt)
                resp = wait_for_response(ser)

                if resp == ACK:
                    sent += len(data)
                    print(f"  Block {block} OK ({sent}/{filesize})")
                    block = (block + 1) & 0xFF
                    break
            else:
                print("❌ Data block failed")
                ser.close()
                return False

    # ---------------- EOT HANDSHAKE ----------------
    print("Sending EOT...")
    ser.write(bytes([EOT]))
    resp = wait_for_response(ser)
    if resp != NAK:
        print(f"Warning: Expected NAK, got {fmt_resp(resp)}")

    ser.write(bytes([EOT]))
    resp = wait_for_response(ser)
    if resp != ACK:
        print(f"❌ Expected ACK after EOT, got {fmt_resp(resp)}")
        ser.close()
        return False

    # ---------------- FINAL EMPTY HEADER ----------------
    resp = wait_for_response(ser)
    if resp != CRC_REQ:
        print(f"❌ Expected 'C' before final header, got {fmt_resp(resp)}")
        ser.close()
        return False

    print("Sending final empty header...")
    empty_pkt = make_packet(0, b'', use_1k=False)
    ser.write(empty_pkt)

    resp = wait_for_response(ser)
    if resp != ACK:
        print(f"❌ Final ACK missing, got {fmt_resp(resp)}")
        ser.close()
        return False

    print("\n✅ YMODEM transfer complete")
    ser.close()
    return True

# -------------------------------------------------

def main():
    port = "/dev/ttyUSB0"
    baudrate = 115200
    filename = "blinky.bin"

    if len(sys.argv) == 4:
        port = sys.argv[1]
        baudrate = int(sys.argv[2])
        filename = sys.argv[3]

    print("=" * 50)
    print("STM32 YMODEM Sender")
    print("=" * 50)

    if send_file_ymodem(port, baudrate, filename):
        print("SUCCESS")
    else:
        print("FAILED")

if __name__ == "__main__":
    main()
