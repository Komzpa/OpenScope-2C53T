#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["hidapi"]
# ///
"""
OpenScope 2C53T HID IAP Flash Tool

Flashes firmware to the device via USB HID when the bootloader is active.
The bootloader enumerates as VID:2E3C PID:AF01 "HID IAP".

Protocol: 64-byte HID reports
  IDLE  → START → ADDR+DATA (1KB chunks) → FINISH → JMP

Usage:
  uv run scripts/hid_flash.py firmware/build/firmware.bin
  ./scripts/hid_flash.py firmware/build/firmware.bin --no-jump
"""

import sys
import struct
import time
import glob
import os
import select

try:
    import hid
except ImportError:
    print("Error: hidapi not installed. Run: pip install hidapi")
    sys.exit(1)

VID = 0x2E3C
PID = 0xAF01
APP_ADDRESS = 0x08004000
APP_SETTINGS_ADDRESS = 0x080FF800
CHUNK_SIZE = 60       # data bytes per HID report (64 - 4 header)
BLOCK_SIZE = 1024     # bootloader buffers this much before programming
REPORT_SIZE = 64

# IAP commands
CMD_IDLE   = 0x5AA0
CMD_START  = 0x5AA1
CMD_ADDR   = 0x5AA2
CMD_DATA   = 0x5AA3
CMD_FINISH = 0x5AA4
CMD_CRC    = 0x5AA5
CMD_JMP    = 0x5AA6
CMD_GET    = 0x5AA7

ACK  = 0xFF00
NACK = 0x00FF


class HidrawBootloaderDevice:
    """Minimal hidraw transport for Linux hosts where hidapi open() fails."""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)

    def write(self, data):
        offset = 0
        while offset < len(data):
            _, writable, _ = select.select([], [self.fd], [], 5.0)
            if not writable:
                raise TimeoutError(f"hidraw write timeout on {self.path}")
            offset += os.write(self.fd, data[offset:])
        return offset

    def read(self, length, timeout_ms=5000):
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([self.fd], [], [], min(0.05, remaining))
            if not readable:
                continue
            try:
                return os.read(self.fd, length)
            except BlockingIOError:
                continue
        return []

    def get_manufacturer_string(self):
        return "Artery"

    def get_product_string(self):
        return "HID IAP (hidraw)"

    def close(self):
        os.close(self.fd)


def _candidate_hidraw_paths():
    patterns = [
        "/dev/input/by-id/usb-Artery_HID_IAP*-hidraw",
        "/dev/input/by-path/*-hidraw",
        "/dev/hidraw*",
    ]
    seen = set()
    for pattern in patterns:
        for raw_path in glob.glob(pattern):
            path = os.path.realpath(raw_path)
            if path in seen:
                continue
            seen.add(path)
            yield path


def _hidraw_matches_bootloader(path):
    try:
        name = os.path.basename(path)
        with open(f"/sys/class/hidraw/{name}/device/uevent", "r", encoding="ascii") as f:
            text = f.read()
    except OSError:
        return False
    return "HID_ID=0003:00002E3C:0000AF01" in text


def open_bootloader_device():
    dev = hid.device()
    try:
        dev.open(VID, PID)
        return dev
    except OSError as exc:
        last_error = exc

    for path in _candidate_hidraw_paths():
        if not _hidraw_matches_bootloader(path):
            continue
        try:
            return HidrawBootloaderDevice(path)
        except OSError as exc:
            last_error = exc

    raise last_error


def make_cmd(cmd, payload=b""):
    """Build a 64-byte HID report."""
    buf = struct.pack(">H", cmd) + payload
    return buf.ljust(REPORT_SIZE, b"\x00")


def send_recv(dev, cmd, payload=b"", expect_cmd=None):
    """Send command and wait for response."""
    report = make_cmd(cmd, payload)
    dev.write(b"\x00" + report)  # report ID 0 + 64 bytes

    resp = dev.read(REPORT_SIZE, timeout_ms=5000)
    if not resp:
        raise TimeoutError(f"No response for command 0x{cmd:04X}")

    resp_cmd = (resp[0] << 8) | resp[1]
    resp_result = (resp[2] << 8) | resp[3]

    if expect_cmd and resp_cmd != expect_cmd:
        raise RuntimeError(f"Unexpected response: cmd=0x{resp_cmd:04X}, expected 0x{expect_cmd:04X}")
    if resp_result == NACK:
        raise RuntimeError(f"NACK for command 0x{cmd:04X}")

    return resp


def flash_firmware(binpath, do_jump=True, app_address=APP_ADDRESS):
    """Flash a firmware binary to the device."""
    with open(binpath, "rb") as f:
        firmware = f.read()

    fw_size = len(firmware)
    print(f"Firmware: {binpath} ({fw_size} bytes)")

    # Pad to BLOCK_SIZE boundary
    pad = BLOCK_SIZE - (fw_size % BLOCK_SIZE)
    if pad < BLOCK_SIZE:
        firmware += b"\xFF" * pad

    if app_address < APP_ADDRESS or app_address + len(firmware) > APP_SETTINGS_ADDRESS:
        max_size = APP_SETTINGS_ADDRESS - app_address
        raise RuntimeError(
            f"refusing to flash {len(firmware)} padded bytes at 0x{app_address:08X}: "
            f"maximum app payload before settings sector is {max_size} bytes"
        )

    print(f"Padded to {len(firmware)} bytes ({len(firmware) // BLOCK_SIZE} blocks)")

    try:
        dev = open_bootloader_device()
    except OSError:
        print(f"Error: Cannot find device VID:0x{VID:04X} PID:0x{PID:04X}")
        print("Make sure the bootloader is running (Settings > Firmware Update, or no valid app)")
        sys.exit(1)

    print(f"Connected: {dev.get_manufacturer_string()} - {dev.get_product_string()}")

    try:
        # IDLE - reset state
        send_recv(dev, CMD_IDLE, expect_cmd=CMD_IDLE)
        print("IAP initialized")

        # START
        send_recv(dev, CMD_START, expect_cmd=CMD_START)
        print("Programming started")

        # Flash in 1KB blocks
        offset = 0
        total_blocks = len(firmware) // BLOCK_SIZE
        block_num = 0

        while offset < len(firmware):
            addr = app_address + offset

            # ADDR - set write address (triggers sector erase)
            addr_payload = struct.pack(">I", addr)
            send_recv(dev, CMD_ADDR, addr_payload, expect_cmd=CMD_ADDR)

            # DATA - send 1KB in CHUNK_SIZE-byte pieces, then wait for ACK
            block_end = offset + BLOCK_SIZE
            pos = offset
            while pos < block_end:
                chunk = firmware[pos:min(pos + CHUNK_SIZE, block_end)]
                data_payload = struct.pack(">H", len(chunk)) + chunk
                dev.write(b"\x00" + make_cmd(CMD_DATA, data_payload))
                pos += len(chunk)
                time.sleep(0.002)

            # Wait for ACK (bootloader sends after 1KB programmed to flash)
            resp = dev.read(REPORT_SIZE, timeout_ms=10000)
            if not resp:
                raise TimeoutError(f"No ACK after block at 0x{addr:08X}")
            resp_result = (resp[2] << 8) | resp[3]
            if resp_result == NACK:
                raise RuntimeError(f"NACK writing block at 0x{addr:08X}")

            offset = block_end
            block_num += 1

            # Progress bar
            pct = block_num * 100 // total_blocks
            bar = "#" * (pct // 2) + "-" * (50 - pct // 2)
            print(f"\r  [{bar}] {pct:3d}% ({block_num}/{total_blocks})", end="", flush=True)

        print()  # newline after progress bar

        # FINISH - set upgrade flag
        send_recv(dev, CMD_FINISH, expect_cmd=CMD_FINISH)
        print("Upgrade flag set")

        if do_jump:
            # JMP - jump to app
            send_recv(dev, CMD_JMP, expect_cmd=CMD_JMP)
            print("Jumping to application...")
            time.sleep(0.5)
        else:
            print("Flash complete (no jump requested)")

    finally:
        dev.close()

    print("Done!")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="OpenScope 2C53T HID IAP Flash Tool")
    parser.add_argument("firmware", help="Path to firmware .bin file")
    parser.add_argument("--no-jump", action="store_true",
                        help="Don't jump to app after flashing")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=APP_ADDRESS,
                        help=f"App start address (default: 0x{APP_ADDRESS:08X})")
    args = parser.parse_args()

    flash_firmware(args.firmware, do_jump=not args.no_jump, app_address=args.address)


if __name__ == "__main__":
    main()
