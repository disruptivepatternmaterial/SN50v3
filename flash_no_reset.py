#!/usr/bin/env python3
"""
flash_no_reset.py — Flash SN50v3 firmware without DTR/RTS auto-reset.

Designed for 3-wire UART connections (TX, RX, GND only) where the stock
tremo_loader.py cannot auto-reset the device into bootloader mode.

Strategy:
  1. Sync at 921600 baud (small packets work over jumper wires)
  2. Switch device to 115200 baud (reliable for larger packets)
  3. Erase, flash, verify, reboot at 115200

Usage:
  1. Set BOOT switch to ISP on the SN50v3 PCB
  2. Run: python3 flash_no_reset.py
  3. Press RESET when prompted
  4. After success, set BOOT switch to FLASH and press RESET

Requires: pyserial (pip3 install pyserial)
"""
import serial
import time
import struct
import zlib
import os
import sys
import glob

INITIAL_BAUD = 921600
FLASH_BAUD = 115200
FLASH_ADDR = 0x0800D000
FIRMWARE = 'Projects/Applications/DRAGINO-LRWAN-AT/Make_out/DRAGINO-LRWAN-AT.bin'
SYNC_TIMEOUT = 60
CHUNK_SIZE = 256


def find_serial_port():
    """Auto-detect USB serial port."""
    patterns = ['/dev/tty.usbserial-*', '/dev/ttyUSB*', '/dev/ttyACM*']
    for pat in patterns:
        matches = glob.glob(pat)
        if matches:
            return matches[0]
    return None


def make_pkt(cmd, data=b""):
    p = struct.pack('<BBH', 0xFE, cmd, len(data)) + data
    c = zlib.crc32(p) & 0xFFFFFFFF
    return p + struct.pack('<IB', c, 0xEF)


CMD_SYNC = 1
CMD_FLASH = 3
CMD_ERASE = 4
CMD_VERIFY = 5
CMD_REBOOT = 12
CMD_BAUDRATE = 16


def main():
    port = find_serial_port()
    if not port:
        print('No USB serial port found.')
        sys.exit(1)
    print('Using port: %s' % port, flush=True)

    ser = serial.Serial(port, INITIAL_BAUD, timeout=5)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(0.05)

    if not os.path.isfile(FIRMWARE):
        print('Firmware not found: %s' % FIRMWARE)
        print('Run "make EXECUTABLE_SUFFIX=" first.')
        ser.close()
        sys.exit(1)

    # --- Sync at 921600 ---
    print('Waiting for sync — press RESET now (timeout %ds)...' % SYNC_TIMEOUT, flush=True)
    synced = False
    t0 = time.time()
    while time.time() - t0 < SYNC_TIMEOUT:
        ser.flushInput()
        ser.write(make_pkt(CMD_SYNC))
        time.sleep(0.4)
        n = ser.in_waiting
        if n >= 9:
            raw = ser.read(n)
            if raw[0] == 0xFE:
                synced = True
                print('SYNCED at %d baud (%.1fs).' % (INITIAL_BAUD, time.time() - t0), flush=True)
                break
        time.sleep(0.1)

    if not synced:
        print('No sync. Check: BOOT switch on ISP, press RESET, wiring (Orange→RX, Yellow→TX, Black→GND).')
        ser.close()
        sys.exit(1)

    # --- Switch to lower baud for reliable transfer ---
    print('Switching to %d baud...' % FLASH_BAUD, flush=True)
    ser.flushInput()
    ser.write(make_pkt(CMD_BAUDRATE, struct.pack('<I', FLASH_BAUD)))
    time.sleep(0.5)
    ser.baudrate = FLASH_BAUD
    time.sleep(0.3)
    ser.flushInput()

    # Re-sync at new baud
    ok = False
    for _ in range(10):
        ser.flushInput()
        ser.write(make_pkt(CMD_SYNC))
        time.sleep(0.4)
        n = ser.in_waiting
        if n >= 9:
            raw = ser.read(n)
            if raw[0] == 0xFE:
                ok = True
                print('Re-synced at %d baud.' % FLASH_BAUD, flush=True)
                break
        time.sleep(0.1)

    if not ok:
        print('Failed to re-sync at %d. Try again.' % FLASH_BAUD)
        ser.close()
        sys.exit(1)

    # --- Load firmware ---
    sz = os.path.getsize(FIRMWARE)
    with open(FIRMWARE, 'rb') as f:
        img = f.read()
    crc = zlib.crc32(img) & 0xFFFFFFFF
    print('Firmware: %d bytes, CRC: 0x%08X' % (sz, crc), flush=True)

    # --- Erase ---
    print('Erasing...', flush=True)
    ser.flushInput()
    ser.write(make_pkt(CMD_ERASE, struct.pack('<II', FLASH_ADDR, sz)))
    ser.timeout = 60
    raw = ser.read(9)
    if len(raw) < 4 or raw[0] != 0xFE:
        print('Erase failed (got %d bytes).' % len(raw))
        ser.close()
        sys.exit(1)
    print('Erased.', flush=True)

    # --- Flash ---
    print('Flashing (%d-byte chunks)...' % CHUNK_SIZE, flush=True)
    off = 0
    fa = FLASH_ADDR
    last_pct = -1
    while off < sz:
        ch = img[off:off + CHUNK_SIZE]
        ser.flushInput()
        ser.write(make_pkt(CMD_FLASH, struct.pack('<II', fa, len(ch)) + ch))
        ser.timeout = 5
        raw = ser.read(9)
        if len(raw) < 4 or raw[0] != 0xFE:
            print('Flash failed at 0x%X.' % fa)
            ser.close()
            sys.exit(1)
        fa += len(ch)
        off += len(ch)
        pct = int(100 * off / sz)
        if pct >= last_pct + 10 or off == sz:
            print('  %d%%' % pct, flush=True)
            last_pct = pct

    # --- Verify ---
    print('Verifying...', flush=True)
    ser.flushInput()
    ser.write(make_pkt(CMD_VERIFY, struct.pack('<III', FLASH_ADDR, sz, crc)))
    ser.timeout = 15
    raw = ser.read(9)
    if len(raw) < 4 or raw[0] != 0xFE or raw[1] != 0:
        print('Verify failed.')
        ser.close()
        sys.exit(1)
    print('Verified OK!', flush=True)

    # --- Reboot ---
    ser.write(make_pkt(CMD_REBOOT, struct.pack('B', 0)))
    time.sleep(1)
    ser.close()
    print('=== SUCCESS — Firmware flashed! ===', flush=True)
    print('Now: set BOOT switch to FLASH, press RESET.')


if __name__ == '__main__':
    main()
