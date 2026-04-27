# SN50v3 Custom Firmware: SHT45 Support

## Overview

Custom firmware for the Dragino SN50v3-LB that adds Sensirion SHT45 (SHT4x family)
temperature and humidity sensor support over I2C. The SHT45 is auto-detected at
startup alongside the existing SHT20/SHT31 support. Payload format is unchanged —
temperature and humidity occupy the same `temp_sht` / `hum_sht` slots used by
SHT20/SHT31, so existing TTN decoders work without modification.

## What Was Changed

### Firmware files modified

| File | Change |
|---|---|
| `Drivers/sensor/I2C_sensor.h` | Added `check_sht45_connect()` and `SHT45_Read()` declarations |
| `Drivers/sensor/I2C_sensor.c` | Implemented SHT45 probe (I2C addr `0x44`, serial number read with CRC-8 validation), high-precision measurement (`0xFD` command), and value conversion. Added `flag_temp==4` dispatch in `I2C_read_data()` |
| `Drivers/sensor/bsp.c` | Added SHT45 detection in `BSP_sensor_Init()` (probe with `flags=4` after SHT31, before BH1750) |
| `Projects/Applications/DRAGINO-LRWAN-AT/inc/lora_config.h` | Added SHT3x HAL macro aliases (`SDA_OPEN`, `SDA_LOW`, `SCL_OPEN`, `SCL_LOW`, `SDA_READ`, `SCL_READ`) to fix pre-existing build errors in `I2C_A.c` |
| `Projects/Applications/DRAGINO-LRWAN-AT/Makefile` | Region define (change `REGION_EU868` → `REGION_US915` etc. as needed) |

### No changes needed

- `main.c` — `temp_sht` and `hum_sht` are already packed into the uplink payload as 16-bit big-endian values scaled by 10
- `Makefile` wildcard — automatically picks up all `.c` files in `Drivers/sensor/`
- TTN payload decoder — same byte positions as SHT20/SHT31

## Building

### Prerequisites (macOS)

```bash
# Install ARM GCC toolchain (complete, not the Homebrew formula)
brew tap osx-cross/arm
brew install arm-gcc-bin

# Install Python serial library
pip3 install pyserial
```

### Compile

```bash
cd /path/to/SN50v3
source build/envsetup.sh
cd Projects/Applications/DRAGINO-LRWAN-AT
make clean
make EXECUTABLE_SUFFIX=
```

The `EXECUTABLE_SUFFIX=` override is needed on macOS because `common.mk` incorrectly
sets `.exe` suffix for non-Linux platforms.

Output: `Make_out/DRAGINO-LRWAN-AT.bin`

### Changing Region

Edit `Projects/Applications/DRAGINO-LRWAN-AT/Makefile`, line with `$(PROJECT)_DEFINES`:

| Region | Define |
|---|---|
| US 902-928 MHz | `-DREGION_US915` |
| EU 863-870 MHz | `-DREGION_EU868` |
| AU 915-928 MHz | `-DREGION_AU915` |
| AS 923 MHz | `-DREGION_AS923` |
| KR 920-923 MHz | `-DREGION_KR920` |
| IN 865-867 MHz | `-DREGION_IN865` |

## Flashing via UART

### Hardware needed

- USB-to-UART adapter (3.3V logic, e.g. Adafruit FTDI Friend or FTDI cable)
- DuPont jumper wires (female-to-female)

### SN50v3 PCB internals

Open the case (4 screws). On the PCB you will find:

1. **3-pin UART header** (labeled GND, TX, RX) — near the main chip
2. **BOOT slide switch** — labeled ISP/FLASH (or PROG/FLASH)
3. **Reset button** — small tactile button labeled RST

### Wiring (Adafruit FTDI cable colors)

| Adapter wire | Color | Board pin |
|---|---|---|
| TXD (adapter output) | Orange | RX |
| RXD (adapter input) | Yellow | TX |
| GND | Black | GND |

Do NOT connect VCC/Red — the battery provides power. Only 3 wires needed.

### Flash procedure

The stock `tremo_loader.py` uses DTR/RTS for auto-reset, which doesn't work with a
3-wire connection. Use the custom `flash_no_reset.py` script instead.

**Steps:**

1. Connect UART adapter to the 3-pin header (Orange→RX, Yellow→TX, Black→GND)
2. Slide **BOOT switch to ISP**
3. Run the flash script:
   ```bash
   cd /path/to/SN50v3
   python3 flash_no_reset.py
   ```
4. When the script says "Waiting for sync", press the **Reset button**
5. Wait for "SUCCESS" message (~15 seconds)
6. Slide **BOOT switch back to FLASH**
7. Press **Reset** to boot the new firmware

### Why the custom flash script?

The SN50v3's 3-pin UART header lacks DTR/RTS connections, so `tremo_loader.py`
cannot auto-reset into bootloader mode. Additionally, at 921600 baud over DuPont
jumper wires, larger packets (like the erase command) are unreliable. The custom
script:

- Syncs at 921600 baud (small 9-byte packets work fine)
- Switches to 115200 baud for reliable data transfer
- Sends erase, flash, and verify commands at the lower baud rate
- Does not require DTR/RTS — manual BOOT switch + Reset button instead

### Troubleshooting flash failures

| Symptom | Fix |
|---|---|
| "No sync" | Verify BOOT switch is on ISP, press Reset, check wiring |
| "Erase fail" / timeout | Likely still at 921600; the script auto-switches to 115200 to avoid this |
| "Device not configured" | USB cable disconnected — secure all cables, don't touch during flash |
| Partial sync bytes | Normal at 921600 over jumpers; the script retries automatically |

## Wiring the SHT45 Sensor

Connect to the SN50v3 **screw terminal block**:

| SHT45 pin | Terminal # | Terminal label | Wire |
|---|---|---|---|
| VDD | 1 | VDD (3.3V) | Red |
| GND | 11 | GND | Black |
| SCL | 4 | SCL | Yellow |
| SDA | 5 | SDA | Blue/White |

The SHT45 uses I2C address `0x44`. No external pull-up resistors needed — the
SN50v3 has internal pull-ups on the I2C bus.

## AT Commands (post-flash)

Connect via UART at **9600 baud**. The AT password is the last 6 characters of the
AppSKey (printed on the device label, or viewable in TTN console).

```
DB2CE7          ← enter password
AT+TDC=180000   ← set transmit interval to 3 minutes (180,000 ms)
AT+TDC=?        ← verify current interval
AT+CHE=2        ← set sub-band 2 (REQUIRED for TTN US915)
AT+CHE=?        ← verify sub-band
ATZ             ← reboot to apply channel changes
```

### Common TDC values

| Interval | AT command |
|---|---|
| 1 minute | `AT+TDC=60000` |
| 3 minutes | `AT+TDC=180000` |
| 5 minutes | `AT+TDC=300000` |
| 10 minutes | `AT+TDC=600000` |
| 30 minutes | `AT+TDC=1800000` |

## TTN Setup

- **Frequency plan**: Must match the firmware region (e.g. US 902-928 MHz FSB 2 for `REGION_US915`)
- **LoRaWAN version**: 1.0.3
- **Activation**: OTAA
- **Payload decoder**: Standard Dragino SN50v3 decoder — SHT45 values appear in the same fields as SHT20/SHT31

### Critical: US915 sub-band (CHE)

TTN US915 listens on **FSB 2 only** (channels 8–15, 903.9–905.3 MHz). If
`AT+CHE=0` (all sub-bands / default), the device wastes join attempts on
channels TTN ignores and will fail to join or join very slowly.

**Fix:** `AT+CHE=2` then `ATZ` to reboot. This restricts the device to FSB 2
channels, matching TTN's gateway configuration.
