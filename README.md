Firmware for the Dragino SN50v3-LB (ASR6601 / LA66 inside), forked from
[dragino/SN50v3](https://github.com/dragino/SN50v3) with:

- Sensirion **SHT45** auto-detect on the I2C port (MOD=1).
- Bosch **BME280** (chip ID `0x60`) or **BMP280** (`0x58`, no humidity)
  auto-detect at `0x76`/`0x77` on that same I2C port, probed once at boot
  after the SHT sensors and before the BH1750. MOD=1/3: temperature in the
  SHT slot, pressure (hPa×10) in the humidity slot, no humidity. MOD=12:
  temperature, humidity and pressure in one frame (see below).
- **Benewake TF02-Pro** LiDAR over UART (MOD=2) with frame validation and
  the sensor kept at full power.
- **Stored readings** (datalog) in flash, with Dragino's PNACKMD replay and
  `0x31` time-range poll on fPort 3.
- A 3-wire UART flasher (`flash_no_reset.py`) that works without DTR/RTS.

The only application is `Projects/Applications/DRAGINO-LRWAN-AT` (GNU make,
`-DREGION_US915` in its `Makefile`). Dragino's v1.3.4 release shipped a second
Keil-only copy, `DRAGINO-LRWAN(AT)`; its features (interrupt debounce, INTMOD
mode 4, MOD=12) are merged into `DRAGINO-LRWAN-AT` and the copy is gone, as it
is upstream.

Wiring details, SHT45 pins, and troubleshooting: [`docs/SHT45-CUSTOM-FIRMWARE.md`](docs/SHT45-CUSTOM-FIRMWARE.md).
TTN payload formatter: [`decoders/Dragino_SN50v3.js`](decoders/Dragino_SN50v3.js).

## Build (macOS build host)

One-time:

```bash
brew tap osx-cross/arm && brew install arm-gcc-bin@10
pip3 install pyserial
```

Every build. `envsetup.sh` must be sourced from the repo root (it derives
`TREMO_SDK_PATH` from the current directory), and `make clean` does not empty
`Make_out`:

```bash
cd ~/Documents/GitHub/lorawan/SN50v3
export PATH=/opt/homebrew/Cellar/arm-gcc-bin@10/10.3-2021.10_1/bin:$PATH   # if arm-none-eabi-gcc is not already on PATH
source build/envsetup.sh
cd Projects/Applications/DRAGINO-LRWAN-AT
rm -rf Make_out && make EXECUTABLE_SUFFIX= -j8
arm-none-eabi-size Make_out/DRAGINO-LRWAN-AT.elf        # text 99136 on 2026-11-19
```

Output: `Projects/Applications/DRAGINO-LRWAN-AT/Make_out/DRAGINO-LRWAN-AT.bin`.
`realpath --relative-to` warnings on macOS are harmless. Change the region in
the `$(PROJECT)_DEFINES` line of the `Makefile`.

## Flash a node

Hardware: FTDI 3.3 V USB-UART cable, three jumper wires. Open the case; the
PCB has a 3-pin header (GND/TX/RX), a BOOT slide switch (ISP/FLASH) and a RESET
button.

| FTDI wire | Board pin |
|---|---|
| Orange (TXD) | RX |
| Yellow (RXD) | TX |
| Black (GND) | GND |

Do not connect the red 5 V wire; the battery powers the board.

```bash
cd ~/Documents/GitHub/lorawan/SN50v3
# 1. BOOT switch -> ISP
python3 flash_no_reset.py            # auto-detects /dev/tty.usbserial-*
# 2. When it prints "Waiting for sync", press RESET
# 3. Wait for SUCCESS (~15 s). It erases, writes and verifies at 0x0800D000.
# 4. BOOT switch -> FLASH, press RESET
```

`flash_no_reset.py` syncs at 921600, drops to 115200 for data, and retries
each 256-byte chunk up to four times; "No sync" means the switch is not on
ISP or RESET was not pressed. Device keys (DevEUI/AppEUI/AppKey at 0x0803E000)
and settings (0x0803F000) are outside the flashed range and survive.

## Configure a new TF02-Pro LiDAR node

Wire the TF02-Pro to the screw terminals: red → **+5V**, black → **GND**,
white (sensor RXD) → **TX** (PB9), green (sensor TXD) → **RX** (PB8). The
firmware detects it at boot (`check_deceive` in `Drivers/sensor/tfsensor.c`).

Open a 9600-baud terminal on the same FTDI cable (`screen /dev/tty.usbserial-* 9600`).
The AT password is the last 6 hex characters of the AppSKey printed on the
device label.

```
<AT_PIN>
AT+MOD=2          distance mode
AT+CHE=2          US915 sub-band 2 (what TTN listens on)
AT+TDC=300000     uplink every 5 minutes
AT+PNACKMD=1      optional: store readings the network does not ACK and replay them
AT+SYNCMOD=1      optional: set the clock from the network so stored readings carry real time
ATZ
```

`AT+GETSENSORVALUE=1` reads the LiDAR once; it returns `AT_BUSY_ERROR` while a
join or transmit is in progress — wait and retry rather than looping.

In TTN: frequency plan "United States 902-928 MHz, FSB 2", LoRaWAN 1.0.3,
OTAA, and paste `decoders/Dragino_SN50v3.js` as the custom JavaScript uplink
formatter. The formatter reports `distance_mm` as the raw millimetres the
sensor measured. Do not subtract a mounting height in the formatter; do that in
the database or dashboard where it can be changed per device.

Uplink (fPort 2, 11 bytes, MOD=1, BME280): same layout as SHT, with bytes 9–10
holding pressure in 0.1 hPa instead of humidity×10. Bytes 7–8 stay
temperature×10. A value 1–1000 in that slot is still SHT humidity; 1001–65534
is pressure. `0xFFFF` is no reading.

Uplink (fPort 2, 11 bytes, MOD=12 with a BME280/BMP280 — `la666152752`):
`Bat_mV(2) | Temp×10(2, signed) | Hum×10(2) | flags(1) | Pressure hPa×10(4)`.
`flags` = `0xAC` (`0xAE` with the digital input high): MOD=12 plus bit7, which marks this
layout. TTN formatters get no DevEUI, so the decoder keys on bit7, not the
device. Null: temp `0x7FFF`, humidity `0xFFFF` (always on a BMP280),
pressure `0xFFFFFFFF`. MOD=12 without a BME/BMP280 is unchanged
(humidity + pulse count, bit7 clear). 11 bytes fits US915 DR0.
The formatter emits only `battery_v`, `temperature_c`, `humidity_%`,
`pressure_hpa` for BME/BMP280 frames (MOD=12, or MOD=1/3 with pressure in
bytes 9–10), omitting any the frame lacks; fPort 3 entries add `timestamp_s`.

Uplink (fPort 2, 11 bytes, MOD=2):
`Bat(2) | DS18B20(2) | ADC(2) | flags(1) | Distance_mm(2) | Strength(2)`.
`distance_mm` of 0 or 0xFFFF means no measurement and decodes as `null`.
`distance_signal_strength` is the strength word as sent; with distance 0 the
firmware sends 65534, meaning no LiDAR frame was caught.

## Stored readings (datalog)

Every reading is written to a 52 KB flash ring (`0x08031000`–`0x0803E000`,
about 3,300 readings; ≈11 days at 5-minute uplinks). Each slot is 16 bytes;
the oldest page is erased when the ring wraps. Settings and keys are not in
that range.

Same commands as Dragino's other datalog nodes:

| Command | Downlink | Effect |
|---|---|---|
| `AT+PNACKMD=1` | `34 01` | Uplinks become confirmed (1 try). A reading with no ACK is stored with bit7 set; after the next ACKed uplink the unacked readings are re-sent on fPort 3, oldest first, as many as fit the data rate, 10 s apart. Readings unacked before a reboot are not re-sent automatically; use the poll. |
| `AT+SYNCMOD=1` | `28 01` | Request network time (DeviceTimeReq) so stored readings carry Unix time. Until then the timestamp is seconds since boot. |
| `AT+SYNCTDC=10` | `29 0A` | Days between clock requests (default 10). |
| — | `30 <unix:4>` | Set the clock directly. |
| — | `31 <start:4> <end:4> <interval_s:1>` | Poll: send every stored reading taken in `[start, end]` on fPort 3 with bit6 set, one frame per `interval_s` (minimum 5 s). |
| `AT+PDTA=first,last` | — | Print stored readings by index (1 = oldest) on the console. |
| `AT+PLDTA=N` | — | Print the last N stored readings. |
| `AT+CLRDTA` | — | Erase the log. `AT+FDR` also erases it. |

fPort 3 record (11 bytes): `primary(2) | second(2) | bat_mV(2) | flags(1) | unix_time(4)`.
`primary`/`second` are distance mm / strength in MOD=2, temperature×10 / humidity×10
in MOD=1/3/12 with an SHT. With a BME280 in MOD=1/3, `primary` is still
temperature×10 and `second` is pressure in 0.1 hPa; humidity does not fit.
MOD=12 with a BME/BMP280 (`la666152752`) sets `flags` bit4 and stores
`Temp×10(2) | Hum×10(2) | Pressure hPa×10(2) | flags(1) | unix_time(4)` —
pressure replaces `bat_mV`, so those entries decode `battery_v: null`.
`flags` = bit7 no-ACK, bit6 poll reply, bit4 BME triplet, low nibble = work mode.
`decoders/Dragino_SN50v3.js` decodes these as `datalog_entries`.

## `la666152752` activation (BME280 temperature + humidity + pressure)

Power the BME280 from the always-on **VDD 3.3 V** terminal, not the **+5V**
output. +5V is switched by PB7 and is off during the boot-time I2C probe in
MOD=1/12, so a sensor on it is never detected (uplinks show `0xFFFF`). I2C is
SCL = PA14, SDA = PA15.

1. Flash `Make_out/DRAGINO-LRWAN-AT.bin` with `flash_no_reset.py` (above).
2. Uplink formatter: `decoders/Dragino_SN50v3.js` (same file as the other
   SN50v3 units; it needs no DevEUI).
3. Serial console, 9600 baud: `AT+MOD=12`, `AT+PNACKMD=1` (datalog replay),
   `AT+SYNCMOD=1` (Unix timestamps), then `ATZ`.
4. Boot log must print `I2C 0x76 chip ID 0x60` (or `0x77`) and
   `Use Sensor is BME280`. `No I2C device detected` means the sensor was not
   found at boot: uplinks then carry `0xFFFF` for temperature and humidity.
   Chip ID `0x58` is a BMP280: temperature and pressure, humidity `null`.

This feature is in this repository's firmware only; Dragino's public SN50v3
firmware does not store readings. A node has it once it is flashed from this
repo.

## Related repositories

Deployed at a hike-in woods site alongside:

- [`forest-weather-machines`](https://github.com/disruptivepatternmaterial/forest-weather-machines) — gateway / The Things Stack ops, Node-RED + TimescaleDB + Grafana pipeline. SN50v3 uplinks land here.
- [`particle-devices`](https://github.com/disruptivepatternmaterial/particle-devices) — Particle side of the deployment; system flowchart in [`docs/ECOSYSTEM.md`](https://github.com/disruptivepatternmaterial/particle-devices/blob/main/docs/ECOSYSTEM.md).
- [`WisBlock-Seismic-Sensor`](https://github.com/disruptivepatternmaterial/WisBlock-Seismic-Sensor) — RAK WisBlock seismic sensor in the same coverage.

Upstream remote: `upstream` → [dragino/SN50v3](https://github.com/dragino/SN50v3); merge `upstream/main` to pick up releases.
