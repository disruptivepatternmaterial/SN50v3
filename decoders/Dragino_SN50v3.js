/**
 * SN50v3-LB payload decoder for The Things Network (TTN / The Things Stack).
 *
 * Device: Dragino SN50v3-LB Universal LoRaWAN Sensor Node running the custom
 * SHT45 firmware in this repo (see docs/SHT45-CUSTOM-FIRMWARE.md). Also valid
 * for stock SHT20/SHT31 / TF-series distance units — SHT45 uses the same
 * temp/humidity slots as SHT20/SHT31.
 *
 * Deployed devices:
 *   - la666152750 — Forest Lands High Point (SHT2x, MOD=1)
 *   - la666152752 — SHT45 (DevEUI A84041D1315DE230, MOD=1)
 *   - la666152751 — TF02-Pro LiDAR snow depth (DevEUI A84041408A5DE22F, MOD=2)
 *
 * Source of truth: this repo's firmware,
 *   Projects/Applications/DRAGINO-LRWAN-AT/src/main.c (payload packing).
 *
 * Ports:
 *   fPort 2  — sensor uplink, 11 bytes. Mode is bits 6:2 of flags (bytes[6]):
 *
 *     mode 0 (AT+MOD=1, IIC):
 *       Bat(2) | DS18B20(2) | ADC(2) | flags(1) | SHT_Temp(2) | SHT_Hum(2)
 *       BH1750 instead of SHT: bytes[7..8]=illuminance, bytes[9..10]=0x0000.
 *       Null sentinels: DS18B20 0x7FFF; SHT temp 0x7FFF/0xFFFF; SHT hum 0xFFFF.
 *
 *     mode 1 (AT+MOD=2, distance — TF-Mini/TF02/TF-Luna/LiDAR-Lite/ultrasonic):
 *       Bat(2) | DS18B20(2) | ADC(2) | flags(1) | Distance_mm(2) | Strength(2)
 *       TF-series: Strength is signal strength; other distance sensors use 0xFFFF.
 *       Distance is millimetres (firmware packs TF cm×10). 0xFFFF = no reading.
 *
 *   fPort 4  — 1-byte MAC/ADR acknowledgement (0x00 after ADR, 0x11 after ATZ).
 *   fPort 5  — device status, 7 bytes:
 *              0x1C | fw_version(2, BCD) | freq_band(1) | sub_band(1) | Bat(2)
 *
 * Known edge cases (inherent to Dragino's sentinel encoding):
 *   - SHT temperature of exactly -0.1 C encodes as 0xFFFF and decodes as null.
 *   - SHT humidity of exactly 0.0 % is indistinguishable from the BH1750
 *     layout and decodes as illuminance.
 *
 * Parser alias chain maps: temperature → temperature_c, humidity → humidity_pct.
 */

var FREQ_BANDS = {
  1: 'EU868', 2: 'US915', 3: 'IN865', 4: 'AU915', 5: 'KZ865', 6: 'RU864',
  7: 'AS923', 8: 'AS923-1', 9: 'AS923-2', 10: 'AS923-3', 11: 'CN470',
  12: 'EU433', 13: 'KR920', 14: 'MA869', 15: 'AS923-4'
};

function decodeUplink(input) {
  var bytes = input.bytes || [];
  var port = input.fPort;
  var decoded = { Node_type: 'SN50v3' };

  // Length validation before the fixed-offset reads below: in JS, `undefined`
  // in a bitwise op is 0, so a truncated frame would otherwise decode to
  // plausible fabricated values (e.g. 0.0 C) instead of erroring.
  if (port === 2 && bytes.length !== 11) {
    return { errors: ["Truncated/invalid payload: fPort 2 payload must be 11 bytes, got " + bytes.length] };
  }

  if (port === 2) {
    var mode = (bytes[6] & 0x7C) >> 2;

    // Shared fields for modes that carry bat / DS18B20 / ADC in the front.
    if (mode === 0 || mode === 1) {
      decoded.battery_v = (bytes[0] << 8 | bytes[1]) / 1000;
      if (bytes[2] === 0x7F && bytes[3] === 0xFF) {
        decoded.temp_probe = null;
      } else {
        decoded.temp_probe = parseFloat(((bytes[2] << 24 >> 16 | bytes[3]) / 10).toFixed(1));
      }
      decoded.adc1_v = (bytes[4] << 8 | bytes[5]) / 1000;
    }

    if (mode === 0) {
      decoded.work_mode = 'IIC';
      if ((bytes[9] << 8 | bytes[10]) === 0) {
        decoded.illuminance = bytes[7] << 8 | bytes[8];
      } else {
        if ((bytes[7] === 0x7F && bytes[8] === 0xFF) || (bytes[7] === 0xFF && bytes[8] === 0xFF)) {
          decoded.temperature = null;
        } else {
          decoded.temperature = parseFloat(((bytes[7] << 24 >> 16 | bytes[8]) / 10).toFixed(1));
        }
        if (bytes[9] === 0xFF && bytes[10] === 0xFF) {
          decoded.humidity = null;
        } else {
          decoded.humidity = parseFloat(((bytes[9] << 8 | bytes[10]) / 10).toFixed(1));
        }
      }
      return { data: decoded };
    }

    if (mode === 1) {
      decoded.work_mode = 'DISTANCE';
      var dist = bytes[7] << 8 | bytes[8];
      var strength = bytes[9] << 8 | bytes[10];
      if (dist === 0xFFFF) {
        decoded.distance_mm = null;
        decoded.distance_cm = null;
      } else {
        decoded.distance_mm = dist;
        decoded.distance_cm = parseFloat((dist / 10).toFixed(1));
      }
      // TF-series fills strength; LiDAR-Lite / ultrasonic send 0xFFFF.
      if (strength !== 0xFFFF) {
        decoded.distance_signal_strength = strength;
      }
      return { data: decoded };
    }

    return { errors: ["Unhandled work mode in flags: " + mode] };
  }

  if (port === 4) {
    decoded.event = bytes[0] === 0x11 ? 'reboot_ack' : 'mac_ack';
    return { data: decoded };
  }

  if (port === 5 && bytes.length === 7 && bytes[0] === 0x1C) {
    decoded.event = 'device_status';
    decoded.sensor_model = 'SN50v3';
    decoded.firmware_version = 'v' + (bytes[1] & 0x0F) + '.' + ((bytes[2] >> 4) & 0x0F) + '.' + (bytes[2] & 0x0F);
    decoded.frequency_band = FREQ_BANDS[bytes[3]] || ('unknown(' + bytes[3] + ')');
    decoded.sub_band = bytes[4] === 0xFF ? null : bytes[4];
    decoded.battery_v = (bytes[5] << 8 | bytes[6]) / 1000;
    return { data: decoded };
  }

  return { errors: ["Unhandled fPort " + port + " (" + bytes.length + " bytes)"] };
}
