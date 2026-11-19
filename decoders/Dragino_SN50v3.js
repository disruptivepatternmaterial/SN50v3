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
 *   - la666152752 — BME280 (DevEUI A84041D1315DE230, MOD=1)
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
 *       BME280 instead of SHT: bytes[7..8]=temperature×10, bytes[9..10]=
 *       pressure in 0.1 hPa (uint16). Humidity is not in the frame.
 *       A second-field value of 1..1000 is humidity×10 (SHT). 1001..65534 is
 *       pressure×10 in hPa (BME280 operating range encodes as 3000..11000).
 *       Null sentinels: DS18B20 0x7FFF; SHT temp 0x7FFF/0xFFFF; SHT hum 0xFFFF.
 *       BME280 with no pressure sample uses that same 0xFFFF in bytes[9..10].
 *
 *     mode 1 (AT+MOD=2, distance — TF-Mini/TF02/TF-Luna/LiDAR-Lite/ultrasonic):
 *       Bat(2) | DS18B20(2) | ADC(2) | flags(1) | Distance_mm(2) | Strength(2)
 *       Distance is raw millimetres (firmware packs TF cm×10).
 *       Null sentinels (Drivers/sensor/tfsensor.c tfsensor_read_distance):
 *         - 0x0000: all-zero UART buffer sets distance_cm = 0 (line 165).
 *         - 0xFFFF: failed frame header/checksum sets distance_cm = 65535 (line 177).
 *         (The 0 / 4500 cm skip in at_tfmini_data_receive at lines 264-266 never
 *         copies those frames into rxdatatemp, leaving the buffer zeroed -> 0.)
 *       Emits { Node_type, battery_v, distance_mm, distance_signal_strength }.
 *       distance_signal_strength is bytes 9..10 as sent (0-65535). With
 *       distance 0 the firmware sends 65534 (no LiDAR frame caught).
 *
 *     mode 11 (AT+MOD=12): Bat | Temp×10 | Hum or pressure | flags | count.
 *       Bytes 4..5 use the same humidity-or-pressure rule as mode 0 bytes 9..10.
 *
 *   fPort 3  — stored readings, 11 bytes each, oldest first, as many as fit
 *              the current data rate. Sent two ways (Drivers/sensor/datalog.c):
 *                - PNACKMD replay: after AT+PNACKMD=1 / downlink 34 01, readings
 *                  the network did not ACK are re-sent after the next ACKed uplink.
 *                - Poll: downlink 31 <start:4> <end:4> <interval_s:1> returns every
 *                  stored reading taken in [start, end], one frame per interval.
 *              0..1 primary (distance mm in distance mode, temperature×10 otherwise)
 *              2..3 second (signal strength, humidity×10, or BME280 pressure
 *                   in 0.1 hPa; 0xFFFF = none). Humidity is not stored for a
 *                   BME280: the record has only this one second field.
 *              4..5 battery millivolts
 *              6    bit7 = network did not acknowledge, bit6 = poll reply,
 *                   low 4 bits = work mode (2 = distance, 1 = temperature/humidity)
 *              7..10 Unix seconds, big-endian, the time the reading was taken
 *                   (seconds since boot until AT+SYNCMOD=1 / downlink 28 01 or a
 *                   30 <unix:4> downlink sets the clock)
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

var DEVICE_PROFILES = {
  A84041D1315DE230: { device_id: 'la666152752', second_field: 'pressure' }
};

function normalizeDevEui(raw) {
  var normalized;
  if (typeof raw !== 'string') {
    return null;
  }
  normalized = raw.replace(/[^a-fA-F0-9]/g, '').toUpperCase();
  if (normalized.length !== 16) {
    return null;
  }
  return normalized;
}

function lookupDeviceProfile(input) {
  var devEui = normalizeDevEui(
    input.devEui
      || input.dev_eui
      || (input.end_device_ids && (input.end_device_ids.dev_eui || input.end_device_ids.devEui))
      || (input.endDeviceIds && (input.endDeviceIds.dev_eui || input.endDeviceIds.devEui))
  );
  if (!devEui) {
    return null;
  }
  return DEVICE_PROFILES[devEui] || null;
}

function applySecond(obj, second, deviceProfile) {
  if (second === 0xFFFF) {
    if (deviceProfile && deviceProfile.second_field === 'pressure') {
      obj.pressure_hpa = null;
    } else {
      obj.humidity = null;
    }
  } else if (deviceProfile && deviceProfile.second_field === 'pressure') {
    obj.pressure_hpa = parseFloat((second / 10).toFixed(1));
  } else if (second <= 1000) {
    obj.humidity = parseFloat((second / 10).toFixed(1));
  } else {
    obj.pressure_hpa = parseFloat((second / 10).toFixed(1));
  }
}

function decodeUplink(input) {
  var bytes = input.bytes || [];
  var port = input.fPort;
  var deviceProfile = lookupDeviceProfile(input);
  var decoded = { Node_type: 'SN50v3' };
  if (deviceProfile) {
    decoded.device_profile = deviceProfile.device_id;
  }

  // Length validation before the fixed-offset reads below: in JS, `undefined`
  // in a bitwise op is 0, so a truncated frame would otherwise decode to
  // plausible fabricated values (e.g. 0.0 C) instead of erroring.
  if (port === 2 && bytes.length !== 11) {
    return { errors: ["Truncated/invalid payload: fPort 2 payload must be 11 bytes, got " + bytes.length] };
  }

  if (port === 2) {
    var mode = (bytes[6] & 0x7C) >> 2;

    if (mode === 0) {
      decoded.battery_v = (bytes[0] << 8 | bytes[1]) / 1000;
      if (bytes[2] === 0x7F && bytes[3] === 0xFF) {
        decoded.temp_probe = null;
      } else {
        decoded.temp_probe = parseFloat(((bytes[2] << 24 >> 16 | bytes[3]) / 10).toFixed(1));
      }
      decoded.adc1_v = (bytes[4] << 8 | bytes[5]) / 1000;
      decoded.work_mode = 'IIC';
      if ((bytes[9] << 8 | bytes[10]) === 0) {
        decoded.illuminance = bytes[7] << 8 | bytes[8];
      } else {
        if ((bytes[7] === 0x7F && bytes[8] === 0xFF) || (bytes[7] === 0xFF && bytes[8] === 0xFF)) {
          decoded.temperature = null;
        } else {
          decoded.temperature = parseFloat(((bytes[7] << 24 >> 16 | bytes[8]) / 10).toFixed(1));
        }
        applySecond(decoded, bytes[9] << 8 | bytes[10], deviceProfile);
      }
      return { data: decoded };
    }

    if (mode === 1) {
      decoded.battery_v = (bytes[0] << 8 | bytes[1]) / 1000;
      var dist = bytes[7] << 8 | bytes[8];
      // Drivers/sensor/tfsensor.c tfsensor_read_distance: all-zero rxdata_dis
      // sets distance_cm = 0 (line 165); failed checksum sets 65535 (line 177).
      // The 0 / 4500 cm skip at lines 264-266 leaves rxdatatemp zeroed -> 0.
      decoded.distance_mm = (dist === 0 || dist === 0xFFFF) ? null : dist;
      decoded.distance_signal_strength = bytes[9] << 8 | bytes[10];
      return { data: decoded };
    }

    if (mode === 11) {
      decoded.battery_v = (bytes[0] << 8 | bytes[1]) / 1000;
      decoded.work_mode = 'IIC_count';
      if ((bytes[2] === 0x7F && bytes[3] === 0xFF) || (bytes[2] === 0xFF && bytes[3] === 0xFF)) {
        decoded.temperature = null;
      } else {
        decoded.temperature = parseFloat(((bytes[2] << 24 >> 16 | bytes[3]) / 10).toFixed(1));
      }
      applySecond(decoded, bytes[4] << 8 | bytes[5], deviceProfile);
      decoded.pulse_count = ((bytes[7] << 24) | (bytes[8] << 16) | (bytes[9] << 8) | bytes[10]) >>> 0;
      return { data: decoded };
    }

    return { errors: ["Unhandled work mode in flags: " + mode] };
  }

  if (port === 3) {
    var entries = [];
    var i;
    for (i = 0; i + 11 <= bytes.length; i += 11) {
      var flags = bytes[i + 6];
      var workMode = flags & 0x0F;
      var primary = (bytes[i] << 8) | bytes[i + 1];
      var second = (bytes[i + 2] << 8) | bytes[i + 3];
      var ts = ((bytes[i + 7] << 24) | (bytes[i + 8] << 16) | (bytes[i + 9] << 8) | bytes[i + 10]) >>> 0;
      var entry = {
        work_mode: workMode,
        no_ack: (flags & 0x80) === 0x80,
        poll_message: (flags & 0x40) === 0x40,
        battery_v: ((bytes[i + 4] << 8) | bytes[i + 5]) / 1000,
        timestamp: ts
      };
      if (workMode === 2) {
        entry.distance_mm = (primary === 0 || primary === 0xFFFF) ? null : primary;
        entry.distance_signal_strength = (second === 0xFFFF) ? null : second;
      } else if (workMode === 1 || workMode === 3 || workMode === 12) {
        if (primary === 0x7FFF || primary === 0xFFFF) {
          entry.temperature = null;
        } else {
          var signed = (primary & 0x8000) ? primary - 0x10000 : primary;
          entry.temperature = parseFloat((signed / 10).toFixed(1));
        }
        applySecond(entry, second, deviceProfile);
      } else {
        entry.primary = primary;
        entry.second = second;
      }
      entries.push(entry);
    }
    if (entries.length === 0) {
      return { errors: ["Truncated/invalid payload: fPort 3 payload must be 11 bytes per saved reading, got " + bytes.length] };
    }
    var first = entries[0];
    var out = { Node_type: 'SN50v3', datalog: true, datalog_entries: entries };
    var key;
    for (key in first) {
      if (Object.prototype.hasOwnProperty.call(first, key)) {
        out[key] = first[key];
      }
    }
    return { data: out };
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
