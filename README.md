Source Code for SN50v3-LB and SN50v3-NB.

SN50v3 uses LA66 module inside. See below for:

# How to Compile Firmware: 
See Step 1 ~ Step 7 of [LA66 Compile Instruction](http://wiki.dragino.com/xwiki/bin/view/Main/User%20Manual%20for%20LoRaWAN%20End%20Nodes/LA66%20LoRaWAN%20Module/Compile%20and%20Upload%20Code%20to%20ASR6601%20Platform/ )

# How to Update Firmware: 
## [via OTA: ](http://wiki.dragino.com/xwiki/bin/view/Main/Firmware%20OTA%20Update%20for%20Sensors/ )
## [via UART ](http://wiki.dragino.com/xwiki/bin/view/Main/UART%20Access%20for%20LoRa%20ST%20v4%20base%20model/ )

## Related repositories (woods deployment ecosystem)

This fork of the Dragino SN50v3-LB / SN50v3-NB firmware is deployed at a hike-in woods site alongside other devices. The cross-cutting docs and ops live in sibling repos:

- [`forest-weather-machines`](https://github.com/disruptivepatternmaterial/forest-weather-machines) — LoRaWAN gateway / The Things Stack ops, Helium config, Node-RED + TimescaleDB + Grafana pipeline, Home Assistant dashboards. SN50v3 uplinks land here.
- [`particle-devices`](https://github.com/disruptivepatternmaterial/particle-devices) — Particle-platform side of the deployment (Tachyon trailcam, Muon weather station, Boron BRN404X). Same site, same backend.
- [`WisBlock-Seismic-Sensor`](https://github.com/disruptivepatternmaterial/WisBlock-Seismic-Sensor) — RAK WisBlock LoRaWAN seismic sensor in the same gateway coverage.

Full system flowchart: [`particle-devices/docs/ECOSYSTEM.md`](https://github.com/disruptivepatternmaterial/particle-devices/blob/main/docs/ECOSYSTEM.md).

This repo remains a fork of [Dragino-LoRaWAN-Devices](https://github.com/dragino/Dragino-LoRaWAN-Devices) so that upstream releases can be rebased in.
