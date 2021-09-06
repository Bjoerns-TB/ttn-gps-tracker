# ttn-gps-tracker
An Arduino Sketch for an GPS-Tracker to work with The Things Network.
This Sketch uses TinyGPSPlus https://github.com/mikalhart/TinyGPSPlus
the Rocketscream Low-Power library https://github.com/rocketscream/Low-Power
and the Arduino LMIC-Library by MCCI Catena https://github.com/mcci-catena/arduino-lmic
The hardware used is an Afafruit Feather 32u4 Lora with the Ultimate GPS featherwing.

#### To fit into the memory of the 32U4 with the MCCI Catana LMIC, please add the follwoing lines to your lmic_project_config.h

```
  #define DISABLE_PING
  #define DISABLE_BEACONS
```

For the non OTAA version it is also possible to disable JOIN and free up more memory
```
  #define DISABLE_JOIN
```

The folder ttn contais the payload decoder for TTN.

The folder node-red contains a sample flow adding a geohash and storing data in InfluxDB.

Changes:
- Added versions for OTAA and LowPower
- Enable SBAS 
- Added support for TTN Mapper integration from within TTN console (HDOP, Altitude, Payload Decoder with correct syntax)
- updated to use TinyGPSPlus
