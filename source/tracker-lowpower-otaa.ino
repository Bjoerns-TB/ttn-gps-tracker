//An Arduino Sketch for an GPS-Tracker to work with The Things Network.
//This Sketch uses TinyGPSPlus https://github.com/mikalhart/TinyGPSPlus
//the Rocketscream Low-Power library https://github.com/rocketscream/Low-Power
//and the Arduino LMIC-Library by MCCI Catena https://github.com/mcci-catena/arduino-lmic
//The hardware used is an Afafruit Feather 32u4 Lora with the Ultimate GPS featherwing.
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>

// use low power sleep; comment next line to not use low power sleep
#define SLEEP

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 32; //multiple of 8

TinyGPSPlus gps;


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x35, 0x7D, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x6C, 0x10, 0x63, 0x2D, 0xD5, 0xD2, 0xF5, 0x75, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


uint8_t coords[8];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {7, 6, LMIC_UNUSED_PIN},
};

void get_coords () {

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial1.available()) {
      char c = Serial1.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      gps.encode(c);
    }
  }

  if (gps.location.isValid()) {

    LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

    coords[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    coords[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    coords[2] = LatitudeBinary & 0xFF;

    coords[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    coords[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    coords[5] = LongitudeBinary & 0xFF;

    altitudeGps = gps.altitude.meters();
    coords[6] = ( altitudeGps >> 8 ) & 0xFF;
    coords[7] = altitudeGps & 0xFF;

    hdopGps = gps.hdop.value() / 10;
    coords[8] = hdopGps & 0xFF;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    get_coords();
    LMIC_setTxData2(1, (uint8_t*) coords, sizeof(coords), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;

    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      next = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Serial1.begin(9600);
  delay(5000);
  Serial1.print("$PMTK301,2*2E\r\n");  // Select SBAS as DGPS source (RTCM)
  Serial1.print("$PMTK313,1*2E\r\n");  // Enable to search a SBAS satellite
  //Serial1.print("$PMTK513,1*28\r\n");

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Start job
  do_send(&sendjob);
}

void loop() {
#ifndef SLEEP

  os_runloop_once();

#else
  extern volatile unsigned long timer0_overflow_count;

  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
    Serial.flush(); // give the serial print chance to complete
    for (int i = 0; i < sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
    next = false;
    // Start job
    do_send(&sendjob);
  }

#endif
}
