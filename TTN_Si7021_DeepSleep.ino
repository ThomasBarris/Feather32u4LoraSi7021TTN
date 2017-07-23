/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "Adafruit_SleepyDog.h"
#include <avr/power.h>

// This library allows you to communicate with I2C / TWI devices.
#include <Wire.h>
 
// SI7021 I2C address is 0x40(64)
#define si7021Addr 0x40

// There is a double-100K resistor divider on the BAT pin, and connected it to D9 (a.k.a analog #7 A7). 
#define VBATPIN A9

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations) if we are not using deepsleep
const unsigned TX_INTERVAL = 300;

const boolean deepsleep = true;




// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={  };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={  };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {  };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}


// struct {
//  int temp;
//  byte humi;
//  byte power; 
// } mydata;
byte mydata[4];

// function for reading values into a array of unsigned int data[2] 
// from the Si7021 with a specific i2c address si7021Addr=0x40
// with one of the commands listed above. Function body at the end of this sketch
void getSiData(unsigned int *_ret_data, byte _i2c_command);

//function that reads the data from the Si7021 sensor, convert it and puts it in the mydata struc
void updatedata();

static osjob_t sendjob;


// Pin mapping
const lmic_pinmap lmic_pins = { 
   .nss = 8, 
   .rxtx = LMIC_UNUSED_PIN, 
   .rst = 4, 
   .dio = {7, 6, LMIC_UNUSED_PIN}, 
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
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
            if (!deepsleep)
            {
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            }
            else
            {
              for (int i=0; i < int(TX_INTERVAL/8+1); i++) {
                Watchdog.sleep(8000);
              }
              os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
            }
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        updatedata();
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata)-1, 0);
        LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);

    Wire.begin();
    
    Serial.println(F("Starting"));
    // we are giong to power of everytjing. we should wait some time before
    delay(20000);

    // power_adc_disable();
    // power_usart0_disable();
    // power_twi_disable();
    // power_timer1_disable();
    // power_timer2_disable();
    // power_timer3_disable();
    // power_usart1_disable();
    // power_usb_disable();
    // USBCON |= (1 << FRZCLK); 
    // PLLCSR &= ~(1 << PLLE);
    // USBCON &= ~(1 << USBE );

    //reset sensor by sending 0xFE command to the Si7021 address
    Wire.beginTransmission(si7021Addr);
    Wire.write(0xFE); // Write reset command
    Wire.endTransmission();
    delay(15); // Default = 15ms

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setLinkCheckMode(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}


void getSiData(unsigned int *_ret_data, byte _i2c_command)
{

  // start i2c communication 
  Wire.beginTransmission(si7021Addr);
  //send i2c command to sensor
  Wire.write(_i2c_command);
  // we are done with our transmission...close i2c communication
  Wire.endTransmission();
  delay(85);
 
  // Request 2 bytes of data
  Wire.requestFrom(si7021Addr, 2);
  // Read 2 bytes of data and save it to _ret_data which points to 'data[2]'
  if(Wire.available() == 2)
  {
    _ret_data[0] = Wire.read();
    _ret_data[1] = Wire.read();
  }
}

void updatedata()
{
  //sensor returns 2 bytes via I2C. It will be converted to temperature or humidity later
  unsigned int data[2];
    
  //Send humidity measurement command and get response into the array 'data'
  getSiData(data, 0xE5);
 
  // Convert the data
  float humidity  = ((data[0] * 256.0) + data[1]);
  humidity = ((125 * humidity) / 65536.0) - 6;
 
  // Send temperature measurement command
  getSiData(data, 0xE3);
  
  // Convert the data
  float temp  = ((data[0] * 256.0) + data[1]);
  float celsTemp = ((175.72 * temp) / 65536.0) - 46.85;

  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  Serial.print("Humi: " ); Serial.println(humidity);
  Serial.print("Temp: " ); Serial.println(celsTemp);

  mydata[0] = (int (celsTemp*10)) >> 8 & 255;
  mydata[1] = (int (celsTemp*10)) & 255;
  mydata[2] = (int (humidity*2)) & 255;
  mydata[3] = (int (measuredvbat*10)) & 255; 
}
