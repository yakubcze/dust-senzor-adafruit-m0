/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2018 Brent Rubell, Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 * 
 * Cast programu souvisejici s LoRaWan byla prevzata z https://lora.vsb.cz/
 * 
 * Program pro cteni dat ze senzoru prachovych castic Sharp GP2Y1010AU0F a
 * odesilani do VSB TTN site.
 *******************************************************************************/

//nacteni knihoven
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

//maximalni velikost zpravy = 51 bitu
CayenneLPP lpp(51);

//piny senzoru
int sensePin = A5; //pin A5 pro snimani
int ledPin = 12; //pin 12 pro IR LED

//hodnoty period pro snimani senzorem
int Tsampling = 280;
int Tdelta = 20;
int Tsleep = 9680;
 
//definovani merenych promennych
float outVoltage = 0;
float adcVoltage= 0;
float dustLevel = 0;

//LoRaWAN klice pro pristup na TTN sit
static const PROGMEM u1_t NWKSKEY[16] = { 0xD2, 0xF4, 0x84, 0x42, 0xBE, 0x77, 0x34, 0x14, 0xC6, 0x99, 0xC2, 0x49, 0xAF, 0x8F, 0x38, 0x4C };   // LoRaWAN NwkSKey, network session key, MSB
static const u1_t PROGMEM APPSKEY[16] = { 0x74, 0xC9, 0x96, 0x5C, 0x80, 0x7E, 0x93, 0xC3, 0x6B, 0x9C, 0x96, 0xEE, 0xC0, 0x77, 0xC4, 0xF1 };   // LoRaWAN AppSKey, application session key, MSB
static const u4_t DEVADDR = 0x260B7BEE;  

// Mapovani LoRa pinu pro Adafruit M0
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)      
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,
    .spi_freq = 8000000,
};
#else
# error "Unknown target"
#endif

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;  //interval odesilani = 60 sekund

//funkce pro naplanovani vysilani
void onEvent (ev_t ev) {
    if(ev == EV_TXCOMPLETE) {
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    }
}

//funkce pro vycteni dat ze senzoru
float getValue () {

  digitalWrite(ledPin,LOW); 
  delayMicroseconds(Tsampling);
 
  outVoltage = analogRead(sensePin); 
 
  delayMicroseconds(Tdelta);
  digitalWrite(ledPin,HIGH); 
  delayMicroseconds(Tsleep);

  adcVoltage = outVoltage * (3.3 / 1023);
  dustLevel = 0.17 * adcVoltage - 0.1;
  return(dustLevel);
}

//funkce ziska data ze senzoru pomoci funkce getValue a posle pomoci lpp
//na seriovou konzoli vypise odesilane hodnoty
void do_send(osjob_t* j){

  dustLevel = getValue();

  Serial.print("Posílám hodnotu prachu (mg/m^3): ");
  Serial.println(dustLevel);

  lpp.reset();                       
  //lpp.addPercentage(1, dustLevel);
  lpp.addAnalogOutput(1, dustLevel); //pridat zmerenou hodnotu do kanalu 1

  LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0); // pripravi data k odesilani
}

void setup() {
    Serial.begin(9600); //nastavi seriovou linku mezi adafruitem a pc s rychlosti 9600 Bd
    pinMode(ledPin,OUTPUT); //nastaveni pinu ktery je pripojeny na IR led senzoru jako vystupniho

    //radky pro odesilani na TTN broker
    os_init();
    LMIC_reset();
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);

    //definovani frekvenci EU868
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
    LMIC_setLinkCheckMode(0);       // Disable link check validation
    LMIC.dn2Dr = DR_SF9;            // TTS uses SF9 for its RX2 window.
    LMIC_setDrTxpow(DR_SF9,14);     // Set data rate and transmit power for uplink
    LMIC_setAdrMode(0);             // Adaptive data rate disabled

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); 

    do_send(&sendjob);     // Start sendjob
}

void loop() {
    os_runloop_once();
}
