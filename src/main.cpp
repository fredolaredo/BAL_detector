#include <Arduino.h>
#include "private.h"

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "time.h"

#define CONTACT_PIN   2
#define LED_PIN       5
#define STANDARD_DIM  50

int nbLoop = 0;

enum STATE {
  INIT,
  SEND,
  WAIT_SEND,
  SLEEP,
  ERROR
};

volatile STATE state = INIT;

void dimLed(int msecs = 1000, int power = STANDARD_DIM, int num = 1) {
  #define pas 2
  for (int nb=0 ; nb < num ; nb++) {
    for (int j=0; j<power ; j+= pas ) {
      analogWrite(LED_PIN,j); 
      delay(msecs / 2 / (power / pas)); 
    }
    for (int j=power; j>0 ; j-= pas) {
      analogWrite(LED_PIN,j);  
      delay( msecs / 2 / (power / pas));
    }
    digitalWrite(LED_PIN,LOW);
  }
  
}


///////////////////////////////////////////////////////////////////
// DHT22
///////////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN     A0        // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, 22);

uint32_t delayMS;
float temperature;
float humidity;

void initDHT() {
  Serial.println("DHT22 started");
  dht.begin();
}

void getTempHumid() {
  sensors_event_t event;

  dht.temperature().getEvent(&event);
  dht.humidity().getEvent(&event);
  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    temperature = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    humidity = event.relative_humidity;
  }
}

//////////////////////////////////////////////////////////////////
// SX1276
///////////////////////////////////////////////////////////////////

#define SX1276_NSS  8
#define SX1276_RST  9 
#define SX1276_DIO2 10
#define SX1276_MOSI 11
#define SX1276_MISO 12
#define SX1276_SCK  13
#define SX1276_DIO0 6
#define SX1276_DIO1 7

void opmode_str(){
  if (LMIC.opmode & OP_NONE) Serial.print(" OP_NONE");
  if (LMIC.opmode & OP_SCAN) Serial.print(" OP_SCAN");          // radio scan to find a beacon
  if (LMIC.opmode & OP_TRACK) Serial.print(" OP_TRACK");        // track my networks beacon (netid)
  if (LMIC.opmode & OP_JOINING) Serial.print(" OP_JOINING");    // device joining in progress (blocks other activities)
  if (LMIC.opmode & OP_TXDATA) Serial.print(" OP_TXDATA");      // TX user data (buffered in pendTxData)
  if (LMIC.opmode & OP_POLL) Serial.print(" OP_POLL");          // send empty UP frame to ACK confirmed DN/fetch more DN data
  if (LMIC.opmode & OP_REJOIN) Serial.print(" OP_REJOIN");      // occasionally send JOIN REQUEST
  if (LMIC.opmode & OP_SHUTDOWN) Serial.print(" OP_SHUTDOWN");  // prevent MAC from doing anything
  if (LMIC.opmode & OP_TXRXPEND) Serial.print(" OP_TXRXPEND");  // TX/RX transaction pending
  if (LMIC.opmode & OP_RNDTX) Serial.print(" OP_RNDTX");        // prevent TX lining up after a beacon
  if (LMIC.opmode & OP_PINGINI) Serial.print(" OP_PINGINI");    // pingable is initialized and scheduling active
  if (LMIC.opmode & OP_PINGABLE) Serial.print(" OP_PINGABLE");  // we're pingable
  if (LMIC.opmode & OP_NEXTCHNL) Serial.print(" OP_NEXTCHNL");  // find a new channel
  if (LMIC.opmode & OP_LINKDEAD) Serial.print(" OP_LINKDEAD");  // link was reported as dead
  if (LMIC.opmode & OP_TESTMODE) Serial.print(" OP_TESTMODE");  // developer test mode
}


#ifdef ABP
  uint8_t APPSKEY[16] = SECRET_APPKEY;  
  uint8_t NWKSKEY[16] = SECRET_NWKSKEY;   
  static const u4_t DEVADDR =  SECRET_DEVADDR; 
  void os_getArtEui (u1_t* buf) { }
  void os_getDevEui (u1_t* buf) { }
  void os_getDevKey (u1_t* buf) { }
#else
  static const u1_t PROGMEM APPEUI[8]= SECRET_APPEUI;
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  static const u1_t PROGMEM DEVEUI[8]= SECRET_DEVEUI;
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  static const u1_t PROGMEM APPKEY[16] = SECRET_APPKEY;
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

static uint8_t mydata[] = "device3";
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SX1276_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = SX1276_RST,
    .dio = {SX1276_DIO0,SX1276_DIO1,SX1276_DIO2}
};

void onEvent (ev_t ev) {
    Serial.println();

    const uint32_t gt = os_getTime();
    tm *mytl = localtime(&gt);
    Serial.print(mytl->tm_hour); Serial.print(":"); Serial.print(mytl->tm_min); Serial.print(":");Serial.print(mytl->tm_sec); 

    Serial.print(String(" - ") + ev + String(" : "));
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
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
            state = SLEEP;
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
            Serial.println(F("Unhandled event : "));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void do_send_array(osjob_t* j, xref2u1_t data, int size){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data, size, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void init_lmic() {
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    #ifdef ABP
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
      //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      // LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF7,DR_SF12),  BAND_CENTI);      // g-band
      // LMIC_setupChannel(0, 868100000, DR_SF7,  BAND_CENTI);      // g-band
      for (int i=1 ; i < 9 ; i++) LMIC_disableChannel(i);
      LMIC_setDrTxpow(DR_SF7,14);
      // Disable link check validation
      LMIC_setLinkCheckMode(0);
      // TTN uses SF9 for its RX2 window.
      LMIC.dn2Dr = DR_SF9;
      // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    #else
      LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    #endif
    
    
}

///////////////////////////////////////////////////////////////////

ISR(WDT_vect){
  //DON'T FORGET THIS!  Needed for the watch dog timer.  
  //This is called after a watch dog timer timeout - 
  //this is the interrupt function called after waking up
}// watchdog interrupt

void digitalInterrupt(){
  //needed for the digital input interrupt
  //Serial.println("interrupt !");
  if (state == SEND || state == WAIT_SEND) {
    //Serial.println("already in SEND process");
  } else {
    state = SEND;
  //do_send(&sendjob);
  }
}

void setup_sleep_mode() {
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  //SETUP WATCHDOG TIMER
  //WDTCSR = (24);//change enable and WDE - also resets
  //WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  //WDTCSR |= (1<<6);//enable interrupt mode
}

void goToSleep() {
  // Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);

  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time
  __asm__  __volatile__("sleep");//in line assembler to go to sleep
}

///////////////////////////////////////////////////////////////////

long int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long int result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

long int Vint = 0;

///////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("----> Setup");
  dimLed(1000);
  // for low power : to be tested
  for (int i = 0; i < 20; i++) {
    pinMode(i, INPUT);
  }
  pinMode(CONTACT_PIN,INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("attachInterrupt " + String(CONTACT_PIN));
  attachInterrupt(digitalPinToInterrupt(CONTACT_PIN), digitalInterrupt,RISING);
  init_lmic();
  setup_sleep_mode();
  initDHT();
  getTempHumid();
  Serial.println("----> Setup Ended");
  dimLed(1000);
}

///////////////////////////////////////////////////////////////////

void loop() {
  //Serial.println("----> Loop : " + String(nbLoop++));

  switch (state) {
  case INIT:
    Serial.println("---- INIT ----"); delay(100);
    opmode_str(); Serial.println();
    state = SLEEP;
    break;

  case SEND:
    Serial.println("---- SEND ----"); //delay(100);
    dimLed(100);
    // opmode_str(); Serial.println();
    Serial.print("LMIC Batt Level : "); Serial.print( map(os_getBattLevel(),0,255,0,100)); Serial.println("%");
    Vint = readVcc();
    Serial.print("battery : "); Serial.print(Vint); Serial.println("mV");
    getTempHumid();
    u1_t payload[7];
    payload[0] = Vint >> 8; 
    payload[1] = Vint & 0xFF;
    payload[2] = (int)(temperature*100) >> 8;
    payload[3] = (int)(temperature*100) & 0xFF;
    payload[4] = (int)(humidity*100) >> 8;
    payload[5] = (int)(humidity*100) & 0xFF;
    payload[6] = DEVICE_ID;
    do_send_array(&sendjob, payload, 7);
    opmode_str(); Serial.println();
    state = WAIT_SEND;
    break;

  case WAIT_SEND:
    // Serial.print("---- WAIT_SEND ---- "); 
    os_runloop_once();
    // if (LMIC.opmode & OP_TXRXPEND) {
    //   Serial.print("sending... ");
    // }
    //Serial.print("Op mode = "); Serial.print(LMIC.opmode); Serial.print("                                     \r");
    // opmode_str(); Serial.print("\n\r"); 
    break;

  case SLEEP:
    Serial.println("---- SLEEP ----"); delay(100);
    // hal_sleep(); not implemented
    // os_radio(RADIO_RST); // SX1276 into Sleep Mode
    // LMIC_init();
    // init_lmic();
    // sensor_t sensor;
    // dht.temperature().getSensor(&sensor);
    dimLed(2000);
    Serial.flush();
    goToSleep(); 
    ADCSRA |= (1 << 7);
    break;

  case ERROR:
    state = SLEEP;
    break;

  default:
    Serial.println("---- UNKNOWN ----"); delay(100);
    dimLed(100);
    Serial.flush();
    state = SLEEP;
    break;
  }
  
  //Serial.println("wait few secs .....");
  delay(100);
}


///////////////////////////////////////////////////////////////////

