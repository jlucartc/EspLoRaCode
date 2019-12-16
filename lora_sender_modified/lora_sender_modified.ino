/*******************************************************************************
 * Modified by Alexandre Barroso, 2019
 * Specific modifications for use of Dragino LoRaSHield on AU915, sub-band 2
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

// Added GPS Libraries and GPS Data -----------------------------------------------
#include <TinyGPS++.h>
TinyGPSPlus gps; // Objeto gps
char lat_str[11], lng_str[12], gps_str[23];
// All GPS Data -------------------------------------------------------------------

// Display OLED Libraries ---------------------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

int counter = 0;
// Display OLED Libraries ---------------------------------------------------------

// UART 2 libraries ---------------------------------------------------------------
//HardwareSerial Serial2(2);
#define RXD2 23 //16 is used for OLED_RST !
#define TXD2 17
// UART 2 libraries ---------------------------------------------------------------

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = {0xC3,0x0E,0xBD,0xC5,0xC5,0xF5,0x92,0xFF,0x87,0xBF,0xD7,0xB3,0xF6,0x04,0xB0,0x2E};

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = {0xC8,0xD7,0x62,0xA4,0x58,0xF3,0x21,0xF6,0xF9,0x26,0x34,0x20,0x5E,0x72,0x8E,0x6D};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x260317F3 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

int i = 0;
int value;
//static uint8_t payload[] = {0, 0, 0, 0};
//uint8_t payload[50] = "                                                  ";
uint8_t payload[24];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;
//const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
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
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        Serial.print("OP_TXRXPEND, not sending; at freq: ");
        Serial.println(LMIC.freq);        
    } else {
        char data[50] = "\0";

        //strcat(data, (char *)gps_str);
        char buffer[10];
        sprintf(buffer, "%d", i++);
        
        strcat(data, "{");
        strcat(data, "umidade: ");
        strcat(data, buffer);
        strcat(data, ", ");
        strcat(data, "status: ");
        strcat(data, "'on'");
        strcat(data, "}");

        // Display operation -------------------------------------------
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("DEV_ID:");
        display.println(DEVADDR,HEX);
        display.setCursor(0,10);
        display.println("SF:7 BW:125");
        display.setCursor(0,20);
        display.println(lat_str); //"LORA SENDER");
        display.setCursor(0,30);
        display.setTextSize(1);
        display.print(lng_str); //"LoRa packet sent.");*/
        display.setCursor(0,40);
        display.print("Counter:");
        display.setCursor(50,40);
        display.print(counter);      
        display.display();
        counter ++;
        // Display operation -------------------------------------------
        
        strcpy((char *)payload, gps_str);//data); 
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.print(F("Packet queued for freq: "));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // SERIAL UART FOR CONNECTION ----------------

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // THIS IS WHERE THE AUSTRALIA FREQUENCY MAGIC HAPPENS!
    // The frequency plan is hard-coded
    // But the band (or selected 8 channels) is configured here!
    // This is the same AU915 band as used by TTN
    
    // First, disable channels 0-7
//    for (int channel=0; channel<8; ++channel) {
//      LMIC_disableChannel(channel);
//    }
    // Now, disable channels 16-72 (is there 72 ??)
//    for (int channel=16; channel<72; ++channel) {
//       LMIC_disableChannel(channel);
//    }
    for (int channel=0; channel<72; ++channel) {
      if(channel != 8) {
        LMIC_disableChannel(channel);
      }
    }
    // This means only channels 8-15 are up

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // GPS Setup -------------------------------------------------------------------
    gps_str[23] = {0};
    // GPS Setup -------------------------------------------------------------------

    // OLED Setup ------------------------------------------------------------------
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);

    Wire.begin(OLED_SDA, OLED_SCL);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
      //Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("LORA SENDER ");
    display.display();

    display.setCursor(0,10);
    display.print("LoRa Initializing OK!");
    display.display();
    delay(2000);
    // OLED Setup ------------------------------------------------------------------
    
    // Start job
    do_send(&sendjob);
}

void loop() {
  if (Serial2.available() > 0){
    if (gps.encode(Serial2.read())){
      dtostrf(gps.location.lat(), 10, 6, lat_str);
      dtostrf(gps.location.lng(), 11, 6, lng_str);

      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("LORA SENDER ");
      
      for (int i = 0; i < 10; i++){
        gps_str[i] = lat_str[i];
      }
      gps_str[10] = ';';
      for (int j = 0; j < 11; j++){
        gps_str[j+11] = lng_str[j];
      }
    }
  }
  Serial.print("Lat;Long: ");
  Serial.println(gps_str);
  
  os_runloop_once();
}
