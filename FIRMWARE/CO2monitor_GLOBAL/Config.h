#pragma once

// General Function Libraries - Already installed in Arduino IDE
#include <Arduino.h>
#include <stdint.h>
#include <Millis.h>
#include <EEPROM.h>        // For writing values to the EEPROM

//Adafruit IO info
//https://io.adafruit.com/api/docs/mqtt.html#mqtt-connection-details
#define         AIO_SERVER                         "io.adafruit.com" // direct
#define         AIO_SERVERPORT                     8883    // Use 8883 if at all possible! For secure conenction

char            AIO_USERNAME_Label[13]            = "AIO_Username";
#define         AIO_USERNAME_LEN                  20
char            AIO_USERNAME[AIO_USERNAME_LEN]    = "YOUR ADAFRUIT USERNAME";

char            AIO_API_KEY_Label[12]             = "AIO_API_Key";
#define         AIO_API_KEY_LEN                   50
char            AIO_KEY[AIO_API_KEY_LEN]          = "YOUR ADAFRUIT API KEY";

char            AIO_feed_name_Label[14]           = "AIO_Feed_Name";
#define         AIO_feed_name_LEN                 20
char            AIO_feed_name[AIO_feed_name_LEN]  = "YOUR FEED NAME";


#define         AP_SSID                    "CO2MONITOR"
#define         AP_PASS                    "password"
#define         AP_TIMEOUT_SEC             20           // Timeout in seconds for AP if no parameters

#define         DEBUG_GRAPH                true // Show graph updates on serial
#define         DEBUG_MQTT                 true // Show MQTT updates on serial
#define         DEBUG_ENCODER              false // Show Encoder updates on serial
#define         DEBUG_CO2                  true // Show CO2 updates sensor on serial
#define         DEBUG_SWITCH               true // Show switch debug info
#define         DEBUG_CALIBRATE            true   // Show calibrate debug info

float           co2High                    = 1400.0;    // a high PPM value
float           co2Low                     = 800.0;     // a low PPM value
float           co2IntegralMax             = 2160000;   // This is 1200 ppm for 30 min in terms of ppm S (1200 x 30 x 60 = 2160000)

const uint8_t   SW_TX                      = D1;    // For software serial
const uint8_t   SW_RX                      = D2;    // For software serial

const uint8_t   SW_SDA                     = D7;    // Software I2C for sensors - also encoder!!
const uint8_t   SW_SCK                     = D6;    // Software I2C for sensors - also encoder!!

const uint8_t   ROT_A_PIN2                 = D7;
const uint8_t   ROT_B_PIN2                 = D6;
const uint8_t   ROT_PUSH_PIN2              = D8;

const uint8_t   CBOLED_SDA_PIN             = D5;
const uint8_t   CBOLED_SCK_PIN             = D4;
#define         CBOLED_CLASS               U8G2_SSD1306_128X64_NONAME_F_SW_I2C
#define         CBOLED_MESSAGE_FONT_8PT    u8g2_font_helvR08_tf
#define         CBOLED_MESSAGE_FONT_24PT   u8g2_font_luBS24_tn

const uint8_t   RGBLED_DATA_PIN            = D3;
const uint8_t   RGBLED_COUNT               = 5;
const uint8_t   RGBLED_BRIGHTNESS          = 100;
#define         RGBLED_TYPE                (NEO_GRB + NEO_KHZ800)

#define         DATA_AVERAGE               30     // The time between uploads in seconds
#define         GRAPH_AVERAGE              60     // The time between updates of the graph display in seconds
#define         DISPLAY_UPDATE             5000   // Time in mS between reading data and displaying new values
#define         WARM_UP_TIME               30     // Time in seconds to warm up...

#define         NUMBER_SENSORS             1
