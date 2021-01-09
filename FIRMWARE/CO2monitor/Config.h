#pragma once

// General Function Libraries - Already installed in Arduino IDE
#include <Arduino.h>
#include <stdint.h>
#include <Millis.h>
#include <EEPROM.h>        // For writing values to the EEPROM

//Adafruit IO info
//https://io.adafruit.com/api/docs/mqtt.html#mqtt-connection-details
#define AIO_SERVER                         "io.adafruit.com"            // direct
#define AIO_SERVERPORT                     8883                         // Use 8883 if at all possible! For secure conenction
#define AIO_USERNAME                      "YOUR ADAFRUIT IO USERNAME"   // This is your Adafruit IO username
#define AIO_KEY                           "YOUR ADAFRUIT IO KEY"        // This is your Adafruit IO Key

#define         DEBUG_GRAPH                1 // Show graph updates on serial
#define         DEBUG_MQTT                 1 // Show MQTT updates on serial
#define         DEBUG_ENCODER              1 // Show Encoder updates on serial
#define         DEBUG_CO2                  1 // Show CO2 updates sensor on serial

const uint8_t   SW_TX                      = D1;    // For software serial
const uint8_t   SW_RX                      = D2;    // For software serial

const uint8_t   SW_SDA                      = D7;    // Software I2C for sensors - also encoder!!
const uint8_t   SW_SCK                      = D6;    // Software I2C for sensors - also encoder!!

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
#define         WARM_UP_TIME               100    // Time in seconds to warm up...

#define         NUMBER_SENSORS             1
