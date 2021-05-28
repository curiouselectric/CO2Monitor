/*
   CO2 Monitor for the ESP8266

   Written by: Matt Little (based on lots of other peoples work!)
   Date: Around May 2021
   Feel free to share, but please with accreditation.


  This unit reads a serial conencted CO2 sensor (see instructions for wiring)


  It connects to WiFi (or creates a hotspot to enter in the Wifi credentials if needed)

  It displays the CO2 level in PPM

  It publishes the sensor values via MQTT to tadafruit IO in this example.

  CO2 sensor is a MH-Z14A CO Sensor.

  It will reconnect to the server if the connection is lost using a blocking
  reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
  achieve the same result without blocking the main loop.

  This sketch demonstrates the capabilities of the pubsub library in combination
  with the ESP8266 board/library.

  To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board" (Use: "NodeMCU 1.0 (ESP-12E Module)")

  You need to include the following libraries, via the library manager in Arduino
    WiFiManager (v 0.15.0)  by tzapu
    Adafruit_NeoPixel       by Adafruit
    Adafruit_MQTT_Library   by Adafruit
    U8g2                    by Oliver
    Button2                 by Lennart Hennings
    ESP Rotary              by Lennart Hennings
    ArduinoJson             V6.0 https://arduinojson.org/ or install via Arduino library manager

  DONE:
  Wifi connection - show IP and SSID and PW - DONE 19/5/2021
  LEDs Off until warm - DONE 19/5/2021
  Colours for LEDs Green->yellow->red - DONE 19/5/2021. Configuable?
  Get Adafriut IO details in config screen via wifi - DONE 22/5/2021
  Create AP after long press and hold of the encoder - to adjust ADAFRUIT IO Settings - DONE 22/5/2021
  WDT to reset if no MQTT? OR need to handle this error if it happens. - Sorted 28/5/2021
  Lights On or Off - Sorted 28/5/2021

  TO DO
  Rate of change of data? - display this?
  Integral of the data?
  Warm up task bar fill - Not needed

  Interesting links:
    https://www.hackster.io/kritch83/getting-started-with-the-mh-z14a-co2-detector-e96234

*/

// Config.h includes all the hardware and defines for the board
#include "Config.h"
#include "Wire.h"     // This is also needed for the OLED screen

#include <SoftwareSerial.h>
SoftwareSerial co2Serial(SW_RX, SW_TX); // RX, TX

byte cmd[9]     = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // get gas command
byte cmdCal[9]  = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // calibrate command
char response[9];  // holds the recieved data

#include <FS.h>
// Now support ArduinoJson 6.0.0+ ( tested with v6.14.1 )
#include <ArduinoJson.h>      // get it from https://arduinojson.org/ or install via Arduino library manager

// ************** WIFI Libraries ************************************
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>

#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
// ESP_wfifmanager
//#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager
#include <WiFiClientSecure.h>
// For storing the cutsom settings:
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// ********** For the RGB LEDS (Neopixels) *************************
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(RGBLED_COUNT, RGBLED_DATA_PIN , RGBLED_TYPE);

// ********** For the I2C OLED Screen and Graphics *****************
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
CBOLED_CLASS u8g2(U8G2_R0, CBOLED_SCK_PIN, CBOLED_SDA_PIN, U8X8_PIN_NONE);

// *********** For the BUTTONS AND ENCODERS *************************
#include "Button2.h"; //  https://github.com/LennartHennigs/Button2
#include "ESPRotary.h";

// ********* For the ADAFRUIT IO MQTT CONNECTION **********************/
// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
// io.adafruit.com SHA1 fingerprint
static const char *fingerprint PROGMEM = "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF";

/******************** Sketch Code ************************************/

byte displayMode = 100;    // Holds the page to display - Start in Start-up screen
byte maxDisplayMode = 2 + (NUMBER_SENSORS * 8); // Roll around the modes Main screen (1) plus 7 for each sensor (all the options)

bool updateMQTTflag = false;

bool wificonnect = false; // Flags for the display
bool mqttconnect = false; // Flags for the display

bool  warmUpFlag = true;  // Want to warm the unit up
int   warmUpCounter = 0;

float co2ppm;             // Holds the CO2 PPM value

float co2Integral = 0;

int   AveCounter    = 0;  // Holds the number of samples read to create the average value
int   PMAveCounter  = 0; // Only average the PM values when they are updated

float co2Max = -9999.9;    // Start low
float co2Min = 9999.9;     // Start High
float co2Ave = 0.0;

int sensorMQTTSend  = 0;  // This is the sensor to send to MQTT 1/number of sensors = time between each MQTT write

long int dataCounterTime    = DATA_AVERAGE * 1000;    // Holds mS until next data upload
long int graphCounterTime   = GRAPH_AVERAGE * 1000;   // Holds ms until next graph point
long int displayCounterTime = DISPLAY_UPDATE;         // How often to check data and update display (mS)

// Graph drawing inputs
#define sizeOfBuffer 120
float co2Buffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data

int   startx = 2;                         // For drawing the graphs (bottom left corner)
int   starty = 46;                        // For drawing the graphs (bottom left corner)
float graphHeight = 30.0;                 // Height of graph (its 120 pixels wide, 64 pixels high)
float graphHeightMaxValue = 30.0;         // Temp Graph Maximum of the int values to be displayed
float graphHeightMinValue = 10.0;         // Temp Graph Minimum of the int values to be displayed
float graphCO2HeightMaxValue = 1800;      // PM2.5 Graph Maximum of the int values to be displayed
float graphCO2HeightMinValue = 350;       // PM2.5 Graph Minimum of the int values to be displayed

float graphCalculatedY;   // for doing calculations

ESPRotary r = ESPRotary(ROT_A_PIN2, ROT_B_PIN2, (int)4);
Button2 b = Button2(ROT_PUSH_PIN2, INPUT_PULLUP, false, false);
//Button2(byte attachTo, byte buttonMode = INPUT_PULLUP, boolean isCapacitive = false, boolean activeLow = true);


// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

uint32_t x = 0;
float value = 0.0;  // Holds the temperature value to display and to send to AdafruitIO

bool shouldSaveConfig = false;      //flag for saving data
bool lights_on_flag = true;         // This controls if the lights are on or off. Start ON.
bool adjust_settings_flag = false;  // This lets us enter an adjustment mode

float ROC_previous_value = 0;           // Holds the previous CO2 data for doing rate of change calculations

//callback notifying us of the need to save config
void saveConfigCallback ()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(100);

  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // Show display
  updateScreen(displayMode, wificonnect, mqttconnect);

  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

  // ******** Sort out stored data from SPIFFS *********
  // From: https://randomnerdtutorials.com/wifimanager-with-esp8266-autoconnect-custom-parameter-and-manage-your-ssid-and-password/
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument jsonBuffer(1024);                       // Changed (renamed) for ArduinoJson 6
        //JsonObject json = deserializeJson(jsonBuffer, buf.get());   //jsonBuffer.parseObject(buf.get());    // Changed (removed &) for ArduinoJson 6
        DeserializationError error = deserializeJson(jsonBuffer, buf.get());
        serializeJson(jsonBuffer, Serial);                                // Changed (https://arduinojson.org/v6/doc/upgrade/) for ArduinoJson 6
        if (!error) {
          Serial.println("\nparsed json");
          //strcpy(output, jsonBuffer["output"]);
          strcpy(AIO_USERNAME, jsonBuffer[AIO_USERNAME_Label]);
          strcpy(AIO_KEY, jsonBuffer[AIO_API_KEY_Label]);
          strcpy(AIO_feed_name, jsonBuffer[AIO_feed_name_Label]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // Sort out Wifi - dont create AP if we dont need to (set false)
  setup_wifi(false);

  displayMode = 101;    // Startup mode to show warming up!
  updateScreen(displayMode, wificonnect, mqttconnect); // Show if connected or not

  // check the fingerprint of io.adafruit.com's SSL cert
  client.setFingerprint(fingerprint);

  // Init the RotaryInput object
  r.setChangedHandler(rotate);
  b.setLongClickHandler(longpress);

  // Initialise the temperature Buffers
  for (int i = 0; i < sizeOfBuffer; i++)
  {
    co2Buffer[i] = 0;
  }
  co2Serial.begin(9600);  // Start the radiation sensor recording
  // Want to warm up the sensor - takes 5 mins (300 seconds) or so!!
  Serial.print("Warming up");

  // Read in the various values for min and max settings. Need 4 bytes for each of these: floats.
  EEPROM.get(2, WARM_UP_TIME);
  EEPROM.get(10, co2High);
  EEPROM.get(20, co2Low);
  EEPROM.get(30, co2IntegralMax);

  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

}

void setup_wifi(bool start_AP) {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  // Add custom parameters
  WiFiManagerParameter custom_AIO_usermame(AIO_USERNAME_Label, "User_Name", AIO_USERNAME, AIO_USERNAME_LEN);
  WiFiManagerParameter custom_AIO_API_KEY(AIO_API_KEY_Label, "API_KEY",  AIO_KEY, AIO_API_KEY_LEN);
  WiFiManagerParameter custom_AIO_feed_name(AIO_feed_name_Label, "Feed_Name", AIO_feed_name,  AIO_feed_name_LEN);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //add all your parameters here
  wifiManager.addParameter(&custom_AIO_usermame);
  wifiManager.addParameter(&custom_AIO_API_KEY);
  wifiManager.addParameter(&custom_AIO_feed_name);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep (in seconds)
  wifiManager.setTimeout(AP_TIMEOUT_SEC);

  if (wificonnect == true)
  {
    wifiManager.disconnect();
    wificonnect = false;
  }
  //fetches ssid and pass and tries to connect

  if (!wifiManager.autoConnect() || start_AP == true)
  {
    // If there is no stored SSID then start an access point
    u8g2.clearBuffer();
    u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // choose a suitable font
    u8g2.setCursor(0, 10);
    u8g2.print(F("SET WIFI DETAILS"));
    u8g2.setCursor(0, 30);
    u8g2.print(F("SSID: "));
    u8g2.setCursor(32, 30);
    u8g2.print(AP_SSID);
    u8g2.setCursor(0, 40);
    u8g2.print(F("PW: "));
    u8g2.setCursor(32, 40);
    u8g2.print(AP_PASS);
    u8g2.sendBuffer();  // Write all the display data to the screen

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal(AP_SSID, AP_PASS))
    {
      Serial.println("Not connected to WiFi but continuing anyway.");
      // At this point we are not connecting to WiFi and will not update Adafruit IO
    }
  }
  if (wifiManager.autoConnect())
  {
    //if you get here you have connected to the WiFi
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    wificonnect = true;
    // At this point we have the correct SSID and PW configured and start the wifi

    // Here we want to get the values from the custom fields:
    strcpy(AIO_USERNAME, custom_AIO_usermame.getValue());
    strcpy(AIO_KEY, custom_AIO_API_KEY.getValue());
    strcpy(AIO_feed_name, custom_AIO_feed_name.getValue());

    //save the custom parameters to FS
    if (shouldSaveConfig) {
      Serial.println("saving config");
      DynamicJsonDocument jsonBuffer(1024);     // Changed (renamed) for ArduinoJson 6
      jsonBuffer[AIO_USERNAME_Label] = AIO_USERNAME ;
      jsonBuffer[AIO_API_KEY_Label] = AIO_KEY;
      jsonBuffer[AIO_feed_name_Label] = AIO_feed_name;

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      serializeJson(jsonBuffer, Serial); // Changed (https://arduinojson.org/v6/doc/upgrade/) for ArduinoJson 6
      serializeJson(jsonBuffer, configFile); // Changed (https://arduinojson.org/v6/doc/upgrade/) for ArduinoJson 6
      configFile.close();
      //end save
    }
  }
}

void loop() {
  // This is the main loop
  // Check the input encoder and button:
  r.loop();
  b.loop();

  if (warmUpFlag == true)
  {
    // In this case we want to warm up the sensor.
    warmUpCounter++;
    delay(1000);   // 1 second wait
    r.loop();
    b.loop();
    Serial.print(".");
    // Update the display with a count down timer:
    warmupTimerScreen((WARM_UP_TIME - warmUpCounter + 1), wificonnect, mqttconnect);

    if (warmUpCounter > WARM_UP_TIME)
    {
      Serial.println("Warm!!!");
      warmUpFlag = false;
      displayMode = EEPROM.read(1);  // After showing - update to mode
    }
    // Keep the LEDs OFF:
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  // ****** Update the display *********************
  if (millis() >= (displayCounterTime + (DISPLAY_UPDATE)) && warmUpFlag == false)
  {
    //Serial.println("Update Display");
    displayCounterTime = millis(); // Store new value for next time

    // Check the CO2 sensor here:
    co2ppm = (float)getReadings();
    if (DEBUG_CO2 == true)
    {
      Serial.print("CO2 data is: ");
      Serial.println(co2ppm);
    }
    // Here we do some statistics on the data:
    if (co2ppm > co2Max)
    {
      co2Max = co2ppm ;
    }
    if (co2ppm  < co2Min)
    {
      co2Min = co2ppm ;
    }
    co2Ave += co2ppm ;
    AveCounter++;

    // ****** DISPLAY THE VALUE **********************
    // Here want to update the display with the value:
    updateScreen(displayMode, wificonnect, mqttconnect);
  }

  // ******* graph buffer update *************
  // Only do this when over the graph update time
  if (millis() >= (graphCounterTime + (GRAPH_AVERAGE * 1000)) && warmUpFlag == false)
  {
    graphCounterTime = millis(); // Store new value for next time
    // Sort out the display buffer
    for (int z = (sizeOfBuffer - 2); z >= 0; z--)
    {
      // Shift all the values along
      co2Buffer[z + 1] = co2Buffer[z];
    }
    // Add the new average values
    co2Buffer[0] = co2Ave / AveCounter;

    if (DEBUG_GRAPH == true)
    {
      Serial.print("Graph Updated: ");
      Serial.print("CO2 Ave: ");
      Serial.println(co2Buffer[0]);
    }
    // reset the averages
    co2Ave = 0;
    AveCounter = 0;
  }
  // ********** End graph update **************

  // Send the MQTT data - This is done at intervals: DATA_AVERAGE / NUMBER_SENSORS
  if (millis() >= (dataCounterTime + ((DATA_AVERAGE * 1000) / NUMBER_SENSORS)) && warmUpFlag == false && wificonnect == true)
  {

    dataCounterTime = millis(); // Store new value for next time
    // Only do this when time is over the next update
    // *********** MQTT SEND VALUE(S) ***************
    // Want this to be non-blocking send data every alternate 2 seconds (200*10ms)
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.

    /************* Feeds for Adafruit IO *********************************/
    // Setup a feed called 'test' for publishing.
    // Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
    //eg: Adafruit_MQTT_Publish fermenterTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fermenterTemp");

    char feed_name[80];  // =   AIO_USERNAME  "/feeds/airC02" ;
    strcpy(feed_name, AIO_USERNAME);
    strcpy(feed_name + strlen(feed_name ), "/feeds/");
    strcpy(feed_name + strlen(feed_name), AIO_feed_name);
    Serial.print(F("Feed Name: "));
    Serial.println(feed_name);
    Adafruit_MQTT_Publish airCO2 = Adafruit_MQTT_Publish(&mqtt, feed_name);

    MQTT_connect();

    switch (sensorMQTTSend)
    {
      case 0:
        // Send CO2 data
        if (DEBUG_MQTT == true)
        {
          Serial.print(F("\nSending val "));
          Serial.print(co2ppm);
          Serial.print(F(" to airCO2 feed..."));
        }
        if (! airCO2.publish(co2ppm)) {
          if (DEBUG_MQTT == 1)
          {
            Serial.println(F("Failed"));
          }
        } else {
          if (DEBUG_MQTT == true)
          {
            Serial.println(F("OK!"));
          }
        }
        break;
    }
    sensorMQTTSend++;
    if (sensorMQTTSend >= NUMBER_SENSORS)
    {
      sensorMQTTSend = 0; // Reset to zero
    }
  }
  // ******* END MQTT UPLOAD ********
  delay(10);  //Short delay to slow it all down!
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()
{
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    mqttconnect = true;
    return;
  }
  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqttconnect = false;
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0)
    {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  mqttconnect = true;
}

void warmupTimerScreen(int _warmupTimeS, bool _wificonnect, bool _mqttconnect)
{
  // This routine displays a count down for the warm up timer
  u8g2.clearBuffer();
  u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // choose a suitable font
  u8g2.setCursor(0, 10);
  u8g2.print(F("WARMING UP"));
  u8g2.setCursor(100, 10);
  u8g2.print(_warmupTimeS);

  // This section draws the bit at the botoom (always there)
  if (_wificonnect == true)
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("Wifi OK"));
  }
  else
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("       "));
  }
  if (_mqttconnect == true)
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("MQTT OK"));
  }
  else
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("       "));
  }
  u8g2.sendBuffer();  // Write all the display data to the screen
}

void updateScreen(int _mode, bool _wificonnect, bool _mqttconnect)
{
  // This routine draws the basic display with all features
  // Displays if WiFi connected
  // Displays if MQTT connected
  // Dsiplays mode (for debugging)
  u8g2.clearBuffer();
  u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // choose a suitable font

  char _buffer[6];  // A holder for data to string conversions

  // Here we decide what to show depending upon the displayMode
  switch (displayMode) {
    case 1:
      // Show data as small values
      u8g2.setCursor(0, 10);
      u8g2.print(F("CO2 Unit"));
      u8g2.setCursor(0, 25);
      u8g2.print(F("CO2:"));
      u8g2.setCursor(38, 25);
      u8g2.print((int)co2ppm);
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 2:
      // Show data as small values
      u8g2.setCursor(0, 10);
      if (lights_on_flag == true)
      {
        u8g2.print(F("Lights ON"));
        checkLEDs(co2ppm, co2High, co2Low);
      }
      else
      {
        u8g2.print(F("Lights OFF"));
        //set all the LEDs off here...
        // Show red as too high
        for (int i = 0; i < RGBLED_COUNT; i++)
        {
          pixels.setPixelColor(i, 0, 0, 0);
        }
        pixels.show();
      }
      break;

    case 3:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print((int)co2Min);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print((int)co2Max);
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("CO2:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(co2ppm, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 4:
      // Show CO2 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("CO2:"));
      u8g2.setCursor(64, 10);
      u8g2.print((int)co2ppm);
      // Want to draw 120 lines from
      // startx, staryy graphHeight, graphHeightMaxValue
      for (int n = 0; n < sizeOfBuffer; n++)
      {
        if (co2Buffer[n] <= graphCO2HeightMaxValue && co2Buffer[n] >= graphCO2HeightMinValue)
        {
          graphCalculatedY = (starty - (((co2Buffer[n] - graphCO2HeightMinValue) / (graphCO2HeightMaxValue - graphCO2HeightMinValue)) * graphHeight ));
        }
        else if (co2Buffer[n] > graphCO2HeightMaxValue)
        {
          graphCalculatedY = starty - (graphHeight);
        }
        else if (co2Buffer[n] < graphCO2HeightMinValue)
        {
          graphCalculatedY = starty;
        }
        u8g2.drawLine(startx + n, starty, startx + n, (int)graphCalculatedY);
      }
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 5:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("RATE OF CHANGE"));

      //      updateROC(ROC_previous_value, co2ppm);
      //      ROC_previous_value=co2ppm;
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 6:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("INTEGRAL"));
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 7:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("Adjust Min"));
      u8g2.setCursor(0, 20);
      u8g2.print(co2Low, 0);
      u8g2.setCursor(0, 30);
      if (adjust_settings_flag == false)
      {
        u8g2.print(F("Press to adjust"));
      }
      else
      {
        u8g2.print(F("Rotate to Adjust"));
      }
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 8:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("Adjust Max"));
      u8g2.setCursor(0, 20);
      u8g2.print(co2High, 0);
      u8g2.setCursor(0, 30);
      if (adjust_settings_flag == false)
      {
        u8g2.print(F("Press to adjust"));
      }
      else
      {
        u8g2.print(F("Rotate to Adjust"));
      }
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 9:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("Adjust Integral"));
      u8g2.setCursor(0, 20);
      u8g2.print(co2IntegralMax, 0);
      u8g2.setCursor(0, 30);
      if (adjust_settings_flag == false)
      {
        u8g2.print(F("Press to adjust"));
      }
      else
      {
        u8g2.print(F("Rotate to Adjust"));
      }
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 10:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("Warm Up Time:"));
      u8g2.setCursor(0, 20);
      u8g2.print(WARM_UP_TIME, 0);
      u8g2.setCursor(0, 30);
      if (adjust_settings_flag == false)
      {
        u8g2.print(F("Press to adjust"));
      }
      else
      {
        u8g2.print(F("Rotate to Adjust"));
      }
      checkLEDs(co2ppm, co2High, co2Low);
      break;

    case 98:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
      u8g2.setCursor(0, 10);
      u8g2.print(F("LIGHTS UPDATED"));
      displayMode = EEPROM.read(1);  // After showing - update to mode
      break;

    case 99:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
      u8g2.setCursor(0, 10);
      u8g2.print(F("SAVED!"));
      displayMode = EEPROM.read(1);  // After showing - update to mode
      break;

    case 100:
      // This is the case when undefined at start
      u8g2.setCursor(0, 10);
      u8g2.print(F("START UP"));
      break;

    case 101:
      u8g2.setCursor(0, 10);
      u8g2.print(F("WARMING UP"));
      break;
  }

  // This section draws the bit at the botoom (always there)
  if (_wificonnect == true)
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("Wifi OK"));
  }
  else
  {
    u8g2.setCursor(0, 64);
    u8g2.print(F("       "));
  }
  if (_mqttconnect == true)
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("MQTT OK"));
  }
  else
  {
    u8g2.setCursor(64, 64);
    u8g2.print(F("       "));
  }

  u8g2.sendBuffer();  // Write all the display data to the screen
}

void checkLEDs(float _temp, float _tempHigh, float _tempLow)
{
  // We dont want to show any LEDs is the light flag is false:

  if (lights_on_flag == false)
  {
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else
  {
    // This lights the LEDs in different colours depending upon high/low setpoints
    // If value is > high then fully ON red LEDs?
    if (_temp > _tempHigh)
    {
      // Show red as too high
      for (int i = 0; i < RGBLED_COUNT; i++)
      {
        pixels.setPixelColor(i, 255, 0, 0);
      }
    }
    else   if (_temp < _tempLow)
    {
      // Show Blue as too low
      for (int i = 0; i < RGBLED_COUNT; i++)
      {
        pixels.setPixelColor(i, 0, 255, 0);
      }
    }
    else
    {
      //  Show green as Just right
      for (int i = 0; i < RGBLED_COUNT; i++)
      {
        pixels.setPixelColor(i, 255, 255, 0);
      }
    }
    pixels.setBrightness(RGBLED_BRIGHTNESS);
    pixels.show();
  }
}


// ****** ENCODER & BUTTON FUNCTIONS *****************
// on change of encoder
void rotate(ESPRotary & r)
{
  if (DEBUG_ENCODER == true)
  {
    Serial.println(r.directionToString(r.getDirection()));
    Serial.print("Position: ");
    Serial.println(r.getPosition());
  }
  if (r.directionToString(r.getDirection()) == "RIGHT")
  {
    if (adjust_settings_flag == false)
    {
      displayMode++;
      if (displayMode > maxDisplayMode)
      {
        displayMode = 1;
      }
    }
    else
    {
      if (displayMode == 7)
      {
        co2Low = co2Low + INC_VALUE;
        if (co2Low > MAX_VALUE)
        {
          co2Low = MAX_VALUE;
        }
      }
      if (displayMode == 8)
      {
        co2High = co2High + INC_VALUE;
        if (co2High > MAX_VALUE)
        {
          co2High = MAX_VALUE;
        }
      }
      if (displayMode == 9)
      {
        co2IntegralMax  = co2IntegralMax  + INC_VALUE;
        if (co2IntegralMax > MAX_VALUE)
        {
          co2IntegralMax = MAX_VALUE;
        }
      }
      if (displayMode == 10)
      {
        WARM_UP_TIME  = WARM_UP_TIME  + 1;
      }
    }
  }
  else if (r.directionToString(r.getDirection()) == "LEFT")
  {
    if (adjust_settings_flag == false)
    {
      displayMode--;
      if (displayMode <= 0)
      {
        displayMode = maxDisplayMode;
      }
    }
    else
    {
      if (displayMode == 7)
      {
        co2Low =  co2Low - INC_VALUE;
        if (co2Low <= 0)
        {
          co2Low = 0;
        }
      }
      if (displayMode == 8)
      {
        co2High =  co2High - INC_VALUE;
        if (co2High <= 0)
        {
          co2High = 0;
        }
      }
      if (displayMode == 9)
      {
        co2IntegralMax  =  co2IntegralMax - INC_VALUE;
        if (co2IntegralMax  <= 0)
        {
          co2IntegralMax  = 0;
        }
      }
      if (displayMode == 10)
      {
        WARM_UP_TIME  = WARM_UP_TIME  - 1;
        if (WARM_UP_TIME  <= 0)
        {
          WARM_UP_TIME = 0;
        }
      }
    }
  }
  updateScreen(displayMode, wificonnect, mqttconnect);
}

// long click of button
void longpress(Button2& btn)
{
  // Highlight we are in this mode with the LEDs
  for (int i = 0; i < RGBLED_COUNT; i++)
  {
    pixels.setPixelColor(i, 0, 0, 255);
  }
  pixels.setBrightness(RGBLED_BRIGHTNESS);
  pixels.show();

  unsigned int time = btn.wasPressedFor();

  if (time > 3000) {
    if (DEBUG_SWITCH == true)
    {
      Serial.println(F("Start AP Mode to change Config"));
    }
    //Start the unit in AP mode
    setup_wifi(true);
  }
  else if (time > 300)
  {
    if (displayMode == 2)
    {
      lights_on_flag = !lights_on_flag;
      displayMode = 98; // Show the 'lights' screen
    }
    else if (displayMode == 7 || displayMode == 8 || displayMode == 9 || displayMode == 10 )
    {
      if (adjust_settings_flag == true)
      {
        // Save the values to EEPROM
        EEPROM.put(10, co2Low);
        EEPROM.put(20, co2High);
        EEPROM.put(30, co2IntegralMax);
        EEPROM.put(2, WARM_UP_TIME);
        EEPROM.write(1, displayMode);  // this writes a good value to it
        EEPROM.commit();
        displayMode = 99;
      }
      adjust_settings_flag = !adjust_settings_flag;  // Toggle the adjustment flag
      Serial.print(F("Adjust Flag = "));
      Serial.println(adjust_settings_flag);
    }
    else
    {
      //Store starting displayMode to EEPROM with press
      EEPROM.write(1, displayMode);  // this writes a good value to it
      EEPROM.commit();
      if (DEBUG_SWITCH == true)
      {
        Serial.println(F("MODE Saved"));
      }
      displayMode = 99; // Show the 'saved' screen
    }
    updateScreen(displayMode, wificonnect, mqttconnect);
  }
  else
  {
    if (DEBUG_SWITCH == true)
    {
      Serial.println(F("Press Too Short"));
    }
  }
}

// Talk to the CO2 sensor and get the data!!
long int getReadings()
{
  if (DEBUG_CO2 == true)
  {
    Serial.println(F("Reading CO2 Sensor"));
  }
  while (co2Serial.available())  // this clears out any garbage in the RX buffer
  {
    int garbage = co2Serial.read();
  }
  if (DEBUG_CO2 == true)
  {
    Serial.println(F("Cleared Garbage"));
    Serial.print(F("Wait for response"));
  }
  co2Serial.write(cmd, 9);  // Sent out read command to the sensor
  co2Serial.flush();  // this pauses the sketch and waits for the TX buffer to send all its data to the sensor
  int timeout = 0;
  while (!co2Serial.available())  // this pauses the sketch and waiting for the sensor responce
  {
    timeout++;
    delay(20);
    if (DEBUG_CO2 == true)
    {
      Serial.print(F("."));
    }
    if (timeout > 50)
    {
      if (DEBUG_CO2 == true)
      {
        Serial.println();
        Serial.print(F("ERROR: No response"));
      }
      return (0);
    }
  }
  if (DEBUG_CO2 == true)
  {
    Serial.println();
    Serial.print(F("Requesting Data"));
  }
  co2Serial.readBytes(response, 9);  // once data is avilable, it reads it to a variable
  int responseHigh  = (int)response[2];
  int responseLow   = (int)response[3];
  long int CO2ppmVALUE = (256 * responseHigh) + responseLow;
  if (DEBUG_CO2 == true)
  {
    Serial.print("Low: ");
    Serial.print(responseLow);
    Serial.print(" High: ");
    Serial.print(responseHigh);
    Serial.print(" Data: ");
    Serial.println(CO2ppmVALUE);
  }
  return (CO2ppmVALUE);
}

void calibrate()
{
  if (DEBUG_CALIBRATE == true)
  {
    Serial.println("Calibrating....Please wait");
  }
  co2Serial.write(cmdCal, 9);
  delay(3000);
}
