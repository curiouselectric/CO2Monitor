
/*

   Some links:
    https://www.hackster.io/kritch83/getting-started-with-the-mh-z14a-co2-detector-e96234


  This sketch demonstrates the capabilities of the pubsub library in combination
  with the ESP8266 board/library.

  It connects to WiFi (or creates a hotspot to enter in the Wifi credentials if needed)

  It displays the CO2 level in PPM

  It publishes the sensor values via MQTT to the broker in the config (adafruit IO in this example).

  CO2 sensor is a MH-Z14A CO Sensor.

  It will reconnect to the server if the connection is lost using a blocking
  reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
  achieve the same result without blocking the main loop.

  To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

  You need to include the following libraries, via the library manager in Arduino
    WiFiManager (v 0.15.0) by tzapu
    Adafruit_NeoPixel by Adafruit
    Adafruit_MQTT_Library by Adafruit
    U8g2 by Oliver
    Button2 by Lennart Hennings
    ESP Rotary by Lennart Hennings
*/

// Config.h includes all the hardware and defines for the board
#include "Config.h"

#include "Wire.h"     // This is also needed for the OLED screen

#include <SoftwareSerial.h>
SoftwareSerial co2Serial(SW_RX, SW_TX); // RX, TX

byte cmd[9]     = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};  // get gas command
byte cmdCal[9]  = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};  // calibrate command
char response[9];  // holds the recieved data

// ************** WIFI Libraries ************************************
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

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
//static const char *fingerprint PROGMEM = "77 00 54 2D DA E7 D8 03 27 31 23 99 EB 27 DB CB A5 4C 57 18";
static const char *fingerprint PROGMEM = "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF";

/************* Feeds for Adafruit IO *********************************/
// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//Adafruit_MQTT_Publish fermenterTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fermenterTemp");
Adafruit_MQTT_Publish airCO2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airC02");

/******************** Sketch Code ************************************/

byte displayMode = 100;    // Holds the page to display - Start in Start-up screen
byte maxDisplayMode = 1 + (NUMBER_SENSORS * 2); // Roll around the modes Main screen (1) plus 2 for each sensor

bool updateMQTTflag = false;

bool wificonnect = false; // Flags for the display
bool mqttconnect = false; // Flags for the display

bool  warmUpFlag = true;  // Want to warm the unit up
int warmUpCounter = 0;

float co2ppm;      // Holds the CO2 PPM value
float co2High   = 1000.0;
float co2Low    = 400.0;

int   AveCounter    = 0;  // Holds the number of samples read to create the average value
int   PMAveCounter  = 0; // Only average the PM values when they are updated

float co2Max = -9999.9;    // Start low
float co2Min = 9999.9;     // Start High
float co2Ave = 0.0;

int sensorMQTTSend  = 0;  // This is the sensor to send to MQTT 1/number of sensors = time between each MQTT write

long int dataCounterTime = DATA_AVERAGE * 1000;   // Holds mS until next data upload
long int graphCounterTime = GRAPH_AVERAGE * 1000;  // Holds ms until next graph point
long int displayCounterTime = DISPLAY_UPDATE;   // How often to check data and update display (mS)

// Graph drawing inputs
#define sizeOfBuffer 120
float co2Buffer[sizeOfBuffer];  // Sets up a buffer of floats for displaying data

int startx = 2;          // For drawing the graphs (bottom left corner)
int starty = 46;          // For drawing the graphs (bottom left corner)
float graphHeight = 30.0;     // Height of graph (its 120 pixels wide, 64 pixels high)
float graphHeightMaxValue = 30.0;     // Temp Graph Maximum of the int values to be displayed
float graphHeightMinValue = 10.0;     // Temp Graph Minimum of the int values to be displayed
float graphCO2HeightMaxValue = 1500;     // PM2.5 Graph Maximum of the int values to be displayed
float graphCO2HeightMinValue = 300;     // PM2.5 Graph Minimum of the int values to be displayed

float graphCalculatedY;   // for doing calculations

ESPRotary r = ESPRotary(ROT_A_PIN2, ROT_B_PIN2, (int)4);
Button2 b = Button2(ROT_PUSH_PIN2);

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(10);

  //Wire.begin(SW_SDA, SW_SCK); // Start the I2C comms with different pins for the sensors

  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // Show display
  updateScreen(displayMode, wificonnect, mqttconnect);

  // Sort out Wifi
  setup_wifi();

  displayMode = 101;
  updateScreen(displayMode, wificonnect, mqttconnect); // Show if connected or not

  // check the fingerprint of io.adafruit.com's SSL cert
  client.setFingerprint(fingerprint);

  // Init the RotaryInput object
  r.setChangedHandler(rotate);
  b.setLongClickHandler(click);

  // Initialise the temperature Buffers
  for (int i = 0; i < sizeOfBuffer; i++)
  {
    co2Buffer[i] = 0;
  }
  co2Serial.begin(9600);  // Start the radiation sensor recording

  // Want to warm up the sensor - takes 5 mins (300 seconds) or so!!
  Serial.print("Warming up");

  displayMode = EEPROM.read(10);  // After showing - update to mode

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("CO2MONITOR")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();  // Reset of cannot connect - got to AP mode
    delay(5000);
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  wificonnect = true;
}

uint32_t x = 0;
float value = 0.0;  // Holds the temperature value to display and to send to AdafruitIO

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
    Serial.print(".");
    if (warmUpCounter > WARM_UP_TIME)
    {
      Serial.println("Warm!!!");
      warmUpFlag = false;
    }
  }

  // ****** Update the display *********************
  if (millis() >= (displayCounterTime + (DISPLAY_UPDATE)) && warmUpFlag == false)
  {
    //Serial.println("Update Display");
    displayCounterTime = millis(); // Store new value for next time

    // Check the CO2 sensor here:
    //co2ppm = 345.67;    // Test line!!
    co2ppm = (float)getReadings();
    if (DEBUG_CO2 == 1)
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
    Serial.println("Update Graph");
    graphCounterTime = millis(); // Store new value for next time
    // Sort out the display buffer
    for (int z = (sizeOfBuffer - 2); z >= 0; z--)
    {
      // Shift all the values along
      co2Buffer[z + 1] = co2Buffer[z];
    }
    // Add the new average values
    co2Buffer[0] = co2Ave / AveCounter;

    if (DEBUG_GRAPH == 1)
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
  if (millis() >= (dataCounterTime + ((DATA_AVERAGE * 1000) / NUMBER_SENSORS)) && warmUpFlag == false)
  {
    Serial.println("Send MQTT");
    dataCounterTime = millis(); // Store new value for next time
    // Only do this when time is over the next update
    // *********** MQTT SEND VALUE(S) ***************
    // Want this to be non-blocking send data every alternate 2 seconds (200*10ms)
    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();

    switch (sensorMQTTSend)
    {
      case 0:
        // Send CO2 data
        if (DEBUG_MQTT == 1)
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
          if (DEBUG_MQTT == 1)
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
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  mqttconnect = true;
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
      checkLEDs(co2ppm, 400, 350);
      break;

    case 2:
      u8g2.setCursor(0, 10);
      u8g2.print(F("MIN:"));
      u8g2.setCursor(32, 10);
      u8g2.print(co2Min);
      u8g2.setCursor(64, 10);
      u8g2.print(F("MAX:"));
      u8g2.setCursor(96, 10);
      u8g2.print(co2Max);
      // Draw the main temp in BIG in the middle
      u8g2.setCursor(0, 36);
      u8g2.print(F("CO2:"));
      // Want to adjust the font here:
      u8g2.setFont(CBOLED_MESSAGE_FONT_24PT);  // choose a suitable font
      u8g2.setCursor(38, 50);
      dtostrf(co2ppm, 3, 0, _buffer);  // 0 DP
      u8g2.print(_buffer);
      u8g2.setFont(CBOLED_MESSAGE_FONT_8PT);  // Font back to normal!
      checkLEDs(co2ppm, 400, 350);
      break;
    case 3:
      // Show temp 1 as a bar chart over time
      u8g2.setCursor(0, 10);
      u8g2.print(F("CO2:"));
      u8g2.setCursor(64, 10);
      u8g2.print(co2ppm);
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
      checkLEDs(co2ppm, 400, 350);
      break;
    case 99:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
      u8g2.setCursor(0, 10);
      u8g2.print(F("MODE SAVED!"));
      displayMode = EEPROM.read(10);  // After showing - update to mode
      break;
    case 100:
      // This is the case when the EEPROM has been saved
      // Displays this screen for a bit!
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

  //  // Decomment this to show the displayMode for debugging
  //  dtostrf(_mode, 7, 0, _buffer);
  //  u8g2.setCursor(100, 64);
  //  u8g2.print(_buffer);

  u8g2.sendBuffer();  // Write all the display data to the screen
}

void checkLEDs(float _temp, float _tempHigh, float _tempLow)
{
  // This lights the LEDs in different colours depending upon high/low setpoints
  if (_temp > _tempHigh)
  {
    // Show red as too warm
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 255, 0, 0);
    }
  }
  else   if (_temp < _tempLow)
  {
    // Show Blue as too cold
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 0, 0, 255);
    }
  }
  else
  {
    //  Show green as Just right
    for (int i = 0; i < RGBLED_COUNT; i++)
    {
      pixels.setPixelColor(i, 0, 255, 0);
    }
  }
  pixels.setBrightness(RGBLED_BRIGHTNESS);
  pixels.show();
}

// ****** ENCODER & BUTTON FUNCTIONS *****************
// on change of encoder
void rotate(ESPRotary & r)
{
  if (DEBUG_ENCODER == 1)
  {
    Serial.println(r.directionToString(r.getDirection()));
    Serial.print("Position: ");
    Serial.println(r.getPosition());
  }
  if (r.directionToString(r.getDirection()) == "RIGHT")
  {
    displayMode++;
    if (displayMode > maxDisplayMode)
    {
      displayMode = 1;
    }
  }
  else if (r.directionToString(r.getDirection()) == "LEFT")
  {
    displayMode--;
    if (displayMode <= 0)
    {
      displayMode = maxDisplayMode;
    }
  }
  updateScreen(displayMode, wificonnect, mqttconnect);
}

// long click of button
void click(Button2 & btn)
{
  //Store starting displayMode to EEPROM with long press
  EEPROM.write(10, displayMode);  // this writes a good value to it
  EEPROM.commit();
  Serial.println("MODE Saved");
  displayMode = 99; // Show the 'saved' screen
  updateScreen(displayMode, wificonnect, mqttconnect);
}

// Talk to the CO2 sensor and get the data!!
long int getReadings()
{
  if (DEBUG_CO2 == 1)
  {
    Serial.println("Reading CO2 Sensor");
  }

  while (co2Serial.available())  // this clears out any garbage in the RX buffer
  {
    int garbage = co2Serial.read();
  }
  co2Serial.write(cmd, 9);  // Sent out read command to the sensor
  co2Serial.flush();  // this pauses the sketch and waits for the TX buffer to send all its data to the sensor

  while (!co2Serial.available())  // this pauses the sketch and waiting for the sensor responce
  {
    delay(2);
  }
  co2Serial.readBytes(response, 9);  // once data is avilable, it reads it to a variable

  int responseHigh  = (int)response[2];
  int responseLow   = (int)response[3];
  long int CO2ppmVALUE = (256 * responseHigh) + responseLow;

  if (DEBUG_CO2 == 1)
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
  Serial.println("Calibrating....Please wait");
  co2Serial.write(cmdCal, 9);
  delay(3000);
}
