// Battery and tank level display system using E-paper display and INA3221 three-channel voltage/current sensors
// for battery sensors, and an ADC1115 4 channel ADC for tank level.
// 
// Based on e-ink display library by Jean-Marc Zingg and SignalK UDP sender by PaddyB
//
// by Andy Barrow (andy@sailor.nu)
//
// mapping suggestion for ESP32, e.g. LOLIN32, see .../variants/.../pins_arduino.h for your board
// NOTE: there are variants with different pins for SPI ! CHECK SPI PINS OF YOUR BOARD
// BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

#define ENABLE_GxEPD2_GFX 1
#define BATTERY_DISPLAY 0
#define TANK_DISPLAY 1

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "GxEPD2_boards_added.h"
#include <INA.h>
#include <Adafruit_ADS1015.h>
#include <WiFi.h>
//this is version 5 of ArduinoJson. Some day I'll port to version 6 ....
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <time.h>
#include "lwip/apps/sntp.h"

/**************************************************************************************************
** Declare program constants, global variables and instantiate INA class                         **
**************************************************************************************************/
const uint32_t SERIAL_SPEED    = 115200;  ///< Use fast serial speed
const uint32_t SHUNT_MICRO_OHM = 375;     ///< Shunt resistance in Micro-Ohm, this is a 75mV / 200A shunt
const uint16_t MAXIMUM_AMPS    = 200;     ///< Max expected amps, values are 1 - clamped to max 1022
uint8_t        devicesFound    = 0;       ///< Number of INAs found
INA_Class      INA;                       ///< INA class instantiation

// Battery monitoring with two INA3221 devices, using two channels each. They are detected here is device numbers,
// One of the devices needs to have the I2C default address changed by jumper.
// You'll have to scan them with this program (uncomment below) to figure out which is which. Device numbers start at 0
const uint8_t  batt1VoltageDev = 4;
const uint8_t  batt1CurrentDev = 5;
const uint8_t  batt2VoltageDev = 1;
const uint8_t  batt2CurrentDev = 2;

const char * batt1Name = "HOUSE";
const char * batt2Name = "ENGINE";

// You'll also need to name the tanks
const char * tank1Name = " FORE";
const char * tank2Name = " STBD";

int refreshCounter = 0; //this is a global varable set up to count until a full screen refresh is needed

// Display offset for right side, in pixels
int rightOffset = 148;
const uint8_t left_screen = 0;
const uint8_t right_screen = 1;


/*********************************************************
 * WIfi and SignalK
*********************************************************/
// We'll be sending SignalK data to a server via UDP. You'll have to tell
// the SignalK server that you have a new UDP connection available
WiFiUDP udp;

// Your wifi credentials go here
const char* ssid = "openplotter";
const char* password = "margaritaville";

// The address and port of your SignalK server goes here 
IPAddress sigkserverip(10,10,10,1);
// This is the port number you need to tell your server
uint16_t sigkserverport = 55561;

byte sendSig_Flag = 1;

// SignalK keys for power from the two battery banks
const char* batt1VoltageKey = "electrical.batteries.house.voltage";
const char* batt1CurrentKey = "electrical.batteries.house.current";
const char* batt2VoltageKey = "electrical.batteries.engine.voltage";
const char* batt2CurrentKey = "electrical.batteries.engine.current";

// SignalK keys for level of the two tanks
const char* tank1LevelKey = "tanks.freshWater.forwardTank.currentLevel";
const char* tank2LevelKey = "tanks.freshWater.starboardTank.currentLevel";

/*********************************************************************************************
* Time
* Right now, the system is set up to get time just from the RPI. If you want more accurate time
* you'll need an internet connection so you can get time from pool.ntp.org, or you'll need a 
* GPS hat or other device with a poll output. You can also just  go to the RPI for time, and 
* make sure it is getting it's time from an accurate source.
* 
* SignalK has a plugin to set system time from GPS.
* *******************************************************************************************/
time_t now;
char strftime_buf[64];
struct tm timeinfo;
char ntpserver1[] = "10.10.10.1";
//char ntpserver2[] = "pool.ntp.org"; //You can get time from the net if you have a connection
const char localTimeZone[23] = "CST6CDT,M4.1.0,M10.5.0";

// This is to avoid PlatformIO Intellisense issues with time.h
_VOID _EXFUN(tzset,	(_VOID));
int	_EXFUN(setenv,(const char *__string, const char *__value, int __overwrite));

/*********************************************************
 * ADC for tank level monitoring
 * ******************************************************/
Adafruit_ADS1115 ads(0x48);

/*********************************************************
 * Touch Control
 * ******************************************************/
const uint8_t touchCtrlRight = 15;
uint8_t screen_mode = BATTERY_DISPLAY;

/*********************************************************
* Function Definitions for PlatformIO
* *******************************************************/
float * getBattDeviceData (int deviceNumber);
void drawScreenOutlineBatt();
void drawScreenOutlineTank();
void setup_wifi();
void testUDP();
void sendSigK(String sigKey, float data);
float * getTankData ();
void display_batt(float shuntAmps, float realVolts, uint8_t rightside);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);
  ads.begin();  //Start the A/D converter for tank level measurement

  // Setup Battery Monitor
  Serial.println("Looking for INA device");
  // IMPORTANT: if no INA devices are found the program will just continue to loop looking
  // for them! If you are unsure, run this with a serial monitor so you are sure you have 
  // INA sensors connected.
  devicesFound = INA.begin(
      MAXIMUM_AMPS, SHUNT_MICRO_OHM);  // Set to the expected Amp maximum and shunt resistance
  while (INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM) == 0) {
    Serial.println("No INA device found, retrying in 10 seconds...");
    delay(10000);  // Wait 10 seconds before retrying
  }                // while no devices detected
  Serial.print(" - Detected ");
  Serial.print(devicesFound);
  Serial.println(" INA devices on the I2C bus");
  INA.setBusConversion(8500);             // Maximum conversion time 8.244ms
  INA.setShuntConversion(8500);           // Maximum conversion time 8.244ms
  INA.setAveraging(32);                   // Average each reading n-times
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
  INA.alertOnBusOverVoltage(true, 15000); // Trigger alert if over 15V on bus

  // Initialize the epaper display
  display.init(115200);

  // Start with the battery display
  drawScreenOutlineBatt();

  setup_wifi();

  // Set up the ESP to retreive time from the server
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  // This assumes your RPI server has NTP running. You can use the SignalK "Set System Time" plugin to set the time
  sntp_setservername(0, ntpserver1);
  //sntp_setservername(0, ntpserver2);
  sntp_init();
}

void loop()
{
  static char busChar[8], busMAChar[10];  // Output buffers
  float shuntAmps;
  float realVolts;
  float *ina_Output;

  time(&now);
  setenv("TZ", localTimeZone, 1);
  tzset();
  localtime_r(&now, &timeinfo);

  /**************************************
   * Read Touch Control
   * ***********************************/
  Serial.print("Right Touch ");
  Serial.println(touchRead(touchCtrlRight));

  // Toggle between Battery display and Tank display
  if (touchRead(touchCtrlRight) < 30){
    Serial.println("RIGHT TOUCH");
    if (screen_mode == BATTERY_DISPLAY){
      screen_mode = TANK_DISPLAY;
      drawScreenOutlineTank();
    }
    else {
      screen_mode = BATTERY_DISPLAY;
      drawScreenOutlineBatt();
    }
  }

  /*****************************
   * Battery Bank 1
   * **************************/
  // Volts
  ina_Output = getBattDeviceData(batt1VoltageDev);
  realVolts = ina_Output[0] / 1000.0;
  sendSigK(batt1VoltageKey, realVolts); //send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  Serial.println("Battery 1");
  Serial.print("Voltage: ");
  Serial.print(busChar);

  // Amps
  ina_Output = getBattDeviceData(batt1CurrentDev);
  shuntAmps = ina_Output[1] / SHUNT_MICRO_OHM;
  sendSigK(batt1CurrentKey, shuntAmps); //send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  Serial.print(" Current: ");
  Serial.print(busMAChar);
  Serial.println();
  
  // Print it on the left side
  if (screen_mode == BATTERY_DISPLAY) {
    display_batt(shuntAmps, realVolts, left_screen);
  }

  /******************************
   * Battery Bank 2
   * ***************************/
  // Volts
  ina_Output = getBattDeviceData(batt2VoltageDev);
  realVolts = ina_Output[0] / 1000.0;
  sendSigK(batt2VoltageKey, realVolts); //send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  Serial.println("Battery 2");
  Serial.print("Voltage: ");
  Serial.print(busChar);

  // Amps
  ina_Output = getBattDeviceData(batt2CurrentDev);
  shuntAmps = ina_Output[1] / SHUNT_MICRO_OHM;
  sendSigK(batt2CurrentKey, shuntAmps); //send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  Serial.print(" Current: ");
  Serial.print(busMAChar);
  Serial.println();

  // Print it on the right side
  if (screen_mode == BATTERY_DISPLAY) {
    display_batt(shuntAmps, realVolts, right_screen);
  }

  /* Uncomment this to detect and display device numbers
  static uint16_t loopCounter = 0;     // Count the number of iterations
  static char     shuntChar[10], busMWChar[10];  // Output buffers

  //Serial.print("Nr Adr Type   Bus      Shunt Curr  Bus         Bus\n");
  //Serial.print("== === ====== ======== =========== =========== ===========\n");
  for (uint8_t i = 0; i < devicesFound; i++)  // Loop through all devices
  {
    x = getBattDeviceData(i);
    busMilliVolts = x[0];
    shuntMicroVolts = x[1];
    busMicroAmps = x[2];
    busMicroWatts = x[3];

    shuntAmps = shuntMicroVolts / SHUNT_MICRO_OHM;

    dtostrf(busMilliVolts / 1000.0, 7, 4, busChar);      // Convert floating point to char
    dtostrf(shuntAmps, 9, 4, shuntChar);  // Convert floating point to char
    dtostrf(busMicroAmps / 1000.0, 9, 4, busMAChar);     // Convert floating point to char
    dtostrf(busMicroWatts / 1000.0, 9, 4, busMWChar);    // Convert floating point to char
    sprintf(sprintfBuffer, "%2d %3d %s %sV %sA %smA %smW\n", i, INA.getDeviceAddress(i),
            INA.getDeviceName(i), busChar, shuntChar, busMAChar, busMWChar);
    Serial.print(sprintfBuffer);
  }  // for-next each INA device loop
  Serial.println();
  delay(1000);  // Wait 10 seconds before next reading
  Serial.print("Loop iteration ");
  Serial.print(++loopCounter);
  Serial.print("\n\n");*/

  refreshCounter ++;
  // Do a full screen refresh to keep the display healthy. With a B/W screen there are about 4 cycles/second, so 
  // setting this to 2400 will fully refresh the screen about every 10 minutes.
  if (refreshCounter > 2400){
    if (screen_mode == BATTERY_DISPLAY){
      drawScreenOutlineBatt();
    }  else {
      drawScreenOutlineTank();
    }
    refreshCounter = 0;
    
    return;
  }
}

// Batteries: Go and get the data from a specific device number
float * getBattDeviceData (int deviceNumber){

  static float x[4];
    
    x[0] = INA.getBusMilliVolts(deviceNumber);
    x[1] = INA.getShuntMicroVolts(deviceNumber);
    x[2] = INA.getBusMicroAmps(deviceNumber);
    x[3] = INA.getBusMicroWatts(deviceNumber);

    return x;
}

// Here were grabbing the ADC data. We're only using two of these right now, but we'll grab all four
float * getTankData () {
  
  static float x[4];

    x[0] = ads.readADC_SingleEnded(0);
    x[1] = ads.readADC_SingleEnded(1);
    x[2] = ads.readADC_SingleEnded(2);
    x[3] = ads.readADC_SingleEnded(3);

    return x;
}

// this builds the screen for the battery display
void drawScreenOutlineBatt()
{
  // Draw two side-by-side boxes with black areas at the top for titles
  display.setRotation(3); // Set to horizontal orentation
  display.setFullWindow();
  display.firstPage();
  do { //Draw two boxes on the screen with a black area at the top for titles
    display.fillScreen(GxEPD_BLACK);
    display.fillRect(2,37, ((display.width()/2) - 3), display.height() - 39, GxEPD_WHITE);
    display.fillRect((display.width() / 2)+2, 37,  (display.width()/2) - 3, display.height()-39, GxEPD_WHITE);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(12, 30);
    display.print(batt1Name);
    display.setCursor(154, 30);
    display.print(batt2Name);
  }
  while (display.nextPage());
  
  // if the device didn't find wifi before, it will just move on after 30 seconds. Here we check again for
  // wifi. This will delay reading battery voltage 30 seconds, so if you don't want that delay and don't care about
  // reconnecting WiFi, comment this out. If wifi is already connected, there will be no delay.
  setup_wifi();

  return;
}

//This builds the screen for the tank display
void drawScreenOutlineTank()
{
  // Draw two side-by-side boxes
  display.setRotation(3); // Set to horizontal orientation
  display.setFullWindow();
  display.firstPage();
  do { //Draw two boxes on the screen
    display.fillScreen(GxEPD_BLACK);
    display.fillRect(2,37, ((display.width()/2) - 3), display.height() - 39, GxEPD_WHITE);
    display.fillRect((display.width() / 2)+2, 37,  (display.width()/2) - 3, display.height()-39, GxEPD_WHITE);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(12, 30);
    display.print(tank1Name);
    display.setCursor(154, 30);
    display.print(tank2Name);
  }
  while (display.nextPage());
  
  // if the device didn't find wifi before, it will just move on after 30 seconds. Here we check again for
  // wifi. This will delay reading battery voltage 30 seconds, so if you don't want that delay and don't care about
  // reconnecting WiFi, comment this out. If wifi is already connected, there will be no delay.
  setup_wifi();

  return;
}

void display_batt(float shuntAmps, float realVolts, uint8_t rightside){
  
  static char busChar[8], busMAChar[10];  // Output buffers
  uint16_t box_x = 20;
  uint16_t box_y = 45;
  uint16_t box_w = 115;
  uint16_t box_h = 70;
  uint16_t cursor_y = box_y + box_h - 47;
  
  if (rightside){
    box_x = box_x + rightOffset;
  }

  dtostrf(realVolts, 2, 1, busChar);
  dtostrf(shuntAmps, 2, 1, busMAChar);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);
  display.firstPage();
  do
    {
      display.setPartialWindow(box_x, box_y, box_w, box_h);
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
      display.setCursor(box_x, cursor_y);
      display.print(busChar);
      display.setCursor(box_x + 80, cursor_y);
      display.print(" V");
      display.setCursor(box_x, cursor_y + 32);
      display.print(busMAChar);
      display.setCursor(box_x + 80, cursor_y + 32);
      display.print(" A");
      display.setCursor(box_x + 20, cursor_y + 50);
      display.setFont(&FreeSansBold9pt7b);

      // Print date on the left, time on the right
      if (rightside){
        strftime(strftime_buf, sizeof(strftime_buf), "%T", &timeinfo);
      } else {
        strftime(strftime_buf, sizeof(strftime_buf), "%D", &timeinfo);
      }
      display.print(strftime_buf);
    }
  while (display.nextPage());

  return;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting ");
  WiFi.begin(ssid, password);
  int reset_index = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); // this sets how long the device will wait for a wifi connection
    Serial.print(".");
    /* Decided not to do this, so the device can go ahead an start 
       even if the wifi isn't working. You can uncomment this if you want
       the device to keep trying wifi*/

    //If WiFi doesn't connect in 60 seconds, do a software reset
    /*reset_index ++;
    if (reset_index > 60) {
      Serial.println("WIFI Failed - restarting");
      delay(1000);
      ESP.restart();
    }*/
    // just give up. If you uncomment above, you must comment out this part
    reset_index ++;
    if (reset_index > 60){
      delay(500);
      Serial.println("Wifi connection did not complete. Proceeding.");
      return;
    }
    //-----------------------------------------
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(" WiFi Connected");
      Serial.println(WiFi.localIP());
      delay(500);
      return;
    }
  }
  return;
}
// send signalk data over UDP - thanks to PaddyB!
void sendSigK(String sigKey, float data)
{
 if (sendSig_Flag == 1)
 {
   DynamicJsonBuffer jsonBuffer;
   String deltaText;

   //  build delta message
   JsonObject &delta = jsonBuffer.createObject();

   //updated array
   JsonArray &updatesArr = delta.createNestedArray("updates");
   JsonObject &thisUpdate = updatesArr.createNestedObject();   //Json Object nested inside delta [...
   JsonArray &values = thisUpdate.createNestedArray("values"); // Values array nested in delta[ values....
   JsonObject &thisValue = values.createNestedObject();
   thisValue["path"] = sigKey;
   thisValue["value"] = data;
   thisUpdate["Source"] = "PanelSensors";

   // Send UDP packet
   udp.beginPacket(sigkserverip, sigkserverport);
   delta.printTo(udp);
   udp.println();
   udp.endPacket();
   delta.printTo(Serial);
   Serial.println();
 }

 return;
}
