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
#include "heydings.h"
#include "GxEPD2_display_selection_added.h"
#include <INA.h>
#include <Adafruit_ADS1015.h>
#include <WiFi.h>
// this is version 5 of ArduinoJson. Some day I'll port to version 6 ....
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <time.h>
#include "lwip/apps/sntp.h"

/**************************************************************************************************
** Declare program constants, global variables and instantiate INA class                         **
**************************************************************************************************/
const uint32_t SERIAL_SPEED = 115200; ///< Use fast serial speed
const uint32_t SHUNT_MICRO_OHM = 375; ///< Shunt resistance in Micro-Ohm, this is a 75mV / 200A shunt
const uint16_t MAXIMUM_AMPS = 200;    ///< Max expected amps, values are 1 - clamped to max 1022
uint8_t devicesFound = 0;             ///< Number of INAs found
INA_Class INA;                        ///< INA class instantiation

// Battery monitoring with two INA3221 devices, using two channels each. They are detected here is device numbers,
// One of the devices needs to have the I2C default address changed by jumper.
// You'll have to scan them with this program (uncomment below) to figure out which is which. Device numbers start at 0
const uint8_t batt1VoltageDev = 4;
const uint8_t batt1CurrentDev = 5;
const uint8_t batt2VoltageDev = 1;
const uint8_t batt2CurrentDev = 2;

const char *batt1Name = "HOUSE";
const char *batt2Name = "ENGINE";

// You'll also need to name the tanks
const char *tank1Name = " FORE";
const char *tank2Name = " STBD";

int refreshCounter = 0; // this is a global varable set up to count until a full screen refresh is needed

// Display offset for right side, in pixels
int rightOffset = 148;

/*********************************************************
 * WIfi and SignalK
 *********************************************************/
// We'll be sending SignalK data to a server via UDP. You'll have to tell
// the SignalK server that you have a new UDP connection available
WiFiUDP udp;

// Your wifi credentials go here
const char *ssid = "openplotter";
const char *password = "margaritaville";

// The address and port of your SignalK server goes here
IPAddress sigkserverip(10, 10, 10, 1);
// This is the port number you need to tell your server
uint16_t sigkserverport = 55561;

byte sendSig_Flag = 1;

// SignalK keys for power from the two battery banks
const char *batt1VoltageKey = "electrical.batteries.house.voltage";
const char *batt1CurrentKey = "electrical.batteries.house.current";
const char *batt2VoltageKey = "electrical.batteries.engine.voltage";
const char *batt2CurrentKey = "electrical.batteries.engine.current";

// SignalK keys for level of the two tanks
const char *tank1LevelKey = "tanks.freshWater.forwardTank.currentLevel";
const char *tank2LevelKey = "tanks.freshWater.starboardTank.currentLevel";

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
// Set this for your NTP server(s)
//char ntpserver1[] = "10.10.10.1";
char ntpserver1[] = "pool.ntp.org"; //You can get time from the net if you have a connection
// This is the Bahia de Banderas time zone. Set it for yours.
const char localTimeZone[23] = "<GMT-6>+6";

// This is to avoid PlatformIO Intellisense issues with time.h
//_VOID _EXFUN(tzset,	(_VOID));
// int	_EXFUN(setenv,(const char *__string, const char *__value, int __overwrite));

/*********************************************************
 * ADC for tank level monitoring
 * ******************************************************/
Adafruit_ADS1115 ads(0x48);

/*********************************************************
 * Touch Control
 * If you touch the bottom right screw, the screen toggles
 * between battery display and tank display
 * ******************************************************/
const uint8_t touchCtrlRight = 15;
uint8_t screen_mode = BATTERY_DISPLAY;

/*********************************************************
 * Screen positions / size of strings
 * We are saving these as a global so we can erase the
 * minimum amount of screen prior to screen update when
 * we return to the display funtion
 * ******************************************************/
int16_t tank1LevelX, tank1LevelY;
uint16_t tank1Width, tank1Height;
int16_t tank2LevelX, tank2LevelY;
uint16_t tank2Width, tank2Height;
int16_t batt1VoltX, batt1VoltY;
uint16_t batt1VoltWidth, batt1VoltHeight;
int16_t batt1AmpX, batt1AmpY;
uint16_t batt1AmpWidth, batt1AmpHeight;
int16_t batt2VoltX, batt2VoltY;
uint16_t batt2VoltWidth, batt2VoltHeight;
int16_t batt2AmpX, batt2AmpY;
uint16_t batt2AmpWidth, batt2AmpHeight;
int16_t dateX, dateY;
uint16_t dateWidth, dateHeight;
int16_t timeX, timeY;
uint16_t timeWidth, timeHeight;
int16_t netIconX, netIconY;
uint16_t netIconWidth, netIconHeight;

/*********************************************************
 * Left and right screen sizes
 * ******************************************************/
uint16_t halfScreen_x, halfScreen_y;
uint16_t halfScreen_w, halfScreen_h;
uint16_t rightScreenOffset;
uint16_t borderWidth;

/*********************************************************
 * Function Definitions for PlatformIO
 * *******************************************************/
float *getBattDeviceData(int deviceNumber);
void drawScreenOutlineBatt();
void drawScreenOutlineTank();
void setup_wifi();
void testUDP();
void sendSigK(String sigKey, float data);
float *getTankData();
void display_batt(float shuntAmps, float realVolts, bool rightSide);
int tankLevelAdjust(float tankLavel, bool leftTank);
void display_tank(int tankLevel, bool rightSide);
void displayStatus(String firstLine, String secondLine);

void setup()
{
  String statusLine1;
  String statusLine2;

  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);
  // Initialize the epaper display
  display.init(115200);
  // Get and set sub_screen sizes
  display.setRotation(3);
  borderWidth = 2;
  halfScreen_x = 2;
  halfScreen_y = 37;
  halfScreen_w = (display.width() / 2) - (borderWidth * 2);
  halfScreen_h = (display.height()) - halfScreen_y - 3;
  rightScreenOffset = (display.width() / 2);
  
  // Start the A/D converter for tank level measurement
  ads.begin(); 

  // Setup Battery Monitor
  Serial.println("Looking for INA device");
  displayStatus("Looking for INA device", " ");
  // IMPORTANT: if no INA devices are found the program will just continue to loop looking
  // for them! If you are unsure, run this with a serial monitor so you are sure you have
  // INA sensors connected.
  devicesFound = INA.begin(
      MAXIMUM_AMPS, SHUNT_MICRO_OHM); // Set to the expected Amp maximum and shunt resistance
  while (INA.begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM) == 0)
  {
    Serial.println("No INA device found, retrying in 10 seconds...");
    delay(10000); // Wait 10 seconds before retrying
    displayStatus("Looking for INA device", "Not found - retrying");
  }               // while no devices detected
  Serial.print(" - Detected ");
  Serial.print(devicesFound);
  Serial.println(" INA devices on the I2C bus");

  statusLine1 = "INA devices detected";
  statusLine2 = devicesFound;
  statusLine2 = statusLine2 + " devices found";
  displayStatus(statusLine1, statusLine2);
  delay(1000);
  INA.setBusConversion(8500);             // Maximum conversion time 8.244ms
  INA.setShuntConversion(8500);           // Maximum conversion time 8.244ms
  INA.setAveraging(128);                  // Average each reading n-times
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
  INA.alertOnBusOverVoltage(true, 15000); // Trigger alert if over 15V on bus

  setup_wifi();

  // Set up the ESP to retreive time from the server
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  // This assumes your RPI server has NTP running. You can use the SignalK "Set System Time" plugin to set the time
  sntp_setservername(0, ntpserver1);
  // sntp_setservername(0, ntpserver2);
  sntp_init();
  
  // Start with the battery display
  drawScreenOutlineBatt(); 
}

void loop()
{
  static char busChar[8], busMAChar[10]; // Output buffers
  float shuntAmps;
  float realVolts;
  float *ina_Output;
  float *adc_Output;
  bool leftTank = true;
  bool rightTank = false;
  bool leftBatt = false;
  bool rightBatt = true;

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
  if (touchRead(touchCtrlRight) < 30)
  {
    Serial.println("RIGHT TOUCH");
    if (screen_mode == BATTERY_DISPLAY)
    {
      screen_mode = TANK_DISPLAY;
      drawScreenOutlineTank();
    }
    else
    {
      screen_mode = BATTERY_DISPLAY;
      drawScreenOutlineBatt();
    }
    int heapSize = esp_get_free_heap_size();
    Serial.print("Heap is: ");
    Serial.print(heapSize);
    Serial.println();
  }

  /*****************************
   * Battery Bank 1
   * **************************/
  // Volts
  ina_Output = getBattDeviceData(batt1VoltageDev);
  realVolts = ina_Output[0] / 1000.0;
  // this is a kluge because the voltage sensor is reading .5v low
  if (realVolts > 0)
  {
    realVolts = realVolts + 0.5;
  }
  sendSigK(batt1VoltageKey, realVolts); // send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  // Serial.println("Battery 1");
  // Serial.print("Voltage: ");
  // Serial.print(busChar);

  // Amps
  ina_Output = getBattDeviceData(batt1CurrentDev);
  shuntAmps = ina_Output[1] / SHUNT_MICRO_OHM;
  sendSigK(batt1CurrentKey, shuntAmps); // send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  // Serial.print(" Current: ");
  // Serial.print(busMAChar);
  // Serial.println();

  // Print it on the left side
  if (screen_mode == BATTERY_DISPLAY)
  {
    display_batt(shuntAmps, realVolts, leftBatt);
  }

  /******************************
   * Battery Bank 2
   * ***************************/
  // Volts
  ina_Output = getBattDeviceData(batt2VoltageDev);
  realVolts = ina_Output[0] / 1000.0;
  // this is a kluge because the voltage sensor is reading .5v low
  if (realVolts > 0)
  {
    realVolts = realVolts + 0.5;
  }
  sendSigK(batt2VoltageKey, realVolts); // send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  // Serial.println("Battery 2");
  // Serial.print("Voltage: ");
  // Serial.print(busChar);

  // Amps
  ina_Output = getBattDeviceData(batt2CurrentDev);
  shuntAmps = ina_Output[1] / SHUNT_MICRO_OHM;
  sendSigK(batt2CurrentKey, shuntAmps); // send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  // Serial.print(" Current: ");
  // Serial.print(busMAChar);
  // Serial.println();

  // Print it on the right side
  if (screen_mode == BATTERY_DISPLAY)
  {
    display_batt(shuntAmps, realVolts, rightBatt);
  }

  /*******************************************************
   * ADC Tank Level Sensor
   * ****************************************************/
  float tankLevel;
  adc_Output = getTankData();
  Serial.print("ADC1: ");
  // tankLevel = (adc_Output[0]/24672)*100;
  tankLevel = (adc_Output[0] / 12336) * 100;

  Serial.println(tankLevelAdjust(tankLevel, leftTank));
  sendSigK(tank1LevelKey, tankLevel); // send to SignalK

  if (screen_mode == TANK_DISPLAY)
  {
    display_tank(tankLevelAdjust(tankLevel, leftTank), leftTank);
  }

  Serial.print("ADC2: ");
  // tankLevel = (adc_Output[1]/24672)*100;
  tankLevel = (adc_Output[1] / 12336) * 100;
  Serial.println(tankLevelAdjust(tankLevel, rightTank));
  sendSigK(tank2LevelKey, tankLevel); // send to SignalK

  if (screen_mode == TANK_DISPLAY)
  {
    display_tank(tankLevelAdjust(tankLevel, rightTank), rightTank);
  }

  Serial.println();
  delay(500);

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

  refreshCounter++;
  // Do a full screen refresh to keep the display healthy. With a B/W screen there are about 4 cycles/second, so
  // setting this to 2400 will fully refresh the screen about every 10 minutes.
  if (refreshCounter > 2400)
  {
    if (screen_mode == BATTERY_DISPLAY)
    {
      drawScreenOutlineBatt();
    }
    else
    {
      drawScreenOutlineTank();
    }
    refreshCounter = 0;

    return;
  }
}

// Batteries: Go and get the data from a specific device number
float *getBattDeviceData(int deviceNumber)
{

  static float x[4];

  x[0] = INA.getBusMilliVolts(deviceNumber);
  x[1] = INA.getShuntMicroVolts(deviceNumber);
  x[2] = INA.getBusMicroAmps(deviceNumber);
  x[3] = INA.getBusMicroWatts(deviceNumber);

  return x;
}

// Here were grabbing the ADC data. We're only using two of these right now, but we'll grab all four
float *getTankData()
{

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
  do
  { // Draw two boxes on the screen with a black area at the top for titles
    display.fillScreen(GxEPD_BLACK);
    display.fillRect(halfScreen_x, halfScreen_y, halfScreen_w, halfScreen_h, GxEPD_WHITE);
    display.fillRect(halfScreen_x + rightScreenOffset, halfScreen_y, halfScreen_w, halfScreen_h, GxEPD_WHITE);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(12, 30);
    display.print(batt1Name);
    display.setCursor(154, 30);
    display.print(batt2Name);
  } while (display.nextPage());

  return;
}

// This builds the screen for the tank display
void drawScreenOutlineTank()
{
  // Draw two side-by-side boxes
  display.setRotation(3); // Set to horizontal orientation
  display.setFullWindow();
  display.firstPage();
  do
  { // Draw two boxes on the screen
    display.fillScreen(GxEPD_BLACK);
    display.fillRect(halfScreen_x, halfScreen_y, halfScreen_w, halfScreen_h, GxEPD_WHITE);
    display.fillRect(halfScreen_x + rightScreenOffset, halfScreen_y, halfScreen_w, halfScreen_h, GxEPD_WHITE);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(18, 30);
    display.print(tank1Name);
    display.setCursor(160, 30);
    display.print(tank2Name);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(12, 52);
    display.print("WATER TANK");
    display.setCursor(162, 52);
    display.print("WATER TANK");
  } while (display.nextPage());

  return;
}

void display_batt(float shuntAmps, float realVolts, bool rightSide)
{

  static char busChar[8], busMAChar[10]; // Output buffers
  uint16_t box_x = halfScreen_x;
  if (rightSide)
  {
    box_x = box_x + rightScreenOffset;
  }
  uint16_t box_y = halfScreen_y + 4;
  uint16_t box_w = halfScreen_w;
  uint16_t box_h = halfScreen_h - 10;
  uint16_t cursor_y = box_y + box_h - 54;
  uint16_t cursor_x = box_x + 20;

  display.setRotation(3);

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
    display.setCursor(cursor_x, cursor_y);
    display.print(busChar);
    display.setCursor(cursor_x + 80, cursor_y);
    display.print(" V");
    display.setCursor(cursor_x, cursor_y + 32);
    display.print(busMAChar);
    display.setCursor(cursor_x + 80, cursor_y + 32);
    display.print(" A");
    display.setCursor(cursor_x + 20, cursor_y + 53);
    display.setFont(&FreeSansBold9pt7b);

    // Print date on the left, time on the right
    if (rightSide)
    {
      strftime(strftime_buf, sizeof(strftime_buf), "%T", &timeinfo);
    }
    else
    {
      strftime(strftime_buf, sizeof(strftime_buf), "%D", &timeinfo);
    }
    display.print(strftime_buf);

    // This displays a little network icon on the bottom right if the network is connected
    if (rightSide)
    {
      display.setCursor(box_x + 120, cursor_y + 53);
      display.setFont(&heydings_icons9pt7b);
      if (WiFi.status() == WL_CONNECTED)
      {
        display.print("R");
      }
      else
      {
        display.print("X");
      }
    }
  } while (display.nextPage());

  return;
}

void display_tank(int tankLevel, bool rightSide)
{

  uint16_t box_x = halfScreen_x;
  if (rightSide)
  {
    box_x = box_x + rightScreenOffset;
  }
  uint16_t box_y = halfScreen_y + 20;
  uint16_t box_w = halfScreen_w;
  uint16_t box_h = halfScreen_h - 25;
  uint16_t cursor_y = box_y + box_h - 30;
  uint16_t cursor_x = box_x + 20;
  String tankString;

  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);
  display.firstPage();
  do
  {
    display.setPartialWindow(box_x, box_y, box_w, box_h);
    tankString = String(tankLevel);
    tankString = tankString + "%";
    if (rightSide)
    {
      display.fillRect(tank2LevelX, tank2LevelY, tank2Width, tank2Height, GxEPD_WHITE);
      display.getTextBounds(tankString, cursor_x, cursor_y, &tank2LevelX, &tank2LevelY, &tank2Width, &tank2Height);
      display.setCursor(box_x + (box_w / 2 - tank2Width / 2), cursor_y);
    }
    else
    {
      display.fillRect(tank1LevelX, tank1LevelY, tank1Width, tank1Height, GxEPD_WHITE);
      display.getTextBounds(tankString, cursor_x, cursor_y, &tank1LevelX, &tank1LevelY, &tank1Width, &tank1Height);
      display.setCursor(box_x + (box_w / 2 - tank1Width / 2), cursor_y);
    }
    display.print(tankString);
    display.setCursor(box_x + 20, cursor_y + 25);
    display.setFont(&FreeSansBold9pt7b);
    // Print date on the left, time on the right
    if (rightSide)
    {
      display.fillRect(timeX, timeY, timeWidth, timeHeight, GxEPD_WHITE);
      strftime(strftime_buf, sizeof(strftime_buf), "%T", &timeinfo);
      display.getTextBounds(strftime_buf, display.getCursorX(), display.getCursorY(), &timeX, &timeY, &timeWidth, &timeHeight);
      display.setCursor(box_x + (box_w / 2 - timeWidth / 2), cursor_y + 25);
    }
    else //left side of the screen
    {
      display.fillRect(dateX, dateY, dateWidth, dateHeight, GxEPD_WHITE);
      strftime(strftime_buf, sizeof(strftime_buf), "%D", &timeinfo);
      display.getTextBounds(strftime_buf, display.getCursorX(), display.getCursorY(), &dateX, &dateY, &dateWidth, &dateHeight);
      display.setCursor(box_x + (box_w / 2 - timeWidth / 2), cursor_y + 25);
    }
    display.print(strftime_buf);

    // This displays a little network icon on the bottom right if the network is connected
    if (rightSide)
    {
      display.setCursor(box_x + 120, cursor_y + 25);
      display.setFont(&heydings_icons9pt7b);
      if (WiFi.status() == WL_CONNECTED)
      {
        display.print("R");
      }
      else
      {
        display.print("X");
      }
    }
  } while (display.nextPage());

  return;
}

int tankLevelAdjust(float tankLevel, bool leftTank)
{
  // Use this function to adjust for oddly shaped tanks or non-linear sensors
  // Tank displayed on the left display. The "tankLevel" variable is what is coming
  // from the sensor. The returned value is what is actually in the tank.
  // Some resistive sensors seem to have a lot of resistors closely grouped togehter
  // so you'll have to play with these values to match your sensor output and your tank(s)
  if (leftTank)
  { // Left display
    if (tankLevel > 99)
    {
      return 100;
    }
    else if (tankLevel > 90)
    {
      return 90;
    }
    else if (tankLevel > 80)
    {
      return 80;
    }
    else if (tankLevel > 70)
    {
      return 70;
    }
    else if (tankLevel > 60)
    {
      return 60;
    }
    else if (tankLevel > 50)
    {
      return 50;
    }
    else if (tankLevel > 40)
    {
      return 40;
    }
    else if (tankLevel > 30)
    {
      return 30;
    }
    else if (tankLevel > 20)
    {
      return 20;
    }
    else if (tankLevel > 10)
    {
      return 10;
    }
    else
    {
      return 0;
    }
  }
  else
  { // Right display
    if (tankLevel > 99)
    {
      return 100;
    }
    else if (tankLevel > 90)
    {
      return 90;
    }
    else if (tankLevel > 80)
    {
      return 80;
    }
    else if (tankLevel > 70)
    {
      return 70;
    }
    else if (tankLevel > 60)
    {
      return 60;
    }
    else if (tankLevel > 50)
    {
      return 50;
    }
    else if (tankLevel > 40)
    {
      return 40;
    }
    else if (tankLevel > 30)
    {
      return 30;
    }
    else if (tankLevel > 20)
    {
      return 20;
    }
    else if (tankLevel > 10)
    {
      return 10;
    }
    else
    {
      return 0;
    }
  }
}

void setup_wifi()
{
  String statusLine1;
  String statusLine2;

  delay(10);
  Serial.println();
  Serial.print("Connecting to Wifi SSID: ");
  Serial.println(ssid);
  statusLine1 = "Connecting to WiFi";
  statusLine2 = ssid;
  displayStatus(statusLine1, statusLine2);

  WiFi.begin(ssid, NULL);
  
  
  //WiFi.begin(ssid, password);
  int reset_index = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500); // this sets how long the device will wait for a wifi connection
    Serial.print(".");
    reset_index++;
    if (reset_index > 60)
    {
      delay(500);
      Serial.println("Wifi connection did not complete. Proceeding.");
      statusLine1 = "No connection to";
      statusLine2 = ssid;
      statusLine2 = statusLine2 + " proceeding";
      displayStatus(statusLine1, statusLine2);
      return;
    }
    //-----------------------------------------
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println(" WiFi Connected");
      Serial.println(WiFi.localIP());
      statusLine1 = "Connection Successful";
      statusLine2 = "IP: ";
      statusLine2 = statusLine2 + WiFi.localIP().toString();
      displayStatus(statusLine1, statusLine2);
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

    // updated array
    JsonArray &updatesArr = delta.createNestedArray("updates");
    JsonObject &thisUpdate = updatesArr.createNestedObject();   // Json Object nested inside delta [...
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
    //  delta.printTo(Serial);
    //  Serial.println();
  }

  return;
}

void displayStatus(String firstLine, String secondLine)
{ 
    display.setRotation(3); // Set to horizontal orentation
    display.setFullWindow();
    display.firstPage();
    do
    { // Draw two boxes on the screen with a black area at the top for titles
      display.fillScreen(GxEPD_WHITE);
      display.setFont(&FreeSansBold12pt7b);
      display.setTextColor(GxEPD_BLACK);
      display.setCursor(12, 30);
      display.print(firstLine);
      display.setCursor(12, 60);
      display.print(secondLine);
    } while (display.nextPage());
  return;
}
