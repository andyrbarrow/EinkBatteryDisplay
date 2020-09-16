// Battery display system using E-paper display and INA3221 three-channel voltage/current sensors
//
// based on e-ink display library by Jean-Marc Zingg and SignalK UDP sender 
// by PaddyB
//
//
// by Andy Barrow (andy@sailor.nu)
//
// mapping suggestion for ESP32, e.g. LOLIN32, see .../variants/.../pins_arduino.h for your board
// NOTE: there are variants with different pins for SPI ! CHECK SPI PINS OF YOUR BOARD
// BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

#define ENABLE_GxEPD2_GFX 1

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "GxEPD2_boards_added.h"
#include "bitmaps/Bitmaps3c128x296.h" // 2.9"  b/w/r
#include <INA.h>
#include <WiFi.h>
//this is version 5 of ArduinoJson. Some day I'll port to version 6 ....
#include <ArduinoJson.h>
#include <WiFiUdp.h>

/**************************************************************************************************
** Declare program constants, global variables and instantiate INA class                         **
**************************************************************************************************/
const uint32_t SERIAL_SPEED    = 115200;  ///< Use fast serial speed
const uint32_t SHUNT_MICRO_OHM = 375;     ///< Shunt resistance in Micro-Ohm, this is a 75mV / 200A shunt
const uint16_t MAXIMUM_AMPS    = 200;     ///< Max expected amps, values are 1 - clamped to max 1022
uint8_t        devicesFound    = 0;       ///< Number of INAs found
INA_Class      INA;                       ///< INA class instantiation

// This is done with two INA3221 devices, using two channels each. They are detected here is device numbers,
// One of the devices needs to have the I2C default address changed by jumper.
// You'll have to scan them with this program (uncomment below) to figure out which is which. Device numbers start at 0
const uint8_t  batt1VoltageDev = 4;
const uint8_t  batt1CurrentDev = 5;
const uint8_t  batt2VoltageDev = 1;
const uint8_t  batt2CurrentDev = 2;

const char * batt1Name = "HOUSE";
const char * batt2Name = "ENGINE";
int refreshCounter = 0;

/*********************************************************
 * WIfi and SignalK
*********************************************************/
// We'll be sending SignalK data to a server via UDP. You'll have to tell
// the server that you have a new UDP connection available
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
const char* batt1VoltageKey = "electrical.batteries.bank1.voltage";
const char* batt1CurrentKey = "electrical.batteries.bank1.current";
const char* batt2VoltageKey = "electrical.batteries.bank2.voltage";
const char* batt2CurrentKey = "electrical.batteries.bank2.current";

/*********************************************************
**Function Definitions for PlatformIO
*********************************************************/
float * getDeviceData (int deviceNumber);
void drawScreenOutline();
void setup_wifi();
void testUDP();
void sendSigK(String sigKey, float data);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  delay(100);
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
  // initialize the epaper display
  display.init(115200);
  drawScreenOutline();
  setup_wifi();
}

void loop()
{
  static char     sprintfBuffer[100];  // Buffer to format output
  static char     busChar[8], busMAChar[10];  // Output buffers
  float shuntAmps;
  float realVolts;
  float *x;

  uint16_t box_x = 20;
  uint16_t box_y = 45;
  uint16_t box_w = 115;
  uint16_t box_h = 70;
  uint16_t cursor_y = box_y + box_h - 45;

  Serial.println("Battery 1");
  Serial.print("Voltage: ");
  x = getDeviceData(batt1VoltageDev);
  realVolts = x[0] / 1000.0;
  sendSigK(batt1VoltageKey, realVolts); //send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  Serial.print(busChar);
  Serial.print(" Current: ");
  x = getDeviceData(batt1CurrentDev);
  shuntAmps = x[1] / SHUNT_MICRO_OHM;
  sendSigK(batt1CurrentKey, shuntAmps); //send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  Serial.print(busMAChar);
  Serial.println();
  sprintf(sprintfBuffer, "Batt 1: %sV %sA\n", busChar, busMAChar);

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
      display.setCursor(box_x, cursor_y+40);
      display.print(busMAChar);
      display.setCursor(box_x + 80, cursor_y+40);
      display.print(" A");
    }
  while (display.nextPage());
  //delay(1000);

  Serial.println("Battery 2");
  Serial.print("Voltage: ");
  x = getDeviceData(batt2VoltageDev);
  realVolts = x[0] / 1000.0;
  sendSigK(batt2VoltageKey, realVolts); //send to SignalK
  dtostrf(realVolts, 2, 1, busChar);
  Serial.print(busChar);
  Serial.print(" Current: ");
  x = getDeviceData(batt2CurrentDev);
  shuntAmps = x[1] / SHUNT_MICRO_OHM;
  sendSigK(batt2CurrentKey, shuntAmps); //send to SignalK
  dtostrf(shuntAmps, 2, 1, busMAChar);
  Serial.print(busMAChar);
  Serial.println();
  sprintf(sprintfBuffer, "Batt 2: %sV %sA\n", busChar, busMAChar);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  box_x = box_x + 148;
  
  do
    {
      display.setPartialWindow(box_x, box_y, box_w, box_h);
      display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
      display.setCursor(box_x, cursor_y);
      display.print(busChar);
      display.setCursor(box_x + 80, cursor_y);
      display.print(" V");
      display.setCursor(box_x, cursor_y+40);
      display.print(busMAChar);
      display.setCursor(box_x + 80, cursor_y+40);
      display.print(" A");
    }
  while (display.nextPage());
  //delay(1000);

  /* Uncomment this to detect and display device numbers
  static uint16_t loopCounter = 0;     // Count the number of iterations
  static char     shuntChar[10], busMWChar[10];  // Output buffers

  //Serial.print("Nr Adr Type   Bus      Shunt Curr  Bus         Bus\n");
  //Serial.print("== === ====== ======== =========== =========== ===========\n");
  for (uint8_t i = 0; i < devicesFound; i++)  // Loop through all devices
  {
    x = getDeviceData(i);
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
  //Do a full screen refresh to keep the display healthy. Each screen take 20 seconds, so 15 here is 5 min.
  if (refreshCounter > 100){
    // This will refresh the entire screen
    drawScreenOutline();
    // Just in case the wifi isn't connected, try again.
    setup_wifi();
    refreshCounter = 0;
  }
}

//Go and get the data from a specific device
float * getDeviceData (int deviceNumber){

  static float x[4];
    
    x[0] = INA.getBusMilliVolts(deviceNumber);
    x[1] = INA.getShuntMicroVolts(deviceNumber);
    x[2] = INA.getBusMicroAmps(deviceNumber);
    x[3] = INA.getBusMicroWatts(deviceNumber);

    return x;
}

void drawScreenOutline()
{
  // Draw two side-by-side boxes
  //display.writeFillRect(0,0,display.width(),display.height(),GxEPD_BLACK);
  display.firstPage();
  do { //Draw two boxes on the screen
    display.fillScreen(GxEPD_WHITE);
    display.fillScreen(GxEPD_BLACK);
    display.fillRect(2,2,display.width()-4, (display.height()/2)-4,GxEPD_WHITE);
    display.fillRect(2, (display.height() / 2)+2, display.width()-4, (display.height()/2)-4,GxEPD_WHITE);
    display.setRotation(3);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(8, 30);
    display.print(batt1Name);
    display.setCursor(154, 30);
    display.print(batt2Name);
  }
  while (display.nextPage());
  // if the device didn't find wifi before, it will just move on after 30 seconds. Here we check again for
  // wifi. This will delay reading battery voltage 30 seconds, so if you don't want that delay and don't care about
  // reconnecting WiFi, comment this out. If wifi is already connected, there will be no delay.
  setup_wifi();
}
 
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting ");
  WiFi.begin(ssid, password);
  int reset_index = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    /* Decided not to do this, so the device can go ahead an start 
       even if the wifi isn't working. You can uncomment this if you want
       the device to keep trying wifi
*/
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
   thisUpdate["Source"] = "BatterySensors";

   // Send UDP packet
   udp.beginPacket(sigkserverip, sigkserverport);
   delta.printTo(udp);
   udp.println();
   udp.endPacket();
   delta.printTo(Serial);
   Serial.println();
 }
}