/**The MIT License (MIT)
Copyright (c) 2015 by Daniel Eichhorn
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
See more at http://blog.squix.ch
AP: 192.168.4.1 
https://home.openweathermap.org/api_keys
*/

#include <simpleDSTadjust.h>

// Setup
#define WM_SSID "AutoConnectAP"
#define WM_PASS "esp8266"
#define WM_HOSTNAME "ThingPulse-weather-station-color"

//#define WIFI_SSID "SSID"
//#define WIFI_PASS "password"
//#define WIFI_HOSTNAME "ThingPulse-weather-station-color"

int UPDATE_INTERVAL_SECS = 10 * 60; // Update every 10 minutes
int SLEEP_INTERVAL_SECS = 0;        // Going to sleep after idle times, set 0 for insomnia

// OpenWeatherMap Settings
// Sign up here to get an API key: https://docs.thingpulse.com/how-tos/openweathermap-key/
//String OPEN_WEATHER_MAP_APP_ID = "OPEN_WEATHER_MAP_APP_ID";
String OPEN_WEATHER_MAP_APP_ID ="8513485f819a832c38d15f247ebb6cb6";
/*
Go to https://openweathermap.org/find?q= and search for a location. Go through the
result set and select the entry closest to the actual location you want to display 
data for. It'll be a URL like https://openweathermap.org/city/2657896. The number
at the end is what you assign to the constant below.
 */
String OPEN_WEATHER_MAP_LOCATION_ID = "2878676";
String DISPLAYED_CITY_NAME = "Leopoldshafen,DE";

String  OPEN_WEATHER_MAP_LANGUAGE = "de";   // was "en"
uint8_t MAX_FORECASTS = 10;

//-- adjust according to your language
const String WDAY_NAMES[] = {"Son", "Mon", "Die", "Mit", "Don", "Fri", "Sam"};
const String MONTH_NAMES[] = {"Jan", "Feb", "Mär", "Apr", "Mai", "Jun", "Jul", "Aug", "Sep", "Okt", "Nov", "Dez"};
const String SUN_MOON_TEXT[] = {"Sonne", "Aufg", "Unter", "Mond", "Alter", "Hell"};
const String MOON_PHASES[] = {"Neumond", "zunehmend", "Ein Viertel", "zunehmend 3/4","Vollmond", "abnehmend 3/4", "Drei Viertel", "abnehmend"};

#define UTC_OFFSET +1
struct dstRule StartRule = {"CEST", Last, Sun, Mar, 2, 3600}; // Central European Summer Time = UTC/GMT +2 hours
struct dstRule EndRule = {"CET", Last, Sun, Oct, 2, 0};       // Central European Time = UTC/GMT +1 hour

// values in metric or imperial system?
boolean IS_METRIC = true;

//--- clock style: 12/24 hour 
bool IS_STYLE_12HR = false;

//--- NTP (time servers)
String NTP_SERVERS_1 = "0.ch.pool.ntp.org";
String NTP_SERVERS_2 = "1.ch.pool.ntp.org";
String NTP_SERVERS_3 = "2.ch.pool.ntp.org";

int FRAME_COUNT = 3;

//--- August 1st, 2018
#define NTP_MIN_VALID_EPOCH 1533081600

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//--- centralized pin settings
//--- settings for LOLIN TFT2.4 with Wemos D1 mini-pinout
//---
#define TFT_DC      D8     //D2
#define TFT_CS      D0     //D1
#define TFT_LED     D4     //D8

/* #define HAVE_TOUCHPAD */
#define TOUCH_CS    D3
#define TOUCH_IRQ   255

#define BME_CS      A0      //define D0 pin for BME280 sensor CSB pin connection
#define LDR_PIN     A0      //analog input for LDR illuminance sensor 

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/***************************
 * End Settings
 **************************/
