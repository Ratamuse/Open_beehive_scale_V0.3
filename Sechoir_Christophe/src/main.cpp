/************************************************************/
/*                     Séchoir Christophe                   */
/*                                                          */
/* par RATAMUSE                                             */
/*                                                          */
/* 26/07/2022                                               */
/************************************************************/
/* ESP-WROOM-32E                                            */
/*                                                          */
/* Type de carte : Custom Test board V1.0 by Ratamuse       */
/* Flash size    : 16Mb/128Mb                               */
/* Debug port    : Serial                                   */
/* CPU Frequency : 80Mhz                                    */
/* Upload Speed  : 921600                                   */
/*                                                          */
/************************************************************/
/*                                                          */
/* Capteurs :                                               */
/* SENSOR SHT40  temperature+humidite                       */
/* BME680        pression                                   */
/*                                                          */
/*                                                          */
/************************************************************/
/*                                                          */
/* Publication sur serveur Domoticz et Grafana:             */
/*                                                          */
/* https://www.domoticz.com/                                */
/* https://grafana.com/                                     */
/*                                                          */
/************************************************************/


#include <Arduino.h>
#include <Wire.h>
#include <PWFusion_TCA9548A.h>
#include <SensirionI2CSht4x.h>
TCA9548A i2cMux;
SensirionI2CSht4x sht4x;

void SHT4x0();
void SHT4x1();
void SHT4x2();
void SHT4x3();
void SHT4x4();
void pression();

int hum_stat0;
int hum_stat1;
int hum_stat2;
int hum_stat3;
int hum_stat4;

/************* WIFI ******************/
#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/rtc_io.h"
HTTPClient http;
const char *ssid = "Livebox-1900";
const char *password = "DxFpFVXEUZbdeb6amj";
const char *host = "192.168.1.17";
const int port = 8080;
const int   watchdog = 20000; // Fréquence d'envoi des données à Domoticz - Frequency of sending data to Domoticz
unsigned long previousMillis = millis();
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
//long TimeToSleep;



void connectToNetwork()
{

  WiFi.begin(ssid, password);
  //WiFi.config(ip, dns, gateway, subnet);
  uint32_t notConnectedCounter = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.println("Wifi connecting...");
    notConnectedCounter++;
    if (notConnectedCounter > 50)
    { // Reset board if not connected after 5s
      Serial.println("Resetting due to Wifi not connecting...");

      ESP.restart();
    }
  }
  Serial.println("Connected to network");
}

/************** Web Server ****************/

#include <WebServer.h>
WebServer server(80);

/****************************BME 680*********************************/   

#include "bsec.h"

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
#define LED_BUILTIN 16

// Create an object of the class Bsec
Bsec iaqSensor;

String output;


void setup() {
  // Initialize I2C and Serial
  Wire.begin();
  Serial.begin(115200);

  // Initialize I2C multiplexor
  i2cMux.begin();

    uint16_t error;
    char errorMessage[256];

    sht4x.begin(Wire);
    
/****************************BME 680*********************************/   

iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);

  
}

void loop() {

  unsigned long currentMillis = millis();
  if ( currentMillis - previousMillis > watchdog ) {
    previousMillis = currentMillis;
Serial.println("Wifi connecting...");
connectToNetwork();


SHT4x0();
  
delay(1000);
  
SHT4x1();
  
delay(1000); 

SHT4x2();
  
delay(1000);

SHT4x3();
  
delay(1000);

SHT4x4();

delay(1000);

pression();

delay(1000);
  
http.end();
     WiFi.disconnect();
  Serial.println("Wifi déconnecté");
  WiFi.mode(WIFI_OFF);

  } 
 
}


void SHT4x0()
{

 float temperature0;
 float humidity0;

  i2cMux.setChannel(CHAN0); 
  Serial.println("Channel 0 selected"); 
uint16_t error;
    char errorMessage[256];

    delay(1000);
sht4x.measureHighPrecision(temperature0, humidity0);

  
   
    error = sht4x.measureHighPrecision(temperature0, humidity0);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {

      
        
        if ( humidity0 > 70 ) {
          hum_stat0 = 3;
        } else if ( humidity0 < 30 ) {
          hum_stat0 = 2; 
        } else if ( ( humidity0 >= 30 ) & (humidity0 <= 45 )) {
          hum_stat0 = 0;
        } else if (( humidity0 > 45 ) & (humidity0 <= 70 )) {
          hum_stat0 = 1;
        

  }
        Serial.print("Temperature:");
        Serial.print(temperature0);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity0);
        Serial.print("\t");
        Serial.print("Tendance:");
        Serial.println(hum_stat0);
    }

  String url = "/json.htm?type=command&param=udevice&idx=1&nvalue=0&svalue=";
  url += String(temperature0, 1);
  url += ";";
  url += String(humidity0, 1);
  url += ";";
  url += String(hum_stat0);
  url += ";";
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);

  String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url);
  http.begin(host, port, url);
  int httpCode = http.GET();
  if (httpCode)
  {
    if (httpCode == 200)
    {
      String payload = http.getString();
      Serial.println("Domoticz response ");
      Serial.println(payload);
    }
    else
    {
      String payload = http.getString();
      Serial.println("Domoticz not response ");
      Serial.println(String(httpCode));
      Serial.println(complete_url);
    }
  }
}

void SHT4x1()
{

 float temperature1;
 float humidity1;
 
  i2cMux.setChannel(CHAN1); 
  Serial.println("Channel 1 selected"); 
uint16_t error;
    char errorMessage[256];

    delay(1000);
sht4x.measureHighPrecision(temperature1, humidity1);

  
   
    error = sht4x.measureHighPrecision(temperature1, humidity1);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {

      
        
        if ( humidity1 > 70 ) {
          hum_stat1 = 3;
        } else if ( humidity1 < 30 ) {
          hum_stat1 = 2; 
        } else if (( humidity1 >= 30) & (humidity1 <= 45 )) {
          hum_stat1 = 0;
        } else if (( humidity1 > 45) & (humidity1 <= 70 )) {
          hum_stat1 = 1;
        

  }
        Serial.print("Temperature:");
        Serial.print(temperature1);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity1);
        Serial.print("\t");
        Serial.print("Tendance:");
        Serial.println(hum_stat1);
    }

  String url = "/json.htm?type=command&param=udevice&idx=2&nvalue=0&svalue=";
  url += String(temperature1, 1);
  url += ";";
  url += String(humidity1, 1);
  url += ";";
  url += String(hum_stat1);
  url += ";";
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);

  String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url);
  http.begin(host, port, url);
  int httpCode = http.GET();
  if (httpCode)
  {
    if (httpCode == 200)
    {
      String payload = http.getString();
      Serial.println("Domoticz response ");
      Serial.println(payload);
    }
    else
    {
      String payload = http.getString();
      Serial.println("Domoticz not response ");
      Serial.println(String(httpCode));
      Serial.println(complete_url);
    }
  }
}

void SHT4x2()
{

 float temperature2;
 float humidity2;

  i2cMux.setChannel(CHAN2); 
  Serial.println("Channel 2 selected"); 
uint16_t error;
    char errorMessage[256];

    delay(1000);
sht4x.measureHighPrecision(temperature2, humidity2);

  
   
    error = sht4x.measureHighPrecision(temperature2, humidity2);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {

      
        
        if ( humidity2 > 70 ) {
          hum_stat2 = 3;
        } else if ( humidity2 < 30 ) {
          hum_stat2 = 2; 
        } else if (( humidity2 >= 30) & (humidity2 <= 45 )) {
          hum_stat2 = 0;
        } else if (( humidity2 > 45) & (humidity2 <= 70 )) {
          hum_stat2 = 1;
        

  }
        Serial.print("Temperature:");
        Serial.print(temperature2);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity2);
        Serial.print("\t");
        Serial.print("Tendance:");
        Serial.println(hum_stat2);
    }

  String url = "/json.htm?type=command&param=udevice&idx=3&nvalue=0&svalue=";
  url += String(temperature2, 1);
  url += ";";
  url += String(humidity2, 1);
  url += ";";
  url += String(hum_stat2);
  url += ";";
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);

  String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url);
  http.begin(host, port, url);
  int httpCode = http.GET();
  if (httpCode)
  {
    if (httpCode == 200)
    {
      String payload = http.getString();
      Serial.println("Domoticz response ");
      Serial.println(payload);
    }
    else
    {
      String payload = http.getString();
      Serial.println("Domoticz not response ");
      Serial.println(String(httpCode));
      Serial.println(complete_url);
    }
  }
}

void SHT4x3()
{

 float temperature3;
 float humidity3;

  i2cMux.setChannel(CHAN3); 
  Serial.println("Channel 3 selected"); 
uint16_t error;
    char errorMessage[256];

    delay(1000);
sht4x.measureHighPrecision(temperature3, humidity3);

  
   
    error = sht4x.measureHighPrecision(temperature3, humidity3);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {

      
        
        if ( humidity3 > 70 ) {
          hum_stat3 = 3;
        } else if ( humidity3 < 30 ) {
          hum_stat3 = 2; 
        } else if ( (humidity3 >= 30 )& (humidity3 <= 45 )) {
          hum_stat3 = 0;
        } else if ( (humidity3 > 45 )& (humidity3 <= 70 )) {
          hum_stat3 = 1;
        

  }
        Serial.print("Temperature:");
        Serial.print(temperature3);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity3);
        Serial.print("\t");
        Serial.print("Tendance:");
        Serial.println(hum_stat3);
    }

  String url = "/json.htm?type=command&param=udevice&idx=4&nvalue=0&svalue=";
  url += String(temperature3, 1);
  url += ";";
  url += String(humidity3, 1);
  url += ";";
  url += String(hum_stat3);
  url += ";";
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);

  String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url);
  http.begin(host, port, url);
  int httpCode = http.GET();
  if (httpCode)
  {
    if (httpCode == 200)
    {
      String payload = http.getString();
      Serial.println("Domoticz response ");
      Serial.println(payload);
    }
    else
    {
      String payload = http.getString();
      Serial.println("Domoticz not response ");
      Serial.println(String(httpCode));      
      Serial.println(complete_url);
    }
  }
}

void SHT4x4()
{

 float temperature4;
 float humidity4;

  i2cMux.setChannel(CHAN4); 
  Serial.println("Channel 4 selected"); 
uint16_t error;
    char errorMessage[256];

    delay(1000);
sht4x.measureHighPrecision(temperature4, humidity4);

  
   
    error = sht4x.measureHighPrecision(temperature4, humidity4);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {

      
        
        if ( humidity4 > 70 ) {
          hum_stat4 = 3;
        } else if ( humidity4 < 30 ) {
          hum_stat4 = 2; 
        } else if (( humidity4 >= 30 )& (humidity4 <= 45 )) {
          hum_stat4 = 0;
        } else if (( humidity4 > 45) & ( humidity4 <= 70 )) {
          hum_stat4 = 1;
        

  }
        Serial.print("Temperature:");
        Serial.print(temperature4);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity4);
        Serial.print("\t");
        Serial.print("Tendance:");
        Serial.println(hum_stat4);
    }

  String url = "/json.htm?type=command&param=udevice&idx=5&nvalue=0&svalue=";
  url += String(temperature4, 1);
  url += ";";
  url += String(humidity4, 1);
  url += ";";
  url += String(hum_stat4);
  url += ";";
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);

  String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url);
  http.begin(host, port, url);
  int httpCode = http.GET();
  if (httpCode)
  {
    if (httpCode == 200)
    {
      String payload = http.getString();
      Serial.println("Domoticz response ");
      Serial.println(payload);
    }
    else
    {
      String payload = http.getString();
      Serial.println("Domoticz not response ");
      Serial.println(String(httpCode));     
      Serial.println(complete_url);
    }
  }
}

void pression(){ 



  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
  } else {
    checkIaqSensorStatus();

  }
delay(10); 

Serial.print(iaqSensor.pressure,1);

   String url = "/json.htm?type=command&param=udevice&idx=6&nvalue=0&svalue=";
        url += String(iaqSensor.pressure/100,2); url += ";";
       
        
  Serial.print("connecting to ");
  Serial.println(host);
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
 String complete_url = "http://";
  complete_url += String(host);
  complete_url += ":";
  complete_url += String(port);
  complete_url += String(url); 
  http.begin(host,port,url);
  int httpCode = http.GET();
    if (httpCode) {
      if (httpCode == 200) {
        String payload = http.getString();
        Serial.println("Domoticz response "); 
        Serial.println(payload);
      }else{
        String payload = http.getString();
        Serial.println("Domoticz not response ");
        Serial.println(String(httpCode));
        // pour interprétation des codes d'erreur : https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.h
        Serial.println(complete_url);
      }
    }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}