#include <MKRWAN.h>           // LoRaWAN Library
#include <CayenneLPP.h>       // CAYENNE LPP
#include "ArduinoLowPower.h"  // Low Power Library




#define FRAME_DELAY       300000
#define DATA_RATE         5
#define ADAPTIVE_DR       true
#define CONFIRMED         false
#define PORT              2
#define CAYENNE_LPP       true
#define LOW_POWER         false
#define DEBUG
//#undef DEBUG

void getWeightscale1();
void getWeightscale2();
void getWeightscale3();
void Max17048();
void SHT4x();
void sendLoRa();
void enterLowPower();




/**** LoRaWAN Setup ******/
LoRaModem modem;
String appEui = "0000000000000000"; 
//String appKey = "4CE1D5496FB4A0A8D2075491464E5ACF";   // Ruche 1
String appKey = "C2E4A90ED4FF2C04529EB7E0B800E475";   // Ruche 2
uint8_t dataToSend[20] = {0}; // Data to send
uint8_t dataReceived[20] = {0}; // Data received via Donwlink
uint32_t frameDelay = FRAME_DELAY;

CayenneLPP dataToSendCayenne(51);
CayenneLPP dataReceivedCayenne(51);


/*********HX711***************/

#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN1 = 4;
const int LOADCELL_SCK_PIN1 = 3;
const int LOADCELL_DOUT_PIN2 = 18;
const int LOADCELL_SCK_PIN2 = 17;
const int LOADCELL_DOUT_PIN3 = 20;
const int LOADCELL_SCK_PIN3 = 19;

HX711 scale1;
HX711 scale2;
HX711 scale3;

#define number1
void HT711_1(number1);

#define number2
void HT711_2(number2);

#define number3
void HT711_3(number3);

/***************************************DS18B20*********************************/

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 13 on the Arduino
#define ONE_WIRE_BUS 13

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/*The setup function. We only start the sensors here*/
int deviceCount = 0;
float tempC;

/*************************MAX17048****************************************/

#include <Wire.h> // Needed for I2C
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library

//SFE_MAX1704X lipo; // Defaults to the MAX17043

//SFE_MAX1704X lipo(MAX1704X_MAX17043); // Create a MAX17043
//SFE_MAX1704X lipo(MAX1704X_MAX17044); // Create a MAX17044
SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048
//SFE_MAX1704X lipo(MAX1704X_MAX17049); // Create a MAX17049

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0; // Variable to keep track of LiPo state-of-charge (SOC)
bool alert; // Variable to keep track of whether alert has been triggered

/********************************SHT4X******************************/

#include <Arduino.h>
#include <SensirionI2CSht4x.h>
SensirionI2CSht4x sht4x;

 
void setup() {
  int connected;
  
  /***** Serial Link Configuration ******/
  Serial.begin(115200);
  //while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  for(uint32_t i=0;i<10;i++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
  }
  Serial.println("\r\n\r\n\r\n");
  Serial.println("########################################");
  Serial.println("########   LoRaWAN MKRWAN Scale   ######");
  Serial.println("#########     OTAA activation   ########\r\n");

  /****** Configure Here the First Module *****/
  //Enter here module configuration, ie : temperature sensor

   /****** Configure Here the Second Module *****/
  //Enter here module configuration, ie : Weight scale
  
  /**** LoRaWAN Configuration *****/
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1);
    Serial.print("Your device EUI is: ");
    Serial.println(modem.deviceEUI());
  };
  
  do{
  Serial.println(" JOIN procedure in progress ...");  
  connected = modem.joinOTAA(appEui, appKey);
  if (!connected)   Serial.println(" JOIN OTAA failed!!! Retry...");
  else              Serial.println(" JOIN procedure : SUCCESS !\r\n");
  }while(!connected);
  modem.dataRate(DATA_RATE);
  modem.setADR(1);
  modem.setPort(PORT);
  Serial.print(" Frame will be sent every ");Serial.print((frameDelay<7000)?7000:frameDelay);Serial.println("ms\r\n");  

/******************* DS18B20 ****************************/
  sensors.begin();
sensors.requestTemperatures(); // Send the command to get temperatures
deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");

/*********HX711-1***************/

Serial.println("HX711-1 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale1.begin(LOADCELL_DOUT_PIN1, LOADCELL_SCK_PIN1);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale1.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale1.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale1.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale1.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  scale1.set_scale(30556);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  //scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale1.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale1.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale1.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale1.get_units(5), 2);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");

/*********HX711-2***************/

Serial.println("HX711-2 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale2.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale2.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale2.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale2.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  scale2.set_scale(29323);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  //scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale2.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale2.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale2.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale2.get_units(5), 2);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:"); 


/*********HX711-3***************/

Serial.println("HX711-3 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale3.begin(LOADCELL_DOUT_PIN3, LOADCELL_SCK_PIN3);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale3.read());      // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale3.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale3.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale3.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
            // by the SCALE parameter (not set yet)

  scale3.set_scale(29508);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  //scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale3.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale3.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale3.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale3.get_units(5), 2);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:"); 

/*************************MAX17048****************************************/

Wire.begin();

  lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();

  // We can set an interrupt to alert when the battery SoC gets too low.
  // We can alert at anywhere between 1% - 32%:
  lipo.setThreshold(20); // Set alert threshold to 20%.



/********************************SHT4X******************************/

uint16_t error;
    char errorMessage[256];

    sht4x.begin(Wire);

    uint32_t serialNumber;
    error = sht4x.serialNumber(serialNumber);
    if (error) {
        Serial.print("Error trying to execute serialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Serial Number: ");
        Serial.println(serialNumber);
    }

}

void loop() {
  // Call here the first Module ie : getTemperature();
//DS18B20_1();
  // Call here the seconde Module ie : getWeight();
getWeightscale1();
  // Etc ...
delay(100);
getWeightscale2();
delay(100);
getWeightscale3();
delay(100);
Max17048();
delay(100);
SHT4x();
delay(100);  
sendLoRa();
delay(100);
 // if(LOW_POWER == true)   
enterLowPower();
 // else     
 //wait();

}


void sendLoRa(){
  int status;
  int i = 0;

  /***** Send data *****/
  if(CONFIRMED)   Serial.print(" Uplink CONFIRMED on PORT ");
  else            Serial.print(" Uplink UNCONFIRMED on PORT ");
  Serial.println(PORT);
  Serial.println(" Sending CAYENNE LPP (Low Power Payload) ");
  modem.beginPacket();
  //modem.write(dataToSend,1);
  modem.write(dataToSendCayenne.getBuffer(),dataToSendCayenne.getSize());
  status = modem.endPacket(CONFIRMED);
  if (status < 0) {
    Serial.println(" Send Frame failed!!!");
  } else {
    Serial.println(" Frame sent. Waiting for Downlink...");
  } 
  delay(3000); // Wait for downlink

 
  /***** Receive data *****/
  if (!modem.available()) {
    Serial.println(" No data received");
  }
  else {
    while (modem.available()) {
      dataReceived[i++] = (char)modem.read();
    }
    Serial.print(" Received Downlink (Hexa) : ");
    for (unsigned int j = 0; j < i; j++) {
      Serial.print(dataReceived[j] >> 4, HEX);
      Serial.print(dataReceived[j] & 0xF, HEX);
      Serial.print(" ");
    }  
    Serial.println();
  // First byte is the time between 2 Uplink in seconds
  // Second byte ... to be defined
  // Third byte ... etc...
    frameDelay=dataReceived[0]*1000;    // First Byte
  } 
  dataToSendCayenne.reset();
}


void  getWeightscale1(void){
  // Get Temperature
  /* Write here your code to get the temperature */
 // units = (scale.get_units(10), 2);
  // Update data to send
  sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);
    scale1.power_up();
 float poids1 = ((scale1.get_units()*(1-(0.00285*(tempC-25))))-3.35);

 float poids_raw1 = (scale1.get_units()-3.35);
 
 
  dataToSendCayenne.addAnalogInput(1, poids_raw1);
  dataToSendCayenne.addAnalogInput(3, poids1);
  dataToSendCayenne.addTemperature(4, tempC);
  //(scale.get_units(10), 1)
  
  
  scale1.power_down();
}

void  getWeightscale2(void){
  // Get Temperature
  /* Write here your code to get the temperature */
 // units = (scale.get_units(10), 2);
  // Update data to send
 // sensors.requestTemperatures(); // Send the command to get temperatures
 // float tempC = sensors.getTempCByIndex(0);
 
    scale2.power_up();
  float poids_raw2 = (scale2.get_units()-4.84);
 
 
  dataToSendCayenne.addAnalogInput(2, poids_raw2);  
  
  scale2.power_down();
}

void  getWeightscale3(void){
  // Get Temperature
  /* Write here your code to get the temperature */
 // units = (scale.get_units(10), 2);
  // Update data to send
 // sensors.requestTemperatures(); // Send the command to get temperatures
 // float tempC = sensors.getTempCByIndex(0);
 
    scale3.power_up();
  float poids_raw3 = (scale3.get_units()-4.84);
 
 
  dataToSendCayenne.addAnalogInput(5, poids_raw3);  
  
  scale3.power_down();
}

/*void getWeight(void){
  // Get Weight
  /* Write here your code to get the weight */
/*  // Update data to send 
  dataToSendCayenne.addAnalogOutput(1, 75.45f);  
}
*/
void enterLowPower(void){
  Serial.print(" Processor goes in Low Power mode during : ");
  Serial.print(frameDelay);Serial.println("ms\r\n");
  Serial.print(" Using the Low Power features usually corrupts the Serial Link ");
  LowPower.deepSleep(frameDelay); 
  Serial.println(" Wake Up !!!");  
}

void wait(void){
  Serial.print(" Processor is going to wait during : ");
  Serial.print(frameDelay);Serial.println("ms\r\n");
  delay(frameDelay); 
}

void DS18B20_1(void){   

sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);
if(tempC != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Temperature for the device 1 (index 0) is: ");
    Serial.println(tempC);
    dataToSendCayenne.addTemperature(4, tempC);
  } 
  else
  {
    Serial.println("Error: Could not read temperature data");
  }
   }

 void Max17048(void){

 // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
  soc = lipo.getSOC(); 

  dataToSendCayenne.addAnalogInput(6, soc);
    
 }

void SHT4x(void){
  
uint16_t error;
    char errorMessage[256];

    delay(1000);

    float temperatureshtx;
    float humidityshtx;
    error = sht4x.measureHighPrecision(temperatureshtx, humidityshtx);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature:");
        Serial.print(temperatureshtx);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidityshtx);

dataToSendCayenne.addTemperature(7, temperatureshtx);
dataToSendCayenne.addRelativeHumidity(8, humidityshtx);        
    }  
}