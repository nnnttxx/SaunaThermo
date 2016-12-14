// ATmega328 with int. OSC 8 MHz @ 3.3V

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "RF24.h"
#include "Sleeper.h"


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7, 8);   // ce_pin, cs_pin
/**********************************************************/

Sleeper g_sleeper;
/* usage
  // Power down for 10 seconds.
  g_sleeper.SleepMillis(10000);
*/

//#define SleepTimeColdMs   1800000   // 30 min
#define SleepTimeColdMs   1000
//#define SleepTimeHotMs    30000     // 0,5 min
#define SleepTimeHotMs   1000

#define RawTempHot        15        // 45 deg
#define RawTempCold       10       // 40 deg

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 9
#define TEMPERATURE_PRECISION 9

// arrays to hold device addresses
DeviceAddress SaunaThermometer;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

enum eMainState
{
  Cold,
  Hot
} eSaunaState;

byte addresses[][6] = {"Master","Slave"};

struct DataStruct 
{
  unsigned int ui16RawTemperature;
  long lSleepTimeMs;
} TxData;

void setup()
{
  // start serial port
  Serial.begin(9600);  

  // Start up the library
  sensors.begin();

  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }


  // Search for devices on the bus and assign based on an index
  if (!sensors.getAddress(SaunaThermometer, 0))
  {
    Serial.println("Unable to find address for Device 0");
  }

  eSaunaState = Cold;

  Serial.println("Init done\n");
}

void loop()
{
  switch (eSaunaState)
  {
    default:
    case Cold:
      sensors.requestTemperatures(); // Send the command to get temperatures
      TxData.ui16RawTemperature = sensors.getTemp(SaunaThermometer);
      TxData.lSleepTimeMs = SleepTimeColdMs;

      if (DallasTemperature::rawToCelsius(TxData.ui16RawTemperature) > RawTempHot)
      {
        eSaunaState = Hot;
        TxData.lSleepTimeMs = SleepTimeHotMs;
      }
      break;

    case Hot:
      sensors.requestTemperatures(); // Send the command to get temperatures
      TxData.ui16RawTemperature = sensors.getTemp(SaunaThermometer);
      TxData.lSleepTimeMs = SleepTimeHotMs;

      if (DallasTemperature::rawToCelsius(TxData.ui16RawTemperature)< RawTempCold)
      {
        eSaunaState = Cold;
        TxData.lSleepTimeMs = SleepTimeColdMs;
        break;
      }

      //radio.powerUp();
      // send ui16RawTemp, SleeptimeHot;
      if (!radio.write(&TxData, sizeof(TxData)))
      {
        Serial.println(F("TX failed"));
      }

      //radio.powerDown();
      TxData.lSleepTimeMs = SleepTimeHotMs;
      break;
  }

  Serial.print("Main State: ");
  Serial.println(eSaunaState);
  Serial.print("Temperature: ");
  Serial.println(DallasTemperature::rawToCelsius(TxData.ui16RawTemperature));

  g_sleeper.SleepMillis(TxData.lSleepTimeMs);
}
