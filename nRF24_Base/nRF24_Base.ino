// ATmega328 with int. OSC 8 MHz @ 3.3V

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <cc1100.h>
#include "Sleeper.h"

// ADC Input defines
#define ADC_InCH0   0
#define ADC_InCH7   7
#define ADC_InTemp  8
#define ADC_InVBG  14
#define ADC_InGND  15
#define DS18B20_GND   7
#define DS18B20_DQ    8
#define DS18B20_VCC   9

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS DS18B20_DQ

#define SleepTimeColdMs   1800000   // 30 min
#define SleepTimeHotMs    30000     // 0,5 min
#define TempKHot        50 + 273       // Hysteresis rising 
#define TempKCold       45 + 273       // Hysteresis falling

//#define SleepTimeColdMs   10000   // 30 min
//#define SleepTimeHotMs    5000     // 0,5 min
//#define TempKHot        20 + 273       // Hysteresis rising 
//#define TempKCold       TempKHot       // Hysteresis falling

enum eMainState
{
  Cold,
  Hot
} eSaunaState;

typedef enum
{
  etSleep,
  etWarmingUp,
  etMeasure,
  etCoolingDown,
} eProtocol;

struct DataStruct
{
  unsigned char len;      // dummy
  unsigned char RxAddr;   // dummy
  unsigned char TxAddr;   // dummy
  unsigned int uiTemperatureK;
  eProtocol etTxProtocol;
  unsigned long ulSleepTimeMs;          // sleep time ms for 16MHz
  unsigned int uiTxCounter;
  unsigned int uiBatteryVoltage;
};

// Global Variables
uint8_t My_addr, Rx_addr, ucTxPackketlength, ucLqi;
DataStruct TxData;

//init CC1100 constructor
CC1100 cc1100;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature DS18B20(&oneWire);

Sleeper g_sleeper;
/* usage
  // Power down for 10 seconds.
  g_sleeper.SleepMillis(10000);
*/

void setup()
{
  // start serial port
  Serial.begin(9600);
  Serial.println(F("**** Master ***"));

  // disable ADC
  ADCSRA &= ~_BV(ADEN);   // sleep with 20nA current consumption on internal RC with 8 MHz

  pinMode(DS18B20_GND, OUTPUT);
  digitalWrite(DS18B20_GND, LOW);
  pinMode(DS18B20_VCC, OUTPUT);
  digitalWrite(DS18B20_VCC, HIGH);      // enable sensor for initialisation
  
  delay(2);       // wait for sensor to start

  // Start up the library
  DS18B20.begin();
  Serial.print(F("Numnber of OneWire devices: ")); Serial.println(DS18B20.getDeviceCount());
  digitalWrite(DS18B20_VCC, LOW);      // disable sensor

  // init CC1101 RF-module
  if (cc1100.begin(CC1100_MODE_GFSK_1_2_kb, CC1100_FREQ_868MHZ, 1, 10, 1)) // modulation mode, frequency, channel, PA level in dBm, own address
  {
#ifndef CC1100_DEBUG
    Serial.println(F("Init CC1101 successful"));
#endif
  }

  cc1100.show_main_settings();             //shows setting debug messages to UART
  cc1100.show_register_settings();         //shows current CC1101 register values
  My_addr = cc1100.get_myaddr();
  cc1100.powerdown();

  Rx_addr = 0x00;                          // BROADCAST

  ucTxPackketlength = sizeof(DataStruct);
  eSaunaState = Cold;
  memset(&TxData, 0x00, sizeof(DataStruct));     // init TX Data structure
  Serial.println(F("Init done"));
}

void loop()
{
  static unsigned int uiCounter;

  ADCSRA |= _BV(ADEN);    // enable ADC
  //TxData.uiTemperatureK = cc1100.get_tempK();     // get temp
  TxData.uiTemperatureK = uiDS18B20MeasureTempK();    // get temp
  TxData.uiBatteryVoltage = (unsigned int) ulMeasureVCC();    // battery Voltage
  ADCSRA &= ~_BV(ADEN);   // disable ADC
  cc1100.powerdown();

  switch (eSaunaState)
  {
    case Cold:
      Serial.println(F("COLD"));
      TxData.etTxProtocol = etSleep;
      TxData.ulSleepTimeMs = SleepTimeColdMs;

      if (TxData.uiTemperatureK > TempKHot)
      {
        Serial.println(F("COLD to HOT"));
        eSaunaState = Hot;
        TxData.etTxProtocol = etWarmingUp;
        TxData.ulSleepTimeMs = SleepTimeHotMs;
        uiCounter = 0;
        goto TX_DATA;
      }
      break;

    case Hot:
      Serial.println(F("HOT"));
      TxData.etTxProtocol = etMeasure;
      TxData.ulSleepTimeMs = SleepTimeHotMs;
      TxData.uiTxCounter = uiCounter;
      uiCounter++;

      Serial.print("TemperatureK: "); Serial.println(TxData.uiTemperatureK);
      Serial.print("etTxProtocol: "); Serial.println(TxData.etTxProtocol);
      Serial.print("SleepTimeMs: "); Serial.println(TxData.ulSleepTimeMs);
      Serial.print("uiTxCounter: "); Serial.println(TxData.uiTxCounter);
      Serial.print("uiBatteryVoltage: "); Serial.println(TxData.uiBatteryVoltage);

      if (TxData.uiTemperatureK < TempKCold)
      {
        Serial.println(F("HOT to COLD"));
        eSaunaState = Cold;
        TxData.ulSleepTimeMs = SleepTimeColdMs;
        TxData.etTxProtocol = etCoolingDown;
      }

TX_DATA:
      cc1100.wakeup();
      cc1100.send_packet(My_addr, Rx_addr, (uint8_t*) &TxData, ucTxPackketlength, 0);     //sents package over air. ACK is received via GPIO polling
      cc1100.powerdown();
      break;
  }

  Serial.print("Main State: "); Serial.println(eSaunaState);
  Serial.println("**** SLEEPING ****");
  Serial.println();
  delay(100);      // wait for TX done in addition to delay(50) in Sleeper::SleepMillis(long millis)

  g_sleeper.SleepMillis(TxData.ulSleepTimeMs);
  //delay(TxData.ulSleepTimeMs);
}

unsigned long ulMeasureVCC(void)
{
  uint16_t adc_low, adc_high;
  uint32_t adc_result;

  ADMUX |= _BV(REFS0);  // Voltage with external capacitor at AREF pin
  ADMUX &= ~_BV(REFS1);  // Voltage with external capacitor at AREF pin
  ADMUX |= ADC_InVBG;   // Input Channel Selection: 1.1V (VBG)
  delay(10);

  ADCSRA |= _BV(ADSC);  //Messung starten

  while (bitRead(ADCSRA, ADSC));  //warten bis Messung beendet ist
  //Ergebnisse des ADC zwischenspeichern. Wichtig: zuerst ADCL auslesen, dann ADCH
  adc_low = ADCL;
  adc_high = ADCH;

  adc_result = (adc_high << 8) | adc_low; //Gesamtergebniss der ADC-Messung

  // voltage reference rises with falling temperature
  return (1125300L / adc_result);  //Versorgungsspannung in mV berechnen (1100mV * 1023 = 1125300)
}

unsigned int uiDS18B20MeasureTempK(void)
{
  unsigned int uiTempK;

  digitalWrite(DS18B20_VCC, HIGH);
  DS18B20.requestTemperatures();
  uiTempK = DS18B20.getTempCByIndex(0);
  digitalWrite(DS18B20_VCC, LOW);
  
  return (uiTempK + 273);
}
