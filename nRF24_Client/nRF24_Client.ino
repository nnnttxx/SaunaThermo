// ATmega328 with ext. OSC 16 MHz @ 3.3V

// Include the libraries we need
#include <cc1100.h>
#include <EEPROM.h>

// EEPROM address define
#define EEInitMarker      0
#define EECalDoneAddress  1
#define EECalDataAddress  2

// Pin defines
#define PWM_Pin         9       // PWM output Pins 3, 5, 6, 9, 10, 11. The PWM outputs generated on pins 5 and 6 will have higher-than-expected duty cycles.
#define PushButton_Pin  3       // ext. int Pins 2, 3

// ADC Input defines
#define ADC_InCH0   0
#define ADC_InCH7   7
#define ADC_InTemp  8
#define ADC_InVBG  14
#define ADC_InGND  15

#define VCCMin    3000    // Minimum ulSlaveBatteryVoltage voltage for PWM output
#define ulRxTmoCounterDefault   75000   // 75s

// status output on scale 0 - 100 degreeC
#define fPWMdutyNoSignal   13     // show no signal
#define fPWMdutyLowBatt    26     // show low Battery 
#define fPWMdutyIdle       38     // do nothing


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
  unsigned int ui16TemperatureK;
  eProtocol etTxProtocol;
  unsigned long ulSleepTimeMs;              // sleep time ms for 16MHz
  unsigned int ucTxCounter;
  unsigned int uiMasterBatteryVoltage;  // Master battery voltage
  unsigned char ucLqi;      // dummy
  unsigned char ucCrc;        // dummy
};

enum eMainState
{
  Idle,
  Receiving,
  LowBatterySlave,
  ReceiveTimeout
} eSlaveState;

// Global Variables
uint8_t my_addr, rx_addr, ucRxPacketlength, ucSender, ucLqi;
int8_t cRssi_dbm;
volatile uint8_t ucCc1101_packet_available;
uint32_t ulSlaveBatteryVoltage, ulRxTmoCounter, ulReferenceVoltage1V1;
float fPWMduty;  // PWM dutyclycle 255 steps
DataStruct RxData;
volatile unsigned long ulWaitForRxMs;


//init CC1100 constructor
CC1100 cc1100;

void setup()
{
  // start serial port
  Serial.begin(9600);
  Serial.println(F("**** Slave ***"));

  pinMode(PushButton_Pin, INPUT_PULLUP);

  // init ADC
  vInitADC();

  // disable ADC for power saving
  ADCSRA &= ~_BV(ADEN);   // disable ADC

  // init CC1101 RF-module
  if (cc1100.begin(CC1100_MODE_GFSK_1_2_kb, CC1100_FREQ_868MHZ, 1, 10, 3)) // modulation mode, frequency, channel, PA level in dBm, own address
  {
#ifndef CC1100_DEBUG
    Serial.println(F("Init CC1101 successful"));
#endif
  }

  cc1100.spi_write_register(IOCFG2, 0x07);  // Asserts when a packet has been received with CRC OK.
  // De-asserts when the first byte is read from the RX FIFO.

  cc1100.show_main_settings();             // shows setting debug messages to UART
  cc1100.show_register_settings();         // shows current CC1101 register values

  // init EEPROM if never used before
  if (EEPROM.read(EEInitMarker) == 255)
  {
    vClearEEPROM();
  }

  // get init marker and calibration data from EEPROM
  Serial.print(F("Voltage Calibration init Marker: ")); Serial.println(EEPROM.read(EECalDoneAddress));
  ulReferenceVoltage1V1 = EEPROM.get(EECalDataAddress, ulReferenceVoltage1V1);
  Serial.print(F("Referene Voltage in EEPROM: ")); Serial.print(ulReferenceVoltage1V1); Serial.println(F("mV"));

  // voltmeter selftest
  analogWrite(PWM_Pin, 255);
  cc1100.receive();
  ulSlaveBatteryVoltage = ulMeasureVCC();     // measure Supply Voltage with max. load
  cc1100.powerdown();
  delay(2000);
  analogWrite(PWM_Pin, 0);

  // enter state machine
  if (ulSlaveBatteryVoltage < VCCMin)
  {
    eSlaveState = LowBatterySlave;
  }
  else
  {
    eSlaveState = Idle;
    cc1100.receive();                        //set to RECEIVE mode
  }

  ulRxTmoCounter = ulRxTmoCounterDefault;

  // init interrrupt function for available packet
  attachInterrupt(digitalPinToInterrupt(GDO2), rf_available_int, RISING);
  attachInterrupt(digitalPinToInterrupt(PushButton_Pin), Rssi_dbm_int, FALLING);

  Serial.println(F("Init done"));
}

void loop()
{
  switch (eSlaveState)
  {
    case Idle:
      Serial.println(F("State Idle"));
      analogWrite(PWM_Pin, fPWMdutyIdle);

      if (ucCc1101_packet_available == TRUE)
      {
        Serial.println(F("RX done"));
        eSlaveState = Receiving;
      }
      break;

    case Receiving:
      Serial.println(F("State RX"));
      ulSlaveBatteryVoltage = ulMeasureVCC();
      Serial.print(F("ulSlaveBatteryVoltage Voltage: ")); Serial.println(ulSlaveBatteryVoltage);

      if (ulSlaveBatteryVoltage < VCCMin)
      {
        cc1100.powerdown();
        eSlaveState = LowBatterySlave;
        break;
      }

      while (ucCc1101_packet_available == FALSE)
      {
        // wait for RX
        delay(1);
        ulRxTmoCounter--;

        if (ulRxTmoCounter == 0)
        {
          eSlaveState = ReceiveTimeout;
          ulRxTmoCounter = ulRxTmoCounterDefault;
          break;
        }
      }

      ucCc1101_packet_available = FALSE;
      ulRxTmoCounter = ulRxTmoCounterDefault;

      Serial.print(F("ulWaitForRxMs: ")); Serial.println(ulWaitForRxMs);
      Serial.println();
      Serial.print(F("TemperatureK: ")); Serial.println(RxData.ui16TemperatureK);
      Serial.print(F("etTxProtocol: ")); Serial.println(RxData.etTxProtocol);
      Serial.print(F("SleepTimeMs: ")); Serial.println(RxData.ulSleepTimeMs);
      Serial.print(F("ucTxCounter: ")); Serial.println(RxData.ucTxCounter);
      Serial.print(F("uiMasterBatteryVoltage:  ")); Serial.println(RxData.uiMasterBatteryVoltage);

      fPWMduty = VCCMin / (float) ulSlaveBatteryVoltage * (RxData.ui16TemperatureK - (273 + 10)) * 2.55F;    // 10 deg scale offset, Scaling 10-110 deg.
      // 2.31 = 255/(110-10)
      analogWrite(PWM_Pin, (unsigned char)fPWMduty);

      cc1100.powerdown();
      Serial.println(F("Delay "));
      Serial.println();
      delay(RxData.ulSleepTimeMs * 1.11F);      // factor 1.11 @ 30s, 3V Slave, no ACK (broadcast)
      // factor 1.12 @ 30s, 3.3V Slave, with ACK
      cc1100.receive();             // wakeup and receive
      ulWaitForRxMs = millis();
      break;

    case LowBatterySlave:
      Serial.println(F("State Low Batt"));
      ulSlaveBatteryVoltage = ulMeasureVCC();
      Serial.print(F("ulSlaveBatteryVoltage Voltage: ")); Serial.println(ulSlaveBatteryVoltage);
      cc1100.powerdown();

      if (ulSlaveBatteryVoltage > (VCCMin + 300))   // 3.3V
      {
        eSlaveState = Idle;
        cc1100.receive();
        break;
      }

      fPWMduty = VCCMin / (float) ulSlaveBatteryVoltage * fPWMdutyLowBatt;
      analogWrite(PWM_Pin, fPWMduty);

      delay(60000);     // 60s delay
      break;

    case ReceiveTimeout:
      Serial.println(F("State Rx Timeout"));
      fPWMduty = VCCMin / (float) ulSlaveBatteryVoltage * fPWMdutyNoSignal;
      analogWrite(PWM_Pin, fPWMduty);

      // Tx not sleeping
      if (ucCc1101_packet_available == TRUE)
      {
        eSlaveState = Idle;
      }
      break;
  }
}


unsigned long ulMeasureVCC(void)
{
  uint16_t adc_low, adc_high;
  uint32_t adc_result;

  ADMUX |= _BV(REFS0);  // ulSlaveBatteryVoltage with external capacitor at AREF pin
  ADMUX |= ADC_InVBG;   // Input Channel Selection: 1.1V (VBG)
  ADCSRA |= _BV(ADEN);  // enable ADC
  ADCSRA |= _BV(ADSC);  // Messung starten
  while (bitRead(ADCSRA, ADSC));  //warten bis Messung beendet ist

  //Ergebnisse des ADC zwischenspeichern. Wichtig: zuerst ADCL auslesen, dann ADCH
  adc_low = ADCL;
  adc_high = ADCH;

  ADCSRA &= ~_BV(ADEN);   // disable ADC
  adc_result = (adc_high << 8) | adc_low; //Gesamtergebniss der ADC-Messung

  // calibrate internal reference
  if (EEPROM.read(EECalDoneAddress) != 42)
  {
    Serial.println(F("Caibrating internal Referene Voltage with Supply Voltage 3.3V"));

    // assume supply voltage is 3V3 when calibratiing
    ulReferenceVoltage1V1 = (3300 * adc_result) >> 10;
    Serial.print(F("Calculated Referene Voltage 1.1V: ")); Serial.print(ulReferenceVoltage1V1); Serial.println(F("mV"));

    // write calibration data to EEPROM
    EEPROM.put(EECalDoneAddress, 42);                      // write 0x01
    EEPROM.put(EECalDataAddress, ulReferenceVoltage1V1);
    Serial.print(F("Referene Voltage written to EEPROM: ")); Serial.print(EEPROM.get(EECalDataAddress, ulReferenceVoltage1V1)); Serial.println(F(" mV"));
  }

  return ((ulReferenceVoltage1V1 << 10) / adc_result);
}


void rf_available_int(void)
{
  detachInterrupt(digitalPinToInterrupt(GDO2));

  ulWaitForRxMs = millis() - ulWaitForRxMs;
  if (cc1100.packet_available())
  {
    //    Serial.println(F("Packet available"));
    cc1100.get_payload((uint8_t *)&RxData, ucRxPacketlength, rx_addr, ucSender, cRssi_dbm, ucLqi);     // stores the payload data
    ucCc1101_packet_available = TRUE;                                                     // set flag that an package is in RX buffer
  }

  attachInterrupt(digitalPinToInterrupt(GDO2), rf_available_int, RISING);
}


void Rssi_dbm_int (void)
{
  detachInterrupt(digitalPinToInterrupt(PushButton_Pin));
  detachInterrupt(digitalPinToInterrupt(GDO2));
  Serial.print(F("Rssi_dbm_int "));

  while (digitalRead(PushButton_Pin) == 0)
  {
    fPWMduty = VCCMin / (float) ulSlaveBatteryVoltage * (abs(cRssi_dbm) - 10) * 2.55F;    // -100dBm eq. 100 degC
    analogWrite(PWM_Pin, fPWMduty);
  }

  eSlaveState = Idle;
  analogWrite(PWM_Pin, fPWMdutyIdle);

  attachInterrupt(digitalPinToInterrupt(PushButton_Pin), Rssi_dbm_int, FALLING);
  attachInterrupt(digitalPinToInterrupt(GDO2), rf_available_int, RISING);
}

void vClearEEPROM(void)
{
  unsigned int i;

  Serial.println(F("Clearing EEPROM"));

  for (i = 0 ; i < EEPROM.length() ; i++)
  {
    EEPROM.write(i, 0);
  }

  Serial.println(F("Done !"));
}

void vInitADC(void)
{
  // dummy conversion to initialize ADC
  ADMUX |= _BV(REFS0);  // ulSlaveBatteryVoltage with external capacitor at AREF pin
  ADMUX |= ADC_InVBG;   // Input Channel Selection: 1.1V (VBG)
  ADCSRA |= _BV(ADEN);  // enable ADC
  ADCSRA |= _BV(ADSC);  // Messung starten
  while (bitRead(ADCSRA, ADSC));  //warten bis Messung beendet ist
}

