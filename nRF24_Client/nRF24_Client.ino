// Include the libraries we need
#include <SPI.h>
#include <Encoder.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "RF24.h"
#include "Sleeper.h"

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(A0, A1);    // ce_pin, cs_pin
/**********************************************************/

#define SleepTimeColdMs   1800000   // 30 min
#define SleepTimeHotMs    30000     // 0,5 min

#define TemperatureArrayElements    60

// Pin defines
#define EncoderAPin   8
#define EncoderBPin   9
#define EncoderButtonPin 3

#define PotiOuputPin  2   // Drive Pin high to read Poti
#define BacklightPin  4
#define ContrastPin   A3

#define ONE_WIRE_BUS 9

// Global Variables
byte addresses[][6] = {"Master","Slave"};

struct DataStruct {
  unsigned int ui16RawTemperature;
  long lSleepTimeMs;
} RxData;

unsigned int TemperatureArray[TemperatureArrayElements];
long lEncoderPosition = 0;

Sleeper g_sleeper;
/* usage
  // Power down for 10 seconds.
  g_sleeper.SleepMillis(10000);
*/

Encoder myEnc(EncoderAPin, EncoderBPin);    // activates Pullup

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 13 - CLK
// pin 12 - MISO (unused)
// pin 11 - MOSI
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!

// onewire for Temperature conversion
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void setup()
{
  // start serial port
  Serial.begin(9600);
  Serial.println("**** Client ***");
  
  // setup I/O pins
  pinMode(EncoderButtonPin, INPUT_PULLUP);
  pinMode(PotiOuputPin, OUTPUT);
  digitalWrite(PotiOuputPin, LOW);

  // Backlight
  pinMode(BacklightPin, OUTPUT);
  digitalWrite(BacklightPin, LOW);    // P-Channel Mosfet

//  display.begin();

  // Read Poti and set constrast
  digitalWrite(PotiOuputPin, HIGH);
//  display.setContrast(analogRead(ContrastPin) >> 2);
  digitalWrite(PotiOuputPin, LOW);
//  display.clearDisplay();
//  display.display();


  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber)
  {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }
  else
  {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }

   radio.startListening();

  //attachInterrupt(digitalPinToInterrupt(EncoderAPin), ReadEncoderAB, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(EncoderButtonPin), isr_EncoderButtonPin , HIGH);

  // Display Selftest
//  display.setCursor(5, 15);       // x, y coordinates
//  display.println("SELFTEST \n");
//  display.display();

  Serial.println("init done");
}

void loop()
{
  if(radio.available())
  {
    Serial.println("RX done:");
    
    while(radio.available())
    {
      radio.read(&RxData, sizeof(RxData));  
      Serial.println(DallasTemperature::rawToCelsius(RxData.ui16RawTemperature));
      Serial.println((RxData.lSleepTimeMs));
    }
  }
  
  //  const static unsigned long ulMillisOffset = millis();
  //
  //  if (!RxData.lSleepTimeMs)
  //  {
  //    // RADIO
  //    radio.powerUp();
  //    radio.startListening();
  //
  //    while (!radio.available())  // wait for rx
  //    {
  //      __asm__ __volatile__ ("nop\n\t");
  //    }
  //
  //    radio.stopListening();
  //    radio.read(&RxData, sizeof(RxData));
  //    radio.powerDown();
  //
  //    // SAVE DATA
  //    // shift last value to right, insert last value
  //    memmove(&TemperatureArray[1], &TemperatureArray[0], sizeof(TemperatureArray));
  //    TemperatureArray[0] = RxData.ui16RawTemperature;
  //    Serial.println("Temperature");
  //    Serial.println(DallasTemperature::rawToCelsius(RxData.ui16RawTemperature));
  //  }
  //  else
  //  {
  //    RxData.lSleepTimeMs -= (millis() - ulMillisOffset);
  //  }
  //
  //  // DISPLAY
  //  DisplayGraph(TemperatureArray);
  //
  //  if (EncoderButtonPin)
  //  {
  //    // wakeup
  //    // enter menu
  //    // listen for next rx
  //  }

  //SLEEP
  //g_sleeper.SleepMillis(RxData.lSleepTimeMs);
}


void DisplayGraph(unsigned int* TempDataArray)
{
  //Display resolution 84x48 px :  60 temp.measurements

  for (unsigned int x; x > TemperatureArrayElements; x++ )
  {
    display.clearDisplay();
    display.drawFastVLine(0, 0, display.height() - 1, BLACK); // draw x scale
    display.drawFastHLine(0, 0, display.width() - 1, BLACK);   // draw y scale
    display.drawPixel(x, DallasTemperature::rawToCelsius(TempDataArray[x]) - 30, BLACK);   // TempDataArray in 1/128 DegC, 30 degC offset, scaling 30 to 104 degC
  }

  display.display();
}

void ReadEncoderAB()
{
  lEncoderPosition = myEnc.read();
}

void isr_EncoderButtonPin()
{
  detachInterrupt(digitalPinToInterrupt(EncoderButtonPin));
}

void UserMenu()
{
  char *MenuItem[] = {"Entry A", "Entry B", "Entry C", 0};

}

