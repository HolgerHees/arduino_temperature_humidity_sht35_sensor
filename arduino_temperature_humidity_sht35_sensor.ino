#include <Arduino.h>

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

#include <KnxTpUart.h>
#include <SHT85.h>

// FF Livingroom
//const String knxAddress              = "1.1.49";
//const String groupTemperatureAddress = "0/7/1";
//const String groupHumidityAddress    = "0/7/2";
// FF Boxroom
//const String knxAddress              = "1.1.29";
//const String groupTemperatureAddress = "0/7/3";
//const String groupHumidityAddress    = "0/7/4";
// FF Guestroom
//const String knxAddress              = "1.1.59";
//const String groupTemperatureAddress = "0/7/5";
//const String groupHumidityAddress    = "0/7/6";
// FF Guest WC
//const String knxAddress              = "1.1.79";
//const String groupTemperatureAddress = "0/7/7";
//const String groupHumidityAddress    = "0/7/8";
// FF Floor
//const String knxAddress              = "1.1.69";
//const String groupTemperatureAddress = "0/7/9";
//const String groupHumidityAddress    = "0/7/10";
// FF Utilityroom
//const String knxAddress              = "1.1.19";
//const String groupTemperatureAddress = "0/7/11";
//const String groupHumidityAddress    = "0/7/12";
// FF Garage
//const String knxAddress              = "1.1.99";
//const String groupTemperatureAddress = "0/7/13";
//const String groupHumidityAddress    = "0/7/14";
// SF Bedroom
//const String knxAddress              = "1.1.129";
//const String groupTemperatureAddress = "1/7/1";
//const String groupHumidityAddress    = "1/7/2";
// SF Dressingroom
//const String knxAddress              = "1.1.119";
//const String groupTemperatureAddress = "1/7/3";
//const String groupHumidityAddress    = "1/7/4";
// SF Child 1
//const String knxAddress              = "1.1.149";
//const String groupTemperatureAddress = "1/7/5";
//const String groupHumidityAddress    = "1/7/6";
// SF Child 2
//const String knxAddress              = "1.1.139";
//const String groupTemperatureAddress = "1/7/7";
//const String groupHumidityAddress    = "1/7/8";
// SF Bathroom
//const String knxAddress              = "1.1.169";
//const String groupTemperatureAddress = "1/7/9";
//const String groupHumidityAddress    = "1/7/10";
// Room: Floor
//const String knxAddress              = "1.1.159";
//const String groupTemperatureAddress = "1/7/11";
//const String groupHumidityAddress    = "1/7/12";
// Room: Attic
//const String knxAddress              = "1.1.179";
//const String groupTemperatureAddress = "1/7/13";
//const String groupHumidityAddress    = "1/7/14";

// FF Gardenhouse
const String knxAddress              = "1.1.211";
const String groupTemperatureAddress = "2/2/0";
const String groupHumidityAddress    = "2/2/1";

#define DEBUG
#define SHT35_ADDRESS         0x44
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

const int ledpin = 13;

volatile bool wdtTriggered;

SHT35 sht;

KnxTpUart knx(&Serial, knxAddress);

#ifdef DEBUG
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_FLUSH() Serial.flush()
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_FLUSH()

#endif

void setup()
{
  Serial.begin(115200);

  // The following saves some extra power by disabling some peripherals I am not using.
  ADCSRA &= ~(1 << ADEN); // Disable ADC, ADEN bit7 to 0

  ACSR = B10000000; // Disable analog comparator, ACD bit7 to 1
  DIDR0 = DIDR0 | B00111111; // Disable digitale inputbuffer, set analoge input pins 0-5 to 1
  
  //power_aca_disable()
  power_adc_disable();

  //Serial.begin(19200);
  //UCSR0C = UCSR0C | B00100000; // Even Parity
  //knx.uartReset();

  Wire.begin();
  sht.begin(SHT35_ADDRESS);
  Wire.setClock(100000);
}

void loop()
{
  DEBUG_PRINTLN("Loop");
  
  sht.read();

   // handle temperature
  float temperature = sht.getTemperature(); // Read temperature
  //knx.groupWrite2ByteFloat(groupTemperatureAddress, temperature); // Send knx message
  DEBUG_PRINTLN(temperature);

  // handle humidity
  float humidity = sht.getHumidity(); // Read humidity  
  //knx.groupWrite2ByteFloat(groupHumidityAddress, humidity); // Send knx message
  DEBUG_PRINTLN(humidity);

  // show Progress for 500 milliseconds
  showProgress( 500 );

  // randomize loop to avoid flooding knx bus with messages from different devices at the same time
  pwrDown( random( 56, 66 ) ); // Shutdown ATmega328 for 56 - 66 seconds;
}

void showProgress(int milliseconds)
{
  digitalWrite(ledpin, HIGH); // Enable LED
  delay( milliseconds );
  digitalWrite(ledpin, LOW);  // Disable LED
}

void pwrDown(int seconds)
{
  DEBUG_PRINTLN("Sleep");
  DEBUG_FLUSH();

  const uint8_t WDTsave = WDTCSR;  

  // force initial sleep mode reset
  wdtTriggered = true;

  power_twi_disable();

  for(int i=0; i < seconds; i++)
  {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set deepest sleep mode PWR_DOWN

    cli(); // disable interrupts for changing the registers

    if ( wdtTriggered )
    {
      wdtTriggered = false;

      MCUSR &= ~(1 << WDRF); // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
      WDTCSR |= (1 << WDCE) | (1 << WDE); // Watchdog change enabled for the next 4 cpu cycles
      WDTCSR  = (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0); // Set watchdog prescaler to 128K > is round about 1 second
      WDTCSR |= 1 << WDIE; // Enable the WD interrupt (note: no reset).
      wdt_reset();
    }
    else
    {
      i--;
    }

    sleep_enable();             // Set the SE (sleep enable) bit.

    sleep_bod_disable();        // must be call directly before sleep_cpu

    sei();                      // enable interrupts

    sleep_cpu();                // sleep ***

    sleep_disable();            // Clear the SE (sleep enable) bit.
  }

  power_twi_enable();

  cli();
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Watchdog change enabled for the next 4 cpu cycles
  WDTCSR = WDTsave; // restore saved WDT settings
  wdt_reset();
  sei();
}

ISR(WDT_vect)
{
    wdtTriggered = true;
    wdt_disable();
}
