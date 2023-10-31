#include <Arduino.h>

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

// successor, but not used => https://github.com/KONNEKTING/KonnektingDeviceLibrary
#include <KnxTpUart.h>  // https://github.com/thorsten-gehrig/arduino-tpuart-knx-user-forum
#include <SHT85.h>      // https://github.com/RobTillaart/SHT85

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
const String knxAddress = "1.1.211";
const String groupTemperatureAddress = "2/2/0";
const String groupHumidityAddress = "2/2/1";

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_FLUSH() Serial.flush()
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_FLUSH()
#endif

#define SHT35_ADDRESS 0x44
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

const int ledpin = 13;

volatile bool wdtTriggered = false;

volatile float activeTemperature = 9999.0;
volatile float activeTemperatureRounded = 9999.0;
volatile float lastTemperatureRounded = 9999.0;
volatile int skippedMinorTemperatureChange = 0;
volatile int skippedTemperatureSubmission = 0;

volatile float activeHumidity = 9999.0;
volatile float activeHumidityRounded = 9999.0;
volatile float lastHumidityRounded = 9999.0;
volatile int skippedMinorHumidityChange = 0;
volatile int skippedHumiditySubmission = 0;

SHT35 sht;

KnxTpUart knx(&Serial, knxAddress);

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#else
  Serial.begin(19200);
  UCSR0C = UCSR0C | B00100000;  // Even Parity
  knx.uartReset();
#endif

  Wire.begin();
  sht.begin(SHT35_ADDRESS);
  Wire.setClock(100000);
}

void loop() {
  DEBUG_PRINTLN("Loop");

  bool isSuccess = sht.read(false);  // slow (repeatable) messurement
  if (isSuccess) 
  {
    // **** HANDLE TEMPERATURE ****
    float _currentTemperature = sht.getTemperature();
    float _currentTemperatureRounded = round(_currentTemperature * 10) / 10.0;
    bool _temperatureChanged = false;

    if (_currentTemperatureRounded != activeTemperatureRounded) 
    {
      // Handle temperature huge temperatur changes immediately or tiny changes after 5 "stable" minutes
      if (abs(_currentTemperature - activeTemperature) >= 0.1 || skippedMinorTemperatureChange >= 5) 
      {
        _temperatureChanged = true;
        activeTemperature = _currentTemperature;
        activeTemperatureRounded = _currentTemperatureRounded;
        skippedMinorTemperatureChange = 0;
      } 
      else if (lastTemperatureRounded == _currentTemperatureRounded) skippedMinorTemperatureChange += 1;
      else skippedMinorTemperatureChange = 0;
    } 
    else skippedMinorTemperatureChange = 0;

    lastTemperatureRounded = _currentTemperatureRounded;

    // Submit on temperature changes or resubmit after 15 minutes
    if (_temperatureChanged || skippedTemperatureSubmission >= 15) 
    {
      skippedTemperatureSubmission = 0;
#ifndef DEBUG
      knx.groupWrite2ByteFloat(groupTemperatureAddress, activeTemperatureRounded);  // Send knx message
#endif
      DEBUG_PRINTLN(activeTemperatureRounded);
    } 
    else 
    {
      skippedTemperatureSubmission += 1;
      DEBUG_PRINTLN("No temperature change");
    }

    // **** HANDLE HUMIDITY ****
    float _currentHumidity = sht.getHumidity();
    float _currentHumidityRounded = round(_currentHumidity);
    bool _humidityChanged = false;

    if (_currentHumidityRounded != activeHumidityRounded) 
    {
      if (abs(_currentHumidity - activeHumidity) >= 1 || skippedMinorHumidityChange >= 5) 
      {
        _humidityChanged = true;
        activeHumidity= _currentHumidity;
        activeHumidityRounded = _currentHumidityRounded;
        skippedMinorHumidityChange = 0;
      } 
      else if (lastHumidityRounded == _currentHumidityRounded) skippedMinorHumidityChange += 1;
      else skippedMinorHumidityChange = 0;
    } 
    else skippedMinorHumidityChange = 0;
    
    lastHumidityRounded = _currentHumidityRounded;

    // Submit on humidity changes or resubmit after 15 minutes
    if (_humidityChanged || skippedHumiditySubmission >= 15) 
    {
      skippedHumiditySubmission = 0;
#ifndef DEBUG
      knx.groupWrite2ByteFloat(groupHumidityAddress, activeHumidityRounded);  // Send knx message
#endif
      DEBUG_PRINTLN(activeHumidityRounded);
    }
    else
    {
      skippedHumiditySubmission += 1;
      DEBUG_PRINTLN("No humidity change");
    }
  } 
  else
  {
    showProgress(500);
    delay(1000);
  }

  showProgress(500);  // show Progress for 500 milliseconds

  pwrDown(random(56, 66));  // Shutdown ATmega328 for 56 - 66 seconds;
}

void showProgress(int milliseconds) {
  digitalWrite(ledpin, HIGH);  // Enable LED
  delay(milliseconds);
  digitalWrite(ledpin, LOW);  // Disable LED
}

void pwrDown(int seconds) {
  DEBUG_PRINTLN("Sleep");
  DEBUG_FLUSH();

  const uint8_t WDTsave = WDTCSR;

  wdtTriggered = true;  // force initial sleep mode reset

  //ACSR = B10000000; // Disable analog comparator, ACD bit7 to 1
  //DIDR0 = DIDR0 | B00111111; // Disable digitale inputbuffer, set analoge input pins 0-5 to 1

  ADCSRA &= ~(1 << ADEN);  // Ensure AD control register is disable before power disable
  power_all_disable();
  /*power_adc_disable();
  power_spi_disable();
  //power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();

  power_usart0_disable();
  power_twi_disable();*/

  for (int i = 0; i < seconds; i++) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set deepest sleep mode PWR_DOWN

    cli();  // disable interrupts for changing the registers

    if (wdtTriggered) {
      wdtTriggered = false;

      MCUSR &= ~(1 << WDRF);                                           // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
      WDTCSR |= (1 << WDCE) | (1 << WDE);                              // Watchdog change enabled for the next 4 cpu cycles
      WDTCSR = (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);  // Set watchdog prescaler to 128K > is round about 1 second
      WDTCSR |= 1 << WDIE;                                             // Enable the WD interrupt (note: no reset).
      wdt_reset();
    } else {
      i--;
    }

    sleep_enable();  // Set the SE (sleep enable) bit.

    sleep_bod_disable();  // must be call directly before sleep_cpu

    sei();  // enable interrupts

    sleep_cpu();  // sleep ***

    sleep_disable();  // Clear the SE (sleep enable) bit.
  }

  //power_usart0_enable();
  //power_twi_enable();
  power_all_enable();
  ADCSRA |= (1 << ADEN);  // enable ADC

  cli();
  WDTCSR |= (1 << WDCE) | (1 << WDE);  // Watchdog change enabled for the next 4 cpu cycles
  WDTCSR = WDTsave;                    // restore saved WDT settings
  wdt_reset();
  sei();
}

ISR(WDT_vect) {
  wdtTriggered = true;
  wdt_disable();
}
