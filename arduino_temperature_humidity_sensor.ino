#include <Arduino.h>

#include <avr/sleep.h>
#include <KnxTpUart.h>
#include <DHT.h>

#define DHTPIN 4     
#define DHTTYPE DHT22 //DHT11, DHT21, DHT22

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
const String knxAddress              = "1.1.99";
const String groupTemperatureAddress = "0/7/13";
const String groupHumidityAddress    = "0/7/14";
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

const int interval = 60;
const int ledpin = 13;

DHT dht(DHTPIN, DHTTYPE);

KnxTpUart knx(&Serial, knxAddress);

void setup()
{
  Serial.begin(19200);
  UCSR0C = UCSR0C | B00100000; // Even Parity
  
  knx.uartReset();
  
  dht.begin();

  watchdogOn(); // Enable watchdog timer.

  // The following saves some extra power by disabling some peripherals I am not using.
  ADCSRA = ADCSRA & B01111111; // Disable ADC, ADEN bit7 to 0
  ACSR = B10000000; // Disable analog comparator, ACD bit7 to 1
  DIDR0 = DIDR0 | B00111111; // Disable digitale inputbuffer, set analoge input pins 0-5 to 1
}

void loop()
{
   // handle temperature
  float temperature = dht.readTemperature(); // Read temperature
  knx.groupWrite2ByteFloat(groupTemperatureAddress, temperature); // Send knx message

  // dht22 datasheet means, we have to wait for 2 seconds between measurements
  delay(3000); // wait for 3 seconds

  // handle humidity
  float humidity = dht.readHumidity(); // Read humidity  
  knx.groupWrite2ByteFloat(groupHumidityAddress, humidity); // Send knx message

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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set deepest sleep mode PWR_DOWN
  for(int i=0; i < seconds; i++)
  {
    sleep_enable(); // Enable sleep mode
    sleep_mode(); // Start sleep mode
    sleep_disable(); // Disable sleep mode 
  }
}

void watchdogOn()
{
  MCUSR = MCUSR & B11110111; // Disable reset flag, WDRF bit3 of MCUSR.
  WDTCSR = WDTCSR | B00011000; // Set bit 3+4 to be able to set prescaler later
  WDTCSR = B00000110; // Set watchdog prescaler to 128K > is round about 1 second
  WDTCSR = WDTCSR | B01000000; // Enable watchdog interrupt
  MCUSR = MCUSR & B11110111;
}

ISR(WDT_vect)
{
  //sleepcounter ++; // count sleepcycles
}
