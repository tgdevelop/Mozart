/* tg_mozart uses an Arduino Uno with IR leds and a One Wire interface
*  It's an AC controller via IR. Measures Temp with One Wire
*  DS1820.
*
*  The Arduino IR Library assumes the LED transmitter is hooked to PWM pin 3
*  This version does not have wireles verification
****************************************************/

#include <IRremote.h>
#include <OneWire.h>
#include <avr/wdt.h>

OneWire  ds(5);               // One Wire on pin 5
#define ledPin 8              // heartbeat LED
#define TARGET_TEMP 80        // temperature setpoint
int buttonPin = 7;            // pushbutton connected to digital #3
int pwmPin = 3;
int statusPin = 9;            // AC status led (indicates AC on or off)
uint8_t AC_status;
IRsend irsend;                // IR library send object



uint8_t roomTemp[] = { 0x28, 0xE3, 0xAB, 0x96, 0x01, 0x00, 0x00, 0x6E };
boolean ping;
float roomTempF;
uint8_t verification;
uint8_t data[8];

// Storage for the IR code
int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
// unsigned int rawCodes[RAWBUF]; // The durations if raw
unsigned int rawCodes[RAWBUF] = {4700, 4200, 800, 1400, 800, 300, 800, 350, 750, 350, 750, 350, 800, 300, 750, 350, 800, 350, 750, 350, 750, 1500, 700, 350, 800, 350, 700, 1500, 750, 1500, 700, 1500, 750, 1450, 750, 1500, 700, 1500, 750, 350, 750, 400, 700};
int codeLen = 43; // The length of the code

long  previousMillis;



void setup() {

        Serial.begin(57600);
        pinMode(buttonPin, INPUT);        // set the pushbutton pin to input
        digitalWrite(buttonPin, HIGH);    // pullup on
        pinMode(pwmPin, OUTPUT);          // IR led
        digitalWrite(pwmPin, HIGH);       // idle the LED's
        pinMode(statusPin, OUTPUT);       // AC status pin led
        digitalWrite(statusPin, LOW);       // AC initially off
        wdt_enable(WDTO_4S);              // watchdog set to 4 seconds
        roomTempF = 75;                   // initialize temp to turn off value
        AC_status = 0;
        ping = 0;                         // init the temperature reading control

        // Serial.println("Setup done");

}


void loop() {
        static boolean led_state;
        uint8_t i;
          wdt_reset();
         unsigned long currentMillis = millis();
         //************************************
        // Periodically read the sensors
        if (currentMillis - previousMillis > 5000)
        {
          previousMillis = currentMillis;
          // read the sensors
         // Serial.println("Sensor read");
          wdt_reset();
          digitalWrite(ledPin, led_state = !led_state);  //blink the heartbeat LED
           wdt_reset();
          if (!ping)
            {
             ds.reset();
             ds.select(roomTemp);
             ds.write(0x44,1);         // start conversion, with parasite power on at the end
            }
          else
            {
            roomTempF =  get_temperature();
            Serial.print("Temp F: ");
            Serial.println(roomTempF);
            }
          ping = !ping;
          wdt_reset();
        }

       // activate the AC control IR LED's if we're over the target temperature
       // set the AC_status to on
       if ((roomTempF > (TARGET_TEMP+0.5)) && (AC_status == 0))
           {
             toggle_AC();
             if (get_status() == 1 )
                 AC_status = 1;
             else
                 AC_status = 0;
           }

       // deactivate the AC control by sending the IR codes, set the AC_status variable to off
         if ((roomTempF < (TARGET_TEMP - 0.5)) && (AC_status == 1))
           {
             toggle_AC();
             AC_status = 0;

           }

       // Set the status LED to indicate AC on or off
         if (AC_status)
             digitalWrite(statusPin, HIGH);
         else
             digitalWrite(statusPin, LOW);

       if (! digitalRead(buttonPin)) {
           // pushbutton pressed

            Serial.println("Sending IR signal");
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             delay(5);
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             delay(5);
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             digitalWrite(pwmPin, HIGH);    // important!!!!!  This idles the LED's
             delay(3*1000);  // wait 3 seconds (* 1000 milliseconds)
             wdt_reset();
             //Serial.println("Resume looop");
              pinMode(buttonPin, INPUT);      // set the pushbutton pin to input
             digitalWrite(buttonPin, HIGH);  // pullup on



     }

}


float get_temperature()
{
   uint8_t i, type_s;
   float celsius, fahrenheit, half;
   uint8_t present;

    present = ds.reset();
            ds.select(roomTemp);
            ds.write(0xBE);         // Read Scratchpad

           // Serial.println(present,HEX);
           // Serial.print(" ");
            for ( i = 0; i < 9; i++) {           // we need 9 bytes
                 data[i] = ds.read();
              //   Serial.print(data[i], HEX);
              //   Serial.print(" ");
                  }

 // convert the data to actual temperature
  type_s = 0;
  unsigned int raw = (data[1] << 8) | data[0];
  if (raw & 0x0008)
      half = 0.5;
  else half = 0;
  celsius = (float)(raw >> 4) + half;
  fahrenheit = celsius * 1.8 + 32.0;
 // Serial.print("  Temperature = ");
 // Serial.print(celsius);
 // Serial.print(" Celsius, ");
 // Serial.print(fahrenheit);
 // Serial.println(" Fahrenheit");

  return fahrenheit;
}

// Toggle the AC on or off
void toggle_AC(void)
{
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             delay(5);
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             delay(5);
             irsend.sendRaw(rawCodes, codeLen, 38);
             wdt_reset();
             delay(5);
             digitalWrite(pwmPin, HIGH);    // important!!!!!  This idles the LED's
}

// send a request to the current sensor for power status
uint8_t get_status(void)
{
  uint8_t retval;
  Serial.println("?8");
  delay(500);     // allow time for status return

  return retval;
}
