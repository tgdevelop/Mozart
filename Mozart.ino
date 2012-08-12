/* tg_mozart uses an Arduino Uno with IR leds and a One Wire interface
*  It's an AC controller via IR. Measures Temp with One Wire
*  DS1820.
*
*  The Arduino IR Library assumes the LED transmitter is hooked to PWM pin 3
*  This version does have wireles verification
****************************************************/

#include <IRremote.h>
#include <OneWire.h>
#include <avr/wdt.h>

OneWire  ds(5);               // One Wire on pin 5
#define ledPin 8              // heartbeat LED
#define TARGET_TEMP 80        // temperature setpoint
#define TMRDOG   30           // watchdog type resp/ack timer
#define INVALID 2             // define a bad response
// the following defines are the state machine
#define IDLE 0
#define REQ 5
#define WAIT 6
#define RXSER 7


int buttonPin = 7;            // pushbutton connected to digital #3
int pwmPin = 3;
int statusPin = 9;            // AC status led (indicates AC on or off)
uint8_t AC_status;
IRsend irsend;                // IR library send object



uint8_t roomTemp[] = { 0x28, 0xE3, 0xAB, 0x96, 0x01, 0x00, 0x00, 0x6E };
boolean ping;
float roomTempF;
uint8_t verification;
uint8_t data[16];

uint8_t tmr0;       // sensor read timer
uint8_t tmr1;       // power status request timer
uint8_t tmr2;       //
uint8_t tmr3;       //


char ch, ch2, ch3;
static boolean led_state;
// Storage for the IR code
int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
// unsigned int rawCodes[RAWBUF]; // The durations if raw
unsigned int rawCodes[RAWBUF] = {4700, 4200, 800, 1400, 800, 300, 800, 350, 750, 350, 750, 350, 800, 300, 750, 350, 800, 350, 750, 350, 750, 1500, 700, 350, 800, 350, 700, 1500, 750, 1500, 700, 1500, 750, 1450, 750, 1500, 700, 1500, 750, 350, 750, 400, 700};
int codeLen = 43; // The length of the code

unsigned long  previousMillis;
unsigned long currentMillis;

unsigned long newMillis;
typedef struct {
  uint8_t state;		  // the state of the verify
  uint8_t retry;                  // flag to turn retry on/off
  uint8_t pwrdly;                 // flag to indicate we want a power on delay
  uint8_t mode;                 // mode = running or off
} _vfy;

_vfy verify;                  // make the verify structure a global instance

void setup() {

        Serial.begin(57600);
        pinMode(buttonPin, INPUT);        // set the pushbutton pin to input
        digitalWrite(buttonPin, HIGH);    // pullup on
        pinMode(pwmPin, OUTPUT);          // IR led
        digitalWrite(pwmPin, HIGH);       // idle the LED's
        pinMode(statusPin, OUTPUT);       // AC status pin led
        digitalWrite(statusPin, LOW);       // AC initially off
        wdt_enable(WDTO_2S);              // watchdog set to 4 seconds
        roomTempF = 75;                   // initialize temp to turn off value
        AC_status = 0;
        ping = 0;                         // init the temperature reading control
        tmr2 = TMRDOG;                        // AC watchdog
        verify.state = IDLE;
        Serial.println("Setup done");

}


void loop() {

          uint8_t i, val;
          wdt_reset();

         //************************************
         // General purpose seconds timer maintenance
         // At top of loop, entered when current millis exceeds one second
         // since last millis. Used like this to avoid interrupts.
         currentMillis = millis();
         if (currentMillis - previousMillis > 1000)
             {
              // Serial.println(tmr0);
              previousMillis = currentMillis;
              if (tmr0) tmr0--;
              if (tmr1) tmr1--;
              if (tmr2) tmr2--;
              if (tmr3) tmr3--;
             }

         //************************************
        // Periodically read the sensors every tmr0 seconds (currently 5 secs)
        if (!tmr0)
        {
          tmr0 = 5;
          do_sensor();
        }

        if (!tmr2)
        	{
        		//Serial.print("*");
        		val = quick_status();
        		if ((val != AC_status) && (val !=2))
                           {
                            Serial.println("Synch error");
                            val = quick_status();                   // try again to be sure
                            if (val != INVALID) AC_status = val;    // if still out of synch, change AC_status
                            }
        		if (val==INVALID) Serial.println("Invalid");

        		tmr2 = TMRDOG;
        	}

       // *********************************************************************
       // activate the AC control IR LED's if we're above the target temperature
       // turn the A/C unit on, load time delay and a request
       if ((roomTempF > (TARGET_TEMP+0.7)) && (AC_status == 0))
           {
             // toggle the AC if we're not waiting for a verify
             if (verify.state == IDLE)
                   {
                   toggle_AC();
                   verify.state = REQ;    // state machine setup for verification
                   verify.pwrdly = 45;    // this is a power on event, indicate a longer wait
                   verify.mode = 1;       // This a power on (1)
                   }
             // AC_status = 1;

             wdt_reset();
           }

       // deactivate the AC control by sending the IR codes, set the AC_status variable to off
         if ((roomTempF < (TARGET_TEMP - 1.0)) && (AC_status == 1))
           {
             // toggle the AC if we're not waiting for a verify
             if (verify.state == IDLE)
                   {
                   toggle_AC();
                   // AC_status = 0;
                   verify.state = REQ;    // state machine setup for verification
                   verify.pwrdly = 15;
                   verify.mode = 0 ;      // this is a power off
                   }
             wdt_reset();
           }

       // Set the status LED to indicate AC on or off
         if (AC_status)
             digitalWrite(statusPin, HIGH);
         else
             digitalWrite(statusPin, LOW);

       // if verification is in progress, process the state
       if (verify.state != IDLE)
           {
              do_statemachine();
           }  // end verify state machine processor

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
             delay(2*1000);  // wait 3 seconds (* 1000 milliseconds)
             wdt_reset();
             delay(1*1000);
             //Serial.println("Resume looop");
              pinMode(buttonPin, INPUT);      // set the pushbutton pin to input
             digitalWrite(buttonPin, HIGH);  // pullup on
     }




}   // end loop


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
             Serial.println("$8");    // this indicates IR toggle over the air
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


// Even though the hardware was flushed, the software buffer seems to have chars in it
// this sequence of reads empties the buffer
void soft_flush(void)
{
   char chx;
   while (Serial.available()>0)
                     {
                      chx = Serial.read();
                      wdt_reset();
                     }
}

// get quick status of the AC unit. No state machine, etc
// down and dirty request, decode response
uint8_t quick_status(void)
{
  char ch;
  unsigned long prevrespMillis,currrespMillis;
  uint8_t retval = INVALID;     // init to invalid state, set this valid later when a valid state is returned
  Serial.println("?8");
  currrespMillis = millis();
  prevrespMillis = currrespMillis;
  while(currrespMillis - prevrespMillis < 2500)
         {
         if (Serial.available()>2) break;     // a correct response has at least 3 characters
         // prevrespMillis = currrespMillis;
         currrespMillis = millis();
         wdt_reset();
         }


  if (Serial.available()>2)          // if at least 3 characters, read the buffer
        {
         ch2 = Serial.read();
         delay(1);
         ch2 = Serial.read();
         delay(1);
         ch3 = Serial.read();
         if (ch3 == '0') retval = 0;
         if (ch3 == '1') retval = 1;
         wdt_reset();
         print_response();
        }
        if (Serial.available()) soft_flush();    // if the buffer still has chars, flush it
        return retval;                           // return what we got above, either INVALID or a good value
}

void print_response(void)
{
       Serial.print("]");                         // verify back (debug pnly)
       Serial.print(ch2);
       Serial.println(ch3);
       wdt_reset();
}

// Do the ds1820 temperature reading dance
void do_sensor(void)
{
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
            Serial.print("F: ");
            Serial.println(roomTempF);
            }
          ping = !ping;
          wdt_reset();


}

// process the request/ack state machine for Xbee comm
void do_statemachine(void)
{
uint8_t retstat;
switch (verify.state)
              {
              case REQ:

                  verify.retry = 5;                  // set the retry count
                  tmr1 = verify.pwrdly;              // set the request delayed read timer with desired time
                  tmr2 = 255;                        // essentially turn the watchdog off until we settle
                  verify.state = WAIT;
                  wdt_reset();
                  break;
              case WAIT:

                  wdt_reset();

                         if (!tmr1)                      // wait indicated seconds for response request
                            {
                            verify.state = RXSER;
                            }
                  break;
              case RXSER:

                  wdt_reset();
                  /*
                  if (!tmr1)   // timeout if no response
                                 {
                                 verify.state = REQ;                   // retry until response
                                 verify.retry = 1;                     // with no power on/off delay
                                 soft_flush();
                                 }
                                 */
                  while (verify.retry > 0)
                      {
                       retstat = quick_status();
                       wdt_reset();
                       if (retstat != INVALID)
                          {
                          if (retstat == verify.mode)
                              {
                                AC_status = retstat;       // only change status when we get the desired mode back
                                // verify.state = IDLE;    // only go to idle when we get a proper response
                                // tmr2 = TMRDOG;          // restore  the watchdog monitor
                                verify.retry = 0;
                              }
                          }
                          else   // keep looping until we get a response that's valid
                             {
                              Serial.println("Retry");
                             (verify.retry--);
                             }
                      }
                  wdt_reset();
                  verify.state = IDLE;    // go to idle when we get a proper response or time out
                  tmr2 = TMRDOG;          // restore  the watchdog monitor
                  break;
               default:
                  wdt_reset();
                  break;
               }



}
