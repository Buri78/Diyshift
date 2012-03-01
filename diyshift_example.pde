

/* Diyshift :  Preston Fall 
http://www.diyshift.com
Diyshift last revision 2/16/12

The purpose of the following code is to facilitate electronic shifting of a multi speed bicycle using an Arduino compatible microcontroller.

THIS SOFTWARE IS PROVIDED  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ 
or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.

 ******************hardware***********************
 Controller: Dorkboard powered via Sparkfun 5v DC to DC boost board for VCC
 Battery: 3.7v lithium polymer 2000 mAh
 Custom "shield" houses servo power mosfet (logic level), pulldown resistors for shift buttons, trim mode switch and servo header
 Adaboot bootloader on atmega 328 (sleep functions have not tested with the stock arduino bootloader! use them at your own risk)
 Servo: HiTec hs-225mg metal gear mini servo
 Rear Derailleur: Sram X5 (modified with E-railleur mod kit)
 10 speed cassette & chain
 Front derailleur and related code in progress
 
 *****************hardware setup*****************
 servo control attached to pin 9 
 Servo's main power is switched on during shifts via MOSFET on pin 7 for several seconds then back off to keep servo from draining battery 
 in sleep mode. Sleep mode will consume approx. .62 mA
 upshift button is attached to digital pin 2 pulled low with 10k resistor (interrupt 0) 
 downshift button is attached to digital pin 3 pulled low with 10k resistor  (interrupt 1) 
 next revision will only use int0 for all shift buttons and shift pins will use pullup resistors
 
 all lines marked with "SPEED VARIABLE" are cassette dependent. All code below assumes you are using a 10 cog shimano or SRAM cassette. 
 
 *****************begin code*************************/
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Servo.h>

//********************setup for ADC shutdown*****************
// Courtesy Martin Nawrath nawrath@khm.de - Watchdog sleep example
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))   
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//********************end ADC shutdown************************


//***********************begin interrupt & sleep functions*********
void dwnShiftInterrupt(void){
  // interrupt to wake from sleep, may consider adding shift code here eventually
  
}
void upShiftInterrupt(void)
{
  //interrupt to wake from sleep, may consider adding shift code here eventually
}
void enterSleep(void)
{

  //Setup pin2 & 3 as interrupts and attach handlers.
  attachInterrupt(0, upShiftInterrupt, RISING); //trigger interrupt 0 if pin 2 is HIGH
  attachInterrupt(1, dwnShiftInterrupt, RISING); //trigger interrupt 1 if pin 3 is HIGH
  delay(100);
  cbi(ADCSRA,ADEN);                    // switch ADC OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //enter lowest power sleep mode
  sleep_enable();
  sleep_mode();  
  sleep_disable(); 
  sbi(ADCSRA,ADEN); //switch ADC back on
  detachInterrupt(0); //detach the interrupt to stop it from continuously firing while the interrupt pin is high
  detachInterrupt(1); //"                                                                                       "
}
//***************************end interrupt and sleep functions********************************

//**************************begin variable declaration****************************************
Servo myservo;
const int upShift = 2;        //attach upshift button to pin 2 (int 0)
byte buttonPushCounterUp = 0; //counter for number of upshift button presses
int buttonStateUp = 0;        //current state of upshift button
int lastButtonStateUp = 0;    //previous state of upshift button
const int dwnShift = 3;       //attach downshift button to pin 3 (int 1)
byte buttonPushCounterDwn = 0; //counter for number of downshift button presses
int buttonStateDwn = 0;     //current state of downshift button
int lastButtonStateDwn = 0; //previous state of downshift button
int lastTrimUp = 0;
int lastTrimDwn = 0;
int buttonTrimUp = 0;
int buttonTrimDwn = 0;
int buttonTrimCounterUp = 0;
int buttonTrimCounterDwn = 0;
int lastButtonTrimUp = 0;
int lastButtonTrimDwn = 0;
int gear = 0;  // stores the gear that bike is in
const int modeSwitch = 8; //attach trim adjust mode slide switch to pin 8
byte modeSwitchState = 0; //stores state of calibration mode switch
const int statusLed = 13;  //attach onboard status led to pin 13
const int servoPower = 7; //attach servo power mosfet to pin 7
int servoPowerState = LOW;             // State used to set power to servo motor
long previousMillis = 0;               //timer for servoPower delay
long interval = 1000;           // interval at which to keep servo powered on during shift(milliseconds)
long buttonHoldTime = 0;        //stores lengh of time button is held
int trimVal = 0;
//***************************************end variable declaration***********************************************

void setup(){
  //Serial.begin(9600);  
  pinMode(modeSwitch, INPUT);
  digitalWrite(modeSwitch, HIGH); //activate internal 20 k pullup resistor
  pinMode(upShift, INPUT);
  pinMode(dwnShift, INPUT);    
  pinMode(servoPower, OUTPUT);
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW); // light status led for 250 ms (dorkboard uses LOW to light onboard LED, arduino use HIGH)
  myservo.attach(9);  //attatch rear shift servo to pin 9  
  delay(250);
  digitalWrite(statusLed, HIGH); //turn onboard LED off  
}

void loop() {
  int gear[] = {0, 0, 20, 30, 40, 50, 62, 74, 85, 99, 120}; //values to write to servo @ 0 trim  //****SPEED VARIABLE*****    
  digitalWrite(statusLed, HIGH); //this turns red LED off (dorkboard only use opposite "LOW" for arduino)
  modeSwitchState = digitalRead (modeSwitch); //read slideswitch state to see if we are in trim mode
  
  
  //***********************************begin set and store trim value *****************************************************
  /* trim mode automatically shifts to 4th gear, keep pressing the downshift button until it begins to shift to third gear then press upshift button 2-3 times
     or until chattering stops, the chain should now be centered on fourth gear, slide mode switch to return to normal shift mode. */ 
  
  if (modeSwitchState == LOW){ //if modeSwitch is pulled LOW set trim values and store them to EEPROM
    digitalWrite(statusLed, LOW); //this turns red LED on (dorkboard only use opposite "HIGH" for arduino)
    buttonTrimUp = digitalRead(upShift); //reads upshift button
    delay(20); //crude debounce button
    buttonTrimDwn = digitalRead(dwnShift);//reads downshift button
    delay(20);//crude debounce button
    digitalWrite(servoPower, HIGH); //power up servo and keep it on for trim adjustment
    if (buttonTrimUp != lastButtonTrimUp) {
      //if state has changed increment the counter
      if (buttonTrimUp == HIGH && buttonTrimCounterUp != 30){ //limit upper trim amount to 30
        buttonTrimCounterUp ++;  //increment trim count
        if(buttonTrimCounterUp >= 29){ //number of trim adjustments allowed including 0
          buttonTrimCounterUp = 30; // button is pressed more than 29 times stay in keep counter at 30
        }      

      }
      lastButtonTrimUp = buttonTrimUp;

    }
    if (buttonTrimDwn != lastButtonTrimDwn) {
      //if state has changed increment the counter
      if (buttonTrimDwn == HIGH){        
        if(buttonPushCounterDwn < 1){ //don't let shift counter go negative
          buttonPushCounterDwn = 1;
        }

        buttonTrimCounterUp --; // downshift once for every button press
        buttonTrimCounterDwn ++; // increment downshift counter      
        if(buttonTrimCounterUp < 1 || buttonTrimCounterUp >30 ){  // don't let shift counter go negative 
          buttonTrimCounterUp = 1;
        }
        if(buttonTrimCounterDwn >= 28){ //this is the maximum amount of downshifts allowed
          buttonTrimCounterDwn = 28; //this will always be 2 less than the number number of degrees of trim you have allow
        }      

        delay(100); 


      }
      //save the current state as the last state
      lastButtonTrimDwn = buttonTrimDwn;    
    }
    EEPROM.write(0, buttonTrimCounterUp); // write trim value to EEPROM address 0 
    trimVal = buttonTrimCounterUp;
    myservo.write(gear[4] + trimVal); //use 4th gear as adjustment gear since it is close to middle
    //Serial.println(buttonTrimCounterUp, DEC);
    //Serial.print("Trim value:  "); 
    //Serial.println(trimVal, DEC);
    //Serial.println("trim adjust mode");
  }
  else{
    //***********************************end set and store trim value********************************************************* 
    
    //***************************read shift inputs and setup shift point data array**********************************************
    
    unsigned long currentMillis = millis(); //setup timer for servo power    
    /* set array to store values for shift points from 1st to 10th (the first 0 is a required dummy value). initial gear spacing values in the array are determined by 
    loading the shift calibrator sketch with a 10k potentiometer attached to analog pin 0, servo on pin 9. Servo angles are recorded via serial monitor. */
    
    
    buttonStateUp = digitalRead(upShift); //reads upshift button
    delay(20); //crude debounce button
    buttonStateDwn = digitalRead(dwnShift);//reads downshift button
    delay(20);//crude debounce button
    //Serial.print("Trim value:  ");  
    //Serial.println(trimVal, DEC);

    //********************************* begin dump cassette code ********************************************
    /* the following code allows you to shift all the way to either the highest or lowest gear by depressing the corresponding button for 700-1200 ms*/
    if (buttonStateDwn != lastButtonStateDwn) {    
      if (buttonStateDwn == HIGH) {  
        buttonHoldTime = millis();    

      }   
    }
    if((buttonStateDwn == HIGH) && (lastButtonStateDwn == HIGH)){
      if(buttonStateDwn == HIGH && (millis() - buttonHoldTime) >700){  //if held for 700 ms shift to first gear
        myservo.write(gear[1] + trimVal); 
        buttonPushCounterUp = 1; 
      }
    }

    if (buttonStateUp != lastButtonStateUp) {    
      if (buttonStateUp == HIGH) {  
        buttonHoldTime = millis();    

      }   
    }
    if((buttonStateUp == HIGH) && (lastButtonStateUp == HIGH)){

      if(buttonStateUp == HIGH && (millis() - buttonHoldTime) >1200){  //if held for 1200 ms shift to highest gear
        myservo.write(gear[10] + trimVal); 
        buttonPushCounterUp = 10;                                                                                            //****SPEED VARIABLE*****
      }
    }
    //**************************************end cassette dump code**************************************************************

    //*****************************************begin upshift********************************************************************
    
    if (buttonStateUp != lastButtonStateUp) {
      //if state has changed increment the counter
      if (buttonStateUp == HIGH && buttonPushCounterUp != 10){ //if already in 10th do not allow any more upshifts              //****SPEED VARIABLE*****
        previousMillis = currentMillis; //mark time servo was powered up
        digitalWrite(servoPower, HIGH); //spool up servo
        servoPowerState = HIGH; //set servo power state
        buttonPushCounterUp ++;  //increment shift count
        if(buttonPushCounterUp >= 10){ //number of upshifts allowed including 0                                                //****SPEED VARIABLE*****
          buttonPushCounterUp = 10; // button is pressed more than 10 times stay in 10th gear                                    //****SPEED VARIABLE*****
        }
        myservo.write(gear[buttonPushCounterUp] + trimVal + 10); //write angle to servo + trimValue (+ 10 is a slight overshift feature to improve shifting)   
        delay(130); //give the servo time to reach the selected gear
        myservo.write(gear[buttonPushCounterUp ]+ trimVal ); //write final angle to servo, this centers the chain on selected cog
        Serial.print("Gear:  ");
        Serial.println(buttonPushCounterUp, DEC);
        Serial.print("Trim value:  ");  
        Serial.println(trimVal, DEC);      
      }
      lastButtonStateUp = buttonStateUp;
    }

    //************************************************************begin downshift*******************************************************
    if (buttonStateDwn != lastButtonStateDwn) {
      //if state has changed increment the counter
      if (buttonStateDwn == HIGH){
        previousMillis = currentMillis;  //mark time servo was powered up
        digitalWrite(servoPower, HIGH); //spool up servo
        servoPowerState = HIGH; //set servo power state 
        if(buttonPushCounterDwn < 1){
          buttonPushCounterDwn = 1;
        }
        if(buttonPushCounterUp == 10){  //if counter is 10 then add one to workaround shift skipping from 10 to 8                      //****SPEED VARIABLE*****
          buttonPushCounterUp  +1; 
        }
        buttonPushCounterUp --; // downshift once for every button press
        buttonPushCounterDwn ++; // increment downshift counter      
        if(buttonPushCounterUp < 1 || buttonPushCounterUp >10 ){  // don't let shift counter go negative                            //****SPEED VARIABLE*****
          buttonPushCounterUp = 1;
        }
        if(buttonPushCounterDwn >= 8){ //this is the maximum amount of downshifts allowed                                           //****SPEED VARIABLE*****
          buttonPushCounterDwn = 8; //this will always be 2 less than the # of gears you have                                       //****SPEED VARIABLE*****
        }      
        myservo.write(gear[buttonPushCounterUp] + trimVal - 15);//write angle to servo + trimValue (-15 is a slight overshift feature to improve shifting)  
        delay(130);  //give the servo time to reach the selected gear
        myservo.write(gear[buttonPushCounterUp] + trimVal); //write final angle to servo, this centers the chain on selected cog
        Serial.print("Gear:  ");
        Serial.println(buttonPushCounterUp, DEC);
        Serial.print("Trim value:  ");  //22 is a good value to have here
        Serial.println(trimVal, DEC);
      }
      //save the current state as the last state
      lastButtonStateDwn = buttonStateDwn;    
    }

    //********************************** delay servo power down and sleep cycle****************************
    /* the following code sets the delay time for the servo power off and sleep mode by using the millis function */
    if(currentMillis - previousMillis > interval){  
      // save the last time you powered the servo 
      previousMillis = currentMillis; 
      digitalWrite(servoPower, LOW);
      servoPowerState = LOW; // cut power to servo
      delay(50);
      enterSleep(); //enter low power sleep mode 
    }

  }
}





