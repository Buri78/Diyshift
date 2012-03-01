/* Diyshift Calibrator:  Preston Fall 
 http://www.diyshift.com
 Diyshift last revision 2/16/12
 
 The purpose of the following code is to use the Arduino serial monitor to discover values for the diyshift "gear" array. FIRST MAKE SURE THAT YOU HAVE SET YOUR HIGH AND LOW
 LIMIT SCREWS ON THE DERAILLEUR, FAILURE TO DO SO MAY CAUSE IT TO SHIFT THE DERAILLEUR INTO THE SPOKES! Information on setting the limit screws can be found here: 
 http://www.parktool.com/blog/repair-help/rear-derailler-adjustments-derailleur
 
 Turning the potentiometer will move the E-railleur through its entire range while displaying the servo angle. Once the chain is centered on the desired cog, 
 record the servo angle and the corresponding gear. You may need to repeat this step later if you have certain gears that are not shifting properly.
 
 THIS SOFTWARE IS PROVIDED  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
 BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License. 
 To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ 
 or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 
 ******************hardware***********************
 Arduino board or compatible, make sure your voltage regulator can source ~500 mA
 Servo: HiTec hs-225mg metal gear mini servo
 Rear Derailleur: Sram X5 (modified with E-railleur mod kit)
 Front derailleur and related code in progress
 
 *****************hardware setup*****************
 servo control attached to pin 9 
 10K linear potetiometer connected to analog pin 0
 
 
 
 
 */

#include <Servo.h> 

Servo myservo;  
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(9);  // attach servo to pin 9


} 


void loop() 
{ 

  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val);  // sets the servo position according to the scaled value 
  Serial.println("Servo angle");
  Serial.println(val, DEC);  // prints servo position value 
  delay(15);                           
} 

