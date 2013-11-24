
/* Battery Monitoring
Written by: Ardya Dipta Nandaviri (dipta@cmu.edu)
MRSD Team B project, Fall 2013

Program to measure the battery voltage and gives alert when
the battery reaches certain value
*/

#define BATT_MON   A0    //analog pin bor battery monitoring
#define BATT_LED  12     //pin for led indicator for battery
int val = 0; // variable to store the value read
int bat_percent = 0; //battery percentage

/* Battery calibration
  battery 6V  , val = 420
  battery 8.5V, val = 470
  battery 7.5V, val = 455
*/
#define BATT_6V 420
#define BATT_8_5V 470
#define BATT_7_5V 455



void setup()
{
  Serial.begin(9600);          //  setup serial
  pinMode(BATT_LED, OUTPUT);
}

void lowbat_alert(int x)
{
  if (x <= BATT_7_5V) // if battery less than 7.5V
  {
    Serial.println("Battery is Low!");
    digitalWrite(BATT_LED, HIGH);
  }
  else
  {
    digitalWrite(BATT_LED, LOW);
  }

}

void loop()
{
  val = analogRead(bat_mon);    // read the input pin
  lowbat_alert(val);
  bat_percent = (val - BATT_6V)*2; 
  // the range is 420 to 470, hence to get 100% value is (val-420)*100/50

}

