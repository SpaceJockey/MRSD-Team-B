/* Battery Monitoring
Written by: Ardya Dipta Nandaviri (dipta@cmu.edu)
MRSD Team B project, Fall 2013

Program to measure the battery voltage and gives alert when
the battery reaches certain value
*/

//Battery Monitoring LED
#define BATT_PIN A8    //pin for battery monitoring
#define BATT_LED 12    //pin for led indicator for battery

//Battery Calibration Values
#define BATT_6V   1700 //6V = 0%
#define BATT_85V  1870 //8.5V = 100%
int batt_mon;
int batt_pct;



void setup()
{
  Serial.begin(9600);          //  setup serial
  pinMode(BATT_LED, OUTPUT);
  analogReadResolution(12);
  batt_mon = analogRead(BATT_PIN);
}

void loop()
{
	//Update battery voltage warning light
	int batt_read = analogRead(BATT_PIN);
	if(abs(batt_read - batt_mon) < 25) {
		batt_mon = ((7 * batt_mon) + batt_read) / 8; 
	} else {
		batt_mon = ((15 * batt_mon) + batt_read) / 16; 
	}
	batt_pct = map(batt_mon, BATT_6V, BATT_85V, 0, 100); //battery percentage
	if (batt_pct > 100) batt_pct = 100;
	if (batt_pct < 0) batt_pct = 0;
	
	Serial.print(batt_read);
	Serial.print(", ");
	Serial.print(batt_mon);
	Serial.print(", ");
	Serial.println(batt_pct);
	//TODO: skew percentage to match logarithmic batt discharge curve
	
	if (batt_pct <= 40) { //if battery level is less than 40%
		digitalWrite(BATT_LED, HIGH); 
		Serial.println("Battery is Low!");
	}else{
		digitalWrite(BATT_LED, LOW);
	}
	delay(250);
}

