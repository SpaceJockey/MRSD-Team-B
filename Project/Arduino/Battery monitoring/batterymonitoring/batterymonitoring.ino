int analogPin = A0;    //pin bor battery monitoring
int bat_led = 12 ;     //pin for led indicator for battery

int val = 0; // variable to store the value read
  // battery 6V  , val = 420
  // battery 8.5V, val = 470
  // battery 7.5V, val = 455

int bat = 0; //battery percentage

void setup()
{
  Serial.begin(9600);          //  setup serial
  pinMode(bat_led, OUTPUT);
}

void lowbat_alert(int x)
{
  if (x <= 455) // if battery less than 7.5V
  {
    Serial.println("Battery is Low!");
    digitalWrite(bat_led, HIGH);
  }
  else
  {
    digitalWrite(bat_led, LOW);
  }

}
void loop()
{
  val = analogRead(analogPin);    // read the input pin
  lowbat_alert(val);


}

