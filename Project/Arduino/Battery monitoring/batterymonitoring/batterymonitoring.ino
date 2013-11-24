int analogPin = A0;     // potentiometer wiper (middle terminal) connected to analog pin 3
                       // outside leads to ground and +5V
int val = 0;           // variable to store the value read
int bat = 0; //battery percentage
void setup()
{
  Serial.begin(9600);          //  setup serial
}

void lowbat_alert(int x)
{
  if (x <= 20) // if battery less than 20%
  {
    Serial.println("Battery is Low!");
  }
  
}
void loop()
{
  val = analogRead(analogPin);    // read the input pin
  // 6V = 420
  //8.5 = 470
  // 50 / 100%
  bat = (val - 420)*2;
//  Serial.print(bat);             // debug value
//  Serial.println("%");             // debug value
//  delay(500);
  lowbat_alert(bat);

}
