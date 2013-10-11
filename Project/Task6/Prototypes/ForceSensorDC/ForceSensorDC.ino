// for the force sensor to control the DC motor
// MRSD PROJECT
// Songjie Zhong

int fsrPin=0;   //set A0 for force sensor pin
double fsrReading;

#define EN1   10 	//Enable pin for motor 1
#define L11   11 	// Logic 1 for motor 1
#define L12   12 	// Logic 2 for motor 2


// SETUP
void setup() {
 
  // define pins
  pinMode(A0,INPUT);    // THIS IS FOR READING
  pinMode (EN1, OUTPUT);
  pinMode (L11, OUTPUT);
  pinMode (L12, OUTPUT); 
  
  Serial.begin (9600); // open Serial
}

// LOOP
void loop(){
 Serial.print("Analog reading =");
 fsrReading=analogRead(fsrPin);
 
 Serial.println(fsrReading);
 
 if ( fsrReading >= 0 && fsrReading<=255.75) {
      digitalWrite(L11,HIGH);
      digitalWrite(L12,LOW);
      analogWrite(EN1, -fsrReading+255.75);
      //counterclockwise
    }
    else {
      digitalWrite(L11,LOW);
      digitalWrite(L12, HIGH);
      analogWrite(EN1, -255.75+fsrReading);
      //clockwise
    }
  Serial.println(((360 / 255.75) * fsrReading-360)/360);  
  delay(100);
}

