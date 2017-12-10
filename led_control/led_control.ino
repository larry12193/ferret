int dutycycle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(12,OUTPUT);
  pinMode(10,OUTPUT);
  analogWrite(10,255);
  analogWrite(11,255);

}

void loop() {
  while( Serial.available() > 0 ) {

    // Look for valid integer
    dutycycle = Serial.parseInt();

    // Look for newline
    if( Serial.read() == '\n' ) {
      Serial.print("Setting duty:");
      Serial.println(dutycycle);
      analogWrite(10,dutycycle);
      analogWrite(11,dutycycle);
    }
  }
}


