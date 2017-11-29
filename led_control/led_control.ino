char inByte[3];
int var;
int count= 0;
int dutycycle;
// purple - pin 10
// blue - pin 13

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
 // digitalWrite(13,LOW);
  pinMode(10,OUTPUT);
 analogWrite(10,0);
  analogWrite(13,0);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0)
  {
    
  inByte[count] = Serial.read();
   // dutycycle[count] = (int)inByte;
   //digitalWrite(13,HIGH- digitalRead(13));
  count++;

  }
  else
  {
    if(count == 1)
    {
    inByte[1] = '\0';
    }
    if(count == 2)
    {
    inByte[2] = '\0';
    }
    dutycycle = atoi(inByte);
    count = 0;

  }
analogWrite(10,dutycycle);
  analogWrite(13,dutycycle);
  }


