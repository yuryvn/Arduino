
int ledPin=12;
int val;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(A1,INPUT);
  digitalWrite(ledPin,LOW);

}

void loop() {
  val = analogRead(A1);    // read the value from the sensor
  Serial.println(val);
  digitalWrite(ledPin, HIGH);  // turn the ledPin on
  delay(val);                  // stop the program for some time
  digitalWrite(ledPin, LOW);   // turn the ledPin off
  delay(val);                  // stop the program for some time

}
