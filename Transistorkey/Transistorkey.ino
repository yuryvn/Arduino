//  File:  ThermistorTemperature.pde
//
//  Use a voltage divider to indicate electrical resistance of a thermistor.
//  Convert the resistance to temperature.
// -- setup() is executed once when sketch is downloaded or Arduino is reset
int Vo,VoOld,veryOld,ThermistorPin;
void setup() {
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
  ThermistorPin=A4;
  VoOld=analogRead(ThermistorPin);
  Vo=VoOld;
Serial.begin(9600);        // open serial port and set data rate to 9600 bps
Serial.println("Transistor check");
}
// -- loop() is repeated indefinitely
void loop() {
   //  Analog input pin for thermistor voltage
Serial.print("ThermistorPin=");
Serial.println(ThermistorPin);
float R = 10000.0;          //  Fixed resistance in the voltage divider
Vo = analogRead(ThermistorPin);
Serial.print("AnalogIn=");  Serial.println(Vo);
delay(200);
if ((Vo<VoOld*0.8||Vo>VoOld*1.2)&&(Vo<veryOld*0.8||Vo>veryOld*1.2)) {digitalWrite(2,HIGH);delay(2000);}
veryOld=VoOld;
VoOld=Vo;
digitalWrite(2,LOW);

}
