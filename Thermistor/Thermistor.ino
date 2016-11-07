//  File:  ThermistorTemperature.pde
//
//  Use a voltage divider to indicate electrical resistance of a thermistor.
//  Convert the resistance to temperature.
// -- setup() is executed once when sketch is downloaded or Arduino is reset
void setup() {
Serial.begin(9600);        // open serial port and set data rate to 9600 bps
Serial.println("Thermistor temperature measurement:");
Serial.println("\n   Vo     Rt     T (C)");
}
// -- loop() is repeated indefinitely
void loop() {
int   ThermistorPin=A1;   //  Analog input pin for thermistor voltage
int   Vo;                  //  Integer value of voltage reading
float R = 10000.0;          //  Fixed resistance in the voltage divider
float logRt,Rt,T;
float c1 =  1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
Vo = analogRead(ThermistorPin);
Rt = R*( 1023.0 / (float)Vo - 1.0 );
logRt = log(Rt);
T = ( 1.0 / (c1 + c2*logRt + c3*logRt*logRt*logRt ) ) - 273.15;
Serial.print("  ");  Serial.print(Vo);
Serial.print("  ");  Serial.print(Rt);
Serial.print("  ");  Serial.println(T);
delay(200);
}
