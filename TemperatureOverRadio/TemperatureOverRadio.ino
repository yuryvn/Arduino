// Include the libraries we need for temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

/*
Now Radio module libraries
*/

#include <SPI.h>
#include "RF24.h"
#include <printf.h>


//Transistor control

int TransistorPin=4;




//-----------------variables for temperature sensor---------------
int OneWireBus=34;
int numberOfDevices;
//double CurrentTemperature=0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(OneWireBus);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
//---------------------------------------------------------------


//--------------------------variables for radio module-----------------
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,53); //9 is CE pin 53 is CSN


byte addresses[][6] = {"Toch1","Toch2"};


//-------------------------------------------------------------------------


//=======================FUNCTIONS=======================
/*
 * Function get and print the temperature
 */
float GetTemp(DallasTemperature &sens)
{ 
  float CurrentTemperature=0;
  int numberOfDevices=0;
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  
  numberOfDevices = sens.getDeviceCount();

  Serial.print("Requesting temperatures...");
  sens.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  CurrentTemperature=sens.getTempCByIndex(0);
  Serial.println(CurrentTemperature);

  return CurrentTemperature;

}
//======================================================================================


//SETUP
void setup(void)
{
//------------------transistor setup------------------------
  pinMode(TransistorPin, OUTPUT);
  digitalWrite(TransistorPin,LOW);
//----------------------------------------------------------
  
  //--------------------------Temperature sensor initialization----------------------------
  // start serial port
  Serial.begin(115200);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();

  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");
  //----------------------------------------------------------------------------------------


  //------------------------------------radio module initialization------------------------
  printf_begin();
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
//  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.printDetails();
  
  // Open a writing and reading pipe on each radio, with opposite addresses

    radio.openWritingPipe(addresses[0]); //we will write to first adress, the first address will be RPIs
    radio.openReadingPipe(1,addresses[1]);//this arduino address
 
  
  // Start the radio listening for data
  radio.startListening();

  //-------------------------------------------------------------------------------------
Serial.println("Waiting for input");
}//setup




void loop() {

   digitalWrite(TransistorPin,LOW);
   delay(2000);
   digitalWrite(TransistorPin,HIGH);
  
      unsigned long got_time;
      float RequestedTemperature=0.0;
    
  float Temperature = GetTemp(sensors);    // Take reading, and send it.  This will block until complete
    
    if( radio.available()){
                                                                    // If receive radio signal
      while (radio.available()) {                                   // 
        radio.read( &RequestedTemperature, sizeof(float) );        //read requested temperature  
      }

     
      radio.stopListening();                                        // First, stop listening so we can talk
      //find out actual temperature at sensor and send it
          Serial.println(F("---------------------------------------------"));
    
    unsigned long start_time=micros();
   // delay(1500);
      if (!radio.write( &Temperature, sizeof(Temperature) )){  //send actual temperature over radio
       Serial.println(F("failed"));
     }
    radio.startListening();

    Serial.print(F("Requested Temperature="));Serial.println(RequestedTemperature);
    Serial.print(F("Actual Temperature from Sensor="));Serial.println(Temperature);
    Serial.print(F("Sent at time="));Serial.println(start_time); 
    Serial.println(F("---------------------------------------------"));  
   }

} // Loop

