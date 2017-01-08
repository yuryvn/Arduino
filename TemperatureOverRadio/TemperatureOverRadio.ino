// Include the libraries we need for temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

/*
Now Radio module libraries
*/

#include <SPI.h>
#include "RF24.h"
#include <printf.h>


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


byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = 0;
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
  delay(3000);
  
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
  Serial.println(F("RF24/examples/GettingStarted"));
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
//  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.printDetails();
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();

  //-------------------------------------------------------------------------------------
}//setup




void loop() {
  
  
/****************** Ping Out Role ***************************/  
if (role == 1)  {
    
    radio.stopListening();                                    // First, stop listening so we can talk.
    
    
    Serial.println(F("Now sending"));

    float Temperature = GetTemp(sensors);    // Take the time, and send it.  This will block until complete
    unsigned long start_time=micros();
    int transmitteddata=100;
     if (!radio.write( &Temperature, sizeof(Temperature) )){
       Serial.println(F("failed"));
     }
        
    radio.startListening();                                    // Now, continue listening
    
    unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
    boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not
    
    while ( ! radio.available() ){                             // While nothing is received
      if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
          timeout = true;
          break;
      }      
    }
        
    if ( timeout ){                                             // Describe the results
        Serial.println(F("Failed, response timed out."));
    }else{
        float RequestedTemperature;                                 // Grab the response, compare, and send to debugging spew
        radio.read( &RequestedTemperature, sizeof(float) );
        unsigned long end_time = micros();
        
        // Spew it
        Serial.print(F("Sent measured Temperature="));
        Serial.print(Temperature);
        Serial.print(F(", Got response:RequestedTemperature="));
        Serial.print(RequestedTemperature);
        Serial.print(F(", Round-trip delay "));
        Serial.print(end_time-start_time);
        Serial.println(F(" microseconds"));
    }

    // Try again 5s later
    delay(5000);
  }



/****************** Pong Back Role ***************************/

  if ( role == 0 )
  {
    unsigned long got_time;
    
    if( radio.available()){
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &got_time, sizeof(unsigned long) );             // Get the payload
      }
     
      radio.stopListening();                                        // First, stop listening so we can talk   
      radio.write( &got_time, sizeof(unsigned long) );              // Send the final one back.      
      radio.startListening();                                       // Now, resume listening so we catch the next packets.     
      Serial.print(F("Sent response "));
      Serial.println(got_time);  
   }
 }




/****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){      
      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    
   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       
    }
  }


} // Loop

