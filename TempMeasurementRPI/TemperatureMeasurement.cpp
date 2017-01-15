

/*
 sending required temperature to arduino and receiving actual temperature reading
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <RF24/RF24.h>
#include <stdio.h>
#include <thread>

using namespace std;
//
// Hardware configuration
// Configure the appropriate pins for your connections

/****************** Raspberry Pi ***********************/

// Radio CE Pin, CSN Pin, SPI Speed
// See http://www.airspayce.com/mikem/bcm2835/group__constants.html#ga63c029bd6500167152db4e57736d0939 and the related enumerations for pin information.

// Setup for GPIO 22 CE and CE0 CSN with SPI Speed @ 4Mhz
//RF24 radio(RPI_V2_GPIO_P1_22, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_4MHZ);

// NEW: Setup for RPi B+
RF24 radio(RPI_BPLUS_GPIO_J8_15,RPI_BPLUS_GPIO_J8_24, BCM2835_SPI_SPEED_8MHZ);

// Setup for GPIO 15 CE and CE0 CSN with SPI Speed @ 8Mhz
//RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);

// RPi generic:
// RF24 radio(22,0);

/*** RPi Alternate ***/
//Note: Specify SPI BUS 0 or 1 instead of CS pin number.
// See http://tmrh20.github.io/RF24/RPi.html for more information on usage

//RPi Alternate, with MRAA
//RF24 radio(15,0);

//RPi Alternate, with SPIDEV - Note: Edit RF24/arch/BBB/spi.cpp and  set 'this->device = "/dev/spidev0.0";;' or as listed in /dev
//RF24 radio(22,0);


/****************** Linux (BBB,x86,etc) ***********************/

// See http://tmrh20.github.io/RF24/pages.html for more information on usage
// See http://iotdk.intel.com/docs/master/mraa/ for more information on MRAA
// See https://www.kernel.org/doc/Documentation/spi/spidev for more information on SPIDEV

// Setup for ARM(Linux) devices like BBB using spidev (default is "/dev/spidev1.0" )
//RF24 radio(115,0);

//BBB Alternate, with mraa
// CE pin = (Header P9, Pin 13) = 59 = 13 + 46 
//Note: Specify SPI BUS 0 or 1 instead of CS pin number. 
//RF24 radio(59,0);

/********** User Config *********/
// Assign a unique identifier for this node, 0 or 1
bool radioNumber = 1;

/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"Toch1","Toch2","Toch3"};
const int PipesLength=3;


char c='s';
bool ExitTempLoop=false;
	
void Exit_loop_with_Thread(){
	//catching ESC press
	while(c!=27){
		c=fgetc_unlocked(stdin);
		std::cout<<c<<"\n";
	}
	return;
}


int main(int argc, char** argv){
	float RequestedTemperature=150.0;


  cout << "RF24/examples/GettingStarted/\n";

  // Setup and configure rf radio
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setChannel(108);
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();

     	
	unsigned long started_waiting_at=0;
	bool ok;bool timeout;
	float Temperature=0;
	unsigned long got_time;
	int Pipe=0;
	

	//open read and write pipes
      radio.openWritingPipe(pipes[2]); //this time we will write to this arduino, change address for other arduinos
      radio.openReadingPipe(1,pipes[0]); //read from first adress, it will be RPIs address



	c='a';

while(true){
	std::thread t1(Exit_loop_with_Thread);
/********* temperature settings ***********/

	std::cout<<"Set the requested temperature\n";
	std::cin>>RequestedTemperature;
	std::cout<<"Press ESC to set new temperature\n";
/***********************************/
c='a';
 
	radio.startListening();
	
	
	ExitTempLoop=0;
	// forever loop
	Pipe=0;
	while (!ExitTempLoop)
	
	{		
		radio.stopListening();
		if(Pipe<PipesLength-1)Pipe++;
			else Pipe=1;
			radio.openWritingPipe(pipes[Pipe]);
			

			// First, stop listening so we can talk.
			

			// Take the time, and send it.  This will block until complete
			std::cout<<"\n--------------------------------------------\n";
			std::cout<<"Asking item "<<pipes[Pipe]<<"\n";
			printf("Now sending...\n");
		

			ok = radio.write( &RequestedTemperature, sizeof(float) );
			started_waiting_at = millis();
			if (!ok){
				printf("failed.\n");
			}
			// Now, continue listening
			radio.startListening();

			// Wait here until we get a response, or timeout (2s)

			timeout = false;
			while ( ! radio.available() && ! timeout ) {
				if (millis() - started_waiting_at > 10000 )
					timeout = true;
			}


			// Describe the results
			if ( timeout )
			{
				printf("Failed, response timed out.\n");
			}
			else
			{
				// Grab the response, compare, and send to debugging spew
				Temperature=0;
				radio.read( &Temperature, sizeof(float) );
				got_time=millis();

				// Spew it
	
				printf("Got response %lu, Startedwaiting %lu,round-trip delay: %lu\n",got_time,started_waiting_at,got_time-started_waiting_at);
				std::cout<<"Requested Temperature="<<RequestedTemperature<<"\n";
				std::cout<<"Actual Temperature="<<Temperature<<"\n";
			}
			

			if (c==27) ExitTempLoop=true;
			
			sleep(5);		


		
		

	} // temperature requests loop; is exited if ESC is pressed
}//forever loop
  return 0;
}
