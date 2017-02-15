

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
#include <ctime>


//for reading and writing to csv
#include <fstream>

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
//First node is RPI itself
const uint8_t pipes[][6] = {"Toch1","Toch2","Toch3","Toch4"};
const int PipesLength=4;


char c='s';
bool ExitTempLoop=false;
	
void Exit_loop_with_Thread(){
	//catching ESC press it will be running asynchoniously
	while(c!=27){
		c=fgetc_unlocked(stdin);
		std::cout<<c<<"\n";
	}
	std::cout<<"ESC pressed\n";
	return;
}


int main(int argc, char** argv){
	float RequestedTemperatures[PipesLength-1]={150,150,150};
	float RequestedTemperature=150;
	int Exit=0; //for exiting the program
	// Setup and configure rf radio
	radio.begin();
	radio.setPALevel(RF24_PA_LOW);

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);
	radio.setChannel(108);
	// Dump the configuration of the rf unit for debugging
	

     	
	unsigned long started_waiting_at=0;
	bool ok;bool timeout;
	float Temperature=0;
	unsigned long got_time;
	int Pipe=0;
	long WrittenRows=0;
	long MaxRows=20000;


	//open read and write pipes
	radio.openWritingPipe(pipes[1]); //this time we will write to this arduino, change address for other arduinos
	radio.openReadingPipe(1,pipes[0]); //read from first adress, it will be RPIs address

	radio.printDetails();

	c='a';
	
	//writing to csv file
	std::ofstream myfile;
	std::string FilenamePrefix="Log";
	std::string Filename;

	// current date/time based on current system
	time_t now = time(0);

	tm *ltm = localtime(&now);

   // print various components of tm structure.
	cout << "Year" << 1900 + ltm->tm_year<<endl;
	cout << "Month: "<< 1 + ltm->tm_mon<< endl;
	cout << "Day: "<<  ltm->tm_mday << endl;
	cout << "Time: "<< ltm->tm_hour << ":";
	cout << ltm->tm_min << ":";
	cout << ltm->tm_sec << endl;
	// concatenate with C++11
	Filename= FilenamePrefix +"_"+std::to_string(1900+ltm->tm_year)+"_"+std::to_string(1+ltm->tm_mon)+"_"+std::to_string(ltm->tm_mday)+
		"_"+std::to_string(ltm->tm_hour)+"_"+std::to_string(ltm->tm_min)+"_"+std::to_string(ltm->tm_sec)+".csv";
	std::cout<<"Filename of log is "<<Filename<<"\n";
	
	myfile.open(Filename);

	//components
	for (int i=1;i<PipesLength;i++){
		myfile<<pipes[i]<<"Date,"<<pipes[i]<<"Time,"<<pipes[i]<<"RequestedTemperature[C],"<<pipes[i]<<"MeasuredTemperature[C],,";
	}
	myfile<<"\n";


	

	
	
	
Pipe = 0;
//RequestedTemperature=150;
while(true){
	
/********* temperature settings ***********/
	Exit=0;//reset exit, if existed
	std::cout<<"\n\nSet the requested temperature, or value<0 to exit:\n";
	std::cout<<"We have "<<PipesLength-1<<" devices, set temperature for each of them\n";
	for (int i=1;i<PipesLength;i++){
		std::cout<<"Input Temperature for device "<<pipes[i]<<"(Previous="<<RequestedTemperatures[i-1]<<"):";
		
		std::cin>>RequestedTemperatures[i-1];
		if (RequestedTemperatures[i-1]<0.0){//we stop for cycle, and then stop program
			Exit=1;
			break; 
		}
	}
	if (Exit==1) break;
	std::cout<<"Press ESC to set new temperature\n";
/***********************************/
	c='a';
	std::thread t1(Exit_loop_with_Thread);

 
	radio.startListening();
	
	
	ExitTempLoop=0;
	// forever loop

	while (!ExitTempLoop)
	
	{		
		if (WrittenRows>MaxRows){//close this file and open next file for output
			myfile.close();
			
			now = time(0);
			ltm = localtime(&now);

			// concatenate with C++11
			Filename= FilenamePrefix +"_"+std::to_string(1900+ltm->tm_year)+"_"+std::to_string(1+ltm->tm_mon)+"_"+std::to_string(ltm->tm_mday)+
				"_"+std::to_string(ltm->tm_hour)+"_"+std::to_string(ltm->tm_min)+"_"+std::to_string(ltm->tm_sec)+".csv";
			std::cout<<"Filename of log is "<<Filename<<"\n";
			
			myfile.open(Filename);

			//components
			for (int i=1;i<PipesLength;i++){
				myfile<<pipes[i]<<"Date,"<<pipes[i]<<"Time,"<<pipes[i]<<"RequestedTemperature[C],"<<pipes[i]<<"MeasuredTemperature[C],,";
			}
			myfile<<"\n";
			
			WrittenRows=0;//reset counter
			
		}
		WrittenRows++;
		radio.stopListening();
		sleep(1);
		if(Pipe<PipesLength-1)Pipe++;
			else {
				Pipe=1; //new cycle
				myfile<<"\n"; //new beginning in log file
			};
			RequestedTemperature=(float)RequestedTemperatures[Pipe-1];
			
			radio.openWritingPipe(pipes[Pipe]);
			

			// First, stop listening so we can talk.
			

			// ask for temperatures  This will block until complete
			std::cout<<"\n--------------------------------------------\n";
			std::cout<<"Asking item "<<pipes[Pipe]<<"\n";
			printf("Now sending requested temperature...\n");
			std::cout<<"sending Requested temperature="<<RequestedTemperature<<"\n";
			sleep(1);
			ok = radio.write( &RequestedTemperature, sizeof(float) );
			started_waiting_at = millis();
			if (!ok){
				printf("failed Sending.\n");
			}
			// Now, continue listening
			radio.startListening();

			// Wait here until we get a response, or timeout (1s)

			timeout = false;
			while ( ! radio.available() && ! timeout ) {
				if (millis() - started_waiting_at > 2000 )
					timeout = true;
			}

				//reask device for temperature if first request have not passed
			if (timeout){
				radio.stopListening();
				sleep(1);
				std::cout<<"\n--------------------------------------------\n";
				std::cout<<"ReAsking item "<<pipes[Pipe]<<"\n";
				printf("Now sending...\n");
			

				ok = radio.write( &RequestedTemperature, sizeof(float) );
				started_waiting_at = millis();
				if (!ok){
					printf("failed Sending.\n");
				}
				// Now, continue listening
				radio.startListening();

				// Wait here until we get a response, or timeout (1s)

				timeout = false;
				while ( ! radio.available() && ! timeout ) {
					if (millis() - started_waiting_at > 2000 )
						timeout = true;
				}
			}

			// Describe the results
			if ( timeout )
			{
				printf("Failed, response timed out.\n");
				
				//Logging information
				now=time(0);
				ltm = localtime(&now);

				myfile<<1900+ltm->tm_year<<1+ltm->tm_mon<<ltm->tm_mday<<","<<ltm->tm_hour<<
					":"<<ltm->tm_min<<":"<<ltm->tm_sec<<","<<RequestedTemperature<<","<<"-1"<<",,";
				
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
				//logging information
				now=time(0);
				ltm = localtime(&now);

				myfile<<1900+ltm->tm_year<<1+ltm->tm_mon<<ltm->tm_mday<<","<<ltm->tm_hour<<
					":"<<ltm->tm_min<<":"<<ltm->tm_sec<<","<<RequestedTemperature<<","<<Temperature<<",,";
				//myfile<<ctime(&now)<<","<<RequestedTemperature<<","<<Temperature<<",,";
				
				
			}
			

			if (c==27) {ExitTempLoop=true; t1.join();}
			
			sleep(3);		


		
		

	} // temperature requests loop; is exited if ESC is pressed
}//forever loop
		
	myfile.close();	

	return 0;
}

