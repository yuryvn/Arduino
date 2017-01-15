// Include the libraries we need for temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

/*
Now Radio module libraries
*/

#include <SPI.h>
#include "RF24.h"
#include <printf.h>

//librariy for PID
#include <PID_v1.h>

//Transistor control

int TransistorPin=4;




//-----------------variables for temperature sensor---------------
int OneWireBus=34;
int numberOfDevices;
float RequestedTemperature=0.0;
float RequestedTemperatureOLD=0.0;
float Eps=1.0e-6;
    
float Temperature = 0;


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




//---------------------------Variables for PID---------------------------
//Define Variables we'll be connecting to
double Setpoint, Input, Output;


//Specify the links and initial tuning parameters
double Kp=5., Ki=1.0, Kd=0.8;
//Conservative tuning parameters
double consKp=0.01, consKi=0.001, consKd=10.0;
double gap=8.;//if difference between target and actual is lower than this, change to conservative PID params
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float WindowSize = 100;//range of pid output from 0 to this, it will be ratio of PIDSizeDuration when PID wants relay ON
float PIDStepDuration=5000;//ms
unsigned long windowStartTime;
unsigned long currentTime,TimeDif;
float Ratio;
int AlreadyRequested=0;
//-----------------------------------------------------------------------




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

//------------------------------PID INITIALIZATION-------------------------------
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 26;
  AlreadyRequested=0;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
//----------------------------------------------------------------------------------


}//setup




void loop() {
  if (millis()-windowStartTime>PIDStepDuration/2&&AlreadyRequested==0){
    Temperature = GetTemp(sensors);    // Take reading in the middle of PIDtimestep.  This will block until complete
    AlreadyRequested=1;
  }
  RequestedTemperatureOLD=RequestedTemperature; //saving previous request
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

    //now change set point for pid if new requested temperature is different to previous
    if(abs(RequestedTemperatureOLD-RequestedTemperature)<Eps) Setpoint=RequestedTemperature;
    
    Serial.print(F("Requested Temperature="));Serial.println(RequestedTemperature);
    Serial.print(F("Actual Temperature from Sensor="));Serial.println(Temperature);
    Serial.print(F("Sent at time="));Serial.println(start_time); 
    Serial.println(F("---------------------------------------------"));  
   }
   
//-------------------------PID CALCULATION and switching relay------------------------------

  currentTime=millis();
  TimeDif=currentTime-windowStartTime;
  if (TimeDif<PIDStepDuration){
    if ((float)TimeDif/PIDStepDuration<Ratio)
      digitalWrite(TransistorPin, HIGH);
    else
      digitalWrite(TransistorPin, LOW);
  }
  else{
    //switching pid parameters if we are close to the target
    //switching back not at the same gap as we do not want to swtich back and forth when at the gap
    //exactly
    if (abs(Temperature-RequestedTemperature)<gap)
      myPID.SetTunings(consKp, consKi, consKd);
     else {
      if (abs(Temperature-RequestedTemperature)>1.5*gap) myPID.SetTunings(Kp, Ki, Kd);
     }
    AlreadyRequested=0;//resetting temperature request counter
    windowStartTime+=PIDStepDuration;
    Input = Temperature;
    myPID.Compute();
    Ratio=Output/WindowSize;//Ratio if end of step matches exactly when last temperatrue was measured
    //need to modify Ratio as most likely during starting part of pid timestep the relaypin is still LOW
    //due to previous relay pin timestep setting it so
    //our pidtimestep is ideally divided into (HIGH__timeratio__LOW)
    //and actuall will most likely be (LOW__startofnewpidtimestep__HIGH__timeration__LOW)
    //we need to adjust timeratio so LOW time at the start + LOW time at the end of actual pidtimestep
    //would be equal to total LOW time for ideal pidtimestep
    //PIDStepDuration*Ratio is duration that PID wants to HIGH time
    //TimeDif-PIDStepDuration  is duration of LOW transfered from previous pidtimestep to current one
    //PIDStepDuration*Ratio+(TimeDif-PIDStepDuration) is when we actually want to swith relay to LOW again
    Ratio=(PIDStepDuration*Ratio+(TimeDif-PIDStepDuration))/PIDStepDuration;
    TimeDif=currentTime-windowStartTime;
    if ((float)TimeDif/PIDStepDuration<Ratio)
      digitalWrite(TransistorPin, HIGH);
    else
      digitalWrite(TransistorPin, LOW);
      
    }
    if (millis()%1000>997){//trying to output each second
     Serial.print(F("PIDOutput="));Serial.println(Output);
     Serial.print(F("currentTime="));Serial.println(currentTime);
     Serial.print(F("currentTime - windowStartTime="));Serial.println(currentTime - windowStartTime);
     Serial.print(F("windowStartTime="));Serial.println(windowStartTime);
     }
  //----------------------------------------------------------------

} // Loop





