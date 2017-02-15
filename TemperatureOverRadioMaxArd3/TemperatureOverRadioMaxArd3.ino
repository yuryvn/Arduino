#include <SPI.h>
// Include the libraries we need for temperature sensor
#include <PlayingWithFusion_MAX31855_J_correction.h>
#include <PlayingWithFusion_MAX31855_STRUCT_corr.h>
#include <PlayingWithFusion_MAX31855_T_correction.h>
#include <PlayingWithFusion_MAX31855_Wcorr.h>

#define J_type 2
#define T_type 3

/*
Now Radio module libraries
*/

#include "RF24.h"
#include <printf.h>

//librariy for PID
#include <PID_v1.h>

//Transistor control

int TransistorPin=4;




//-----------------variables for temperature sensor---------------
// setup CS pins used for the connection with the sensor
// other connections are controlled by the SPI library)
int8_t CS0_PIN = 3;
PWFusion_MAX31855_TC  thermocouple0(CS0_PIN);
float RequestedTemperature=0.0;
float RequestedTemperatureOLD=0.0;
float Eps=1.0e-6;
    
float Temperature = 0;

//---------------------------------------------------------------


//--------------------------variables for radio module-----------------
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,53); //9 is CE pin 53 is CSN


byte addresses[][6] = {"Toch1","Toch4"};
//-------------------------------------------------------------------------




//---------------------------Variables for PID---------------------------
//Define Variables we'll be connecting to
double Setpoint, Input, Output;


//Specify the links and initial tuning parameters
double Kp=5., Ki=1.0, Kd=0.8;
//Conservative tuning parameters
double consKp=0.5, consKi=0.1000, consKd=2.0;
double gap=8.;//if difference between target and actual is lower than this, change to conservative PID params
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float WindowSize = 10;//range of pid output from 0 to this, it will be ratio of PIDSizeDuration when PID wants relay ON
float PIDStepDuration=5000;//ms
unsigned long windowStartTime;
unsigned long currentTime,TimeDif;
float Ratio;
int AlreadyRequested=0;
int SwitchedToNormal=0,SwitchedToCons=0;
//-----------------------------------------------------------------------




//=======================FUNCTIONS=======================
/*
 * Function get and print the temperature
 */
float GetTemp(PWFusion_MAX31855_TC &sens)
{ 
  double tmp;
  float CurrentTemperature=0;
  static struct var_max31855 TC_CH0 = {0, 0, 0, J_type, 0};
  struct var_max31855 *tc_ptr;
  int numberOfDevices=0;
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus

   Serial.print("Requesting temperatures...");
  // update TC0
  tc_ptr = &TC_CH0;
  sens.MAX31855_update(tc_ptr);        // Update MAX31855 readings
  Serial.println("DONE");
  
  tmp = (double)TC_CH0.ref_jcn_temp * 0.0625;  // convert fixed pt # to double
  Serial.print("Tint = ");                      // print internal temp heading
  if((-100 > tmp) || (150 < tmp)){Serial.println("unknown fault");}
  else{Serial.println(tmp);}
  // MAX31855 External (thermocouple) Temp
  tmp = (double)TC_CH0.value * 0.25;           // convert fixed pt # to double
  CurrentTemperature=(float)tmp;
  Serial.print("TC Temp = ");                   // print TC temp heading
  if(0x00 == TC_CH0.status){Serial.println(tmp);}
  else if(0x01 == TC_CH0.status){Serial.println("OPEN");}
  else if(0x02 == TC_CH0.status){Serial.println("SHORT TO GND");}
  else if(0x04 == TC_CH0.status){Serial.println("SHORT TO Vcc");}
  else{Serial.println("unknown fault");}

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
  // initalize the chip select pins
  pinMode(CS0_PIN, OUTPUT);
  
  Serial.println("Playing With Fusion: MAX31855-1CH, SEN-30002");

  //----------------------------------------------------------------------------------------


  //------------------------------------radio module initialization------------------------
  printf_begin();
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(15,15);
//  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  
  
  // Open a writing and reading pipe on each radio, with opposite addresses

    radio.openWritingPipe(addresses[0]); //we will write to first adress, the first address will be RPIs
    radio.openReadingPipe(1,addresses[1]);//this arduino address
 
  radio.printDetails();
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
    Temperature = GetTemp(thermocouple0);    // Take reading in the middle of PIDtimestep.  This will block until complete
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
   // delay(1000);
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
      {
        if (SwitchedToCons==0){
          myPID.SetTunings(consKp, consKi, consKd);
          SwitchedToCons=1;
          SwitchedToNormal=0;
          Serial.println(F("SWITCHING TO CONSERVATIVE"));
        }
      }
     else {
      if (abs(Temperature-RequestedTemperature)>1.5*gap&&SwitchedToNormal==0) {
         myPID.SetTunings(Kp, Ki, Kd);
         SwitchedToCons=0;
         SwitchedToNormal=1;
          Serial.println(F("SWITCHING TO NORMAL"));
      }
      
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
