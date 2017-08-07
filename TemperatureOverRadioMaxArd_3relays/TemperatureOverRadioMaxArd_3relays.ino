#include <SPI.h>
// Include the libraries we need for temperature sensor
#include <PlayingWithFusion_MAX31855_J_correction.h>
#include <PlayingWithFusion_MAX31855_STRUCT_corr.h>
#include <PlayingWithFusion_MAX31855_T_correction.h>
#include <PlayingWithFusion_MAX31855_Wcorr.h>

#define K_type 1
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

const float TC2Cor[8]={-3.82371217e-12 ,  9.59537434e-10 , -1.54011212e-09 , -1.23583681e-05,   9.18982104e-04 , -2.36002184e-02 ,  1.46579258e+00 , -6.13656839e+00};
const float TC3Cor[8]={-2.88994787e-12   ,9.83797425e-10,  -9.60434093e-08   ,1.98371664e-06,   1.35986650e-04  ,-6.54505392e-03  , 1.33944386e+00 , -6.03994568e+00};

//direct relay control
int Relay1Pin=4; //relay for thermocouple 2
int Relay2Pin=11; //second relay for thermocouple 2
int Relay3Pin=12; //relay for thermocouple 3




//-----------------variables for temperature sensor---------------
// setup CS pins used for the connection with the sensor
// other connections are controlled by the SPI library)
int8_t CS0_PIN = 3; //marked 1
int8_t CS1_PIN = 6; //marked 2
int8_t CS2_PIN = 10; //marked 3



float Eps=1.0e-6;



PWFusion_MAX31855_TC thermocouple[3]={
          PWFusion_MAX31855_TC(CS0_PIN),
          PWFusion_MAX31855_TC(CS1_PIN),
          PWFusion_MAX31855_TC(CS2_PIN)};
/*
thermocouple[0]=new PWFusion_MAX31855_TC(CS0_PIN);
thermocouple[1]=new PWFusion_MAX31855_TC(CS1_PIN);
thermocouple[2]=new PWFusion_MAX31855_TC(CS2_PIN);
*/
/*
float RequestedTemperature=0.0;
float RequestedTemperatureOLD=0.0;
float Temperature= 0;


PWFusion_MAX31855_TC  thermocouple0(CS0_PIN);
PWFusion_MAX31855_TC  thermocouple1(CS1_PIN);
PWFusion_MAX31855_TC  thermocouple2(CS2_PIN);
*/

float RequestedTemperatureArray[3]={0.0,0.0,0.0};
float RequestedTemperatureOLDArray[3]={0.0,0.0,0.0};
float TemperatureArray[3] = {0.0,0.0,0.0};

//---------------------------------------------------------------


//--------------------------variables for radio module-----------------
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,53); //9 is CE pin 53 is CSN


byte addresses[][6] = {"Toch1","Toch2"};

unsigned long start_timeArray[3]={0,0,0};//for logging when radio trasmission went
//-------------------------------------------------------------------------




//---------------------------Variables for PID---------------------------
//Define Variables we'll be connecting to
double SetpointArray[3]={26,50,80};
double OutputArray[3]={0,0,0};
double Setpoint0, Setpoint1,Setpoint2,
        Input0,Input1,Input2, Output0, Output1, Output2;


//Specify the links and initial tuning parameters
//double Kp=5., Ki=1.0, Kd=0.8;
double KpArray[3]={5.,5.,5.}, KiArray[3]={1.0,1.0,1.0}, KdArray[3]={0.8,0.8,0.8};


//Conservative tuning parameters
//double consKp=0.5, consKi=0.1000, consKd=2.0;
double consKpArray[3]={0.5,0.5,0.5}, consKiArray[3]={0.1,0.1,0.1}, consKdArray[3]={2.0,2.0,2.0};

double gap=8.;//if difference between target and actual is lower than this, change to conservative PID params

PID myPID[3]={PID(&Input0, &Output0, &Setpoint0, KpArray[0], KiArray[0], KdArray[0], DIRECT),
              PID(&Input1, &Output1, &Setpoint1, KpArray[1], KiArray[1], KdArray[1], DIRECT),
              PID(&Input2, &Output2, &Setpoint2, KpArray[2], KiArray[2], KdArray[2], DIRECT)};

/*
float Ratio;
int AlreadyRequested=0;
int SwitchedToNormal=0,SwitchedToCons=0;
*/

float RatioArray[3]={1,1,1};
int AlreadyRequestedArray[3]={0,0,0};
int SwitchedToNormalArray[3]={0,0,0}, SwitchedToConsArray[3]={0,0,0}; //boolean for checks if we already changed parameters for PID

unsigned long windowStartTimeArray[3];
unsigned long currentTimeArray[3],TimeDifArray[3];


float WindowSizeArray[3] = {10,10,10};//range of pid output from 0 to this, it will be ratio of PIDSizeDuration when PID wants relay ON
float PIDStepDurationArray[3]={10000,10000,10000};//ms




//-----------------------------------------------------------------------




//=======================FUNCTIONS=======================
/*
 * Function get and print the temperature
 */
float GetTemp(PWFusion_MAX31855_TC &sens)
{ 
  double tmp;
  float CurrentTemperature=0;
  static struct var_max31855 TC_CH = {0, 0, 0, J_type, 0};
  struct var_max31855 *tc_ptr;
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus

   Serial.print("Requesting temperatures...");
  // update TC0
  tc_ptr = &TC_CH;
  sens.MAX31855_update(tc_ptr);        // Update MAX31855 readings
  Serial.println("DONE");
  
  tmp = (double)TC_CH.ref_jcn_temp * 0.0625;  // convert fixed pt # to double
  Serial.print("Tint = ");                      // print internal temp heading
  if((-100 > tmp) || (150 < tmp)){Serial.println("unknown fault");}
  else{Serial.println(tmp);}
  // MAX31855 External (thermocouple) Temp
  tmp = (double)TC_CH.value * 0.25;           // convert fixed pt # to double
  CurrentTemperature=(float)tmp;
  Serial.print("TC Temp = ");                   // print TC temp heading
  if(0x00 == TC_CH.status){Serial.println(tmp);}
  else if(0x01 == TC_CH.status){Serial.println("OPEN");}
  else if(0x02 == TC_CH.status){Serial.println("SHORT TO GND");}
  else if(0x04 == TC_CH.status){Serial.println("SHORT TO Vcc");}
  else{Serial.println("unknown fault");}

  return CurrentTemperature;

}
float ThermocoupleCorrection(float T,int TCnumber){
  float Res=0.0;
  if (TCnumber==2){
    for (int i=0;i<8;i++){
      Res=Res+TC2Cor[i]*pow(T,7-i);
      }
      return Res;
    }
  if (TCnumber==3){
    for (int i=0;i<8;i++){
      Res=Res+TC3Cor[i]*pow(T,7-i);
      }
      return Res;
    }
  return T;
  }
//======================================================================================


//SETUP
void setup(void)
{

//------------------relay setup setup------------------------
  pinMode(Relay1Pin, OUTPUT);
  digitalWrite(Relay1Pin,LOW);
  pinMode(Relay2Pin, OUTPUT);
  digitalWrite(Relay2Pin,LOW);
  pinMode(Relay3Pin, OUTPUT);
  digitalWrite(Relay3Pin,LOW);
//----------------------------------------------------------
  
  //--------------------------Temperature sensor initialization----------------------------
  // start serial port
  Serial.begin(115200);
  // initalize the chip select pins
  pinMode(CS0_PIN, OUTPUT);
  pinMode(CS1_PIN, OUTPUT);
  pinMode(CS2_PIN, OUTPUT);
  
  Serial.println("Playing With Fusion: MAX31855-1CH, SEN-30002");

  //----------------------------------------------------------------------------------------


  //------------------------------------radio module initialization------------------------
  printf_begin();
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_HIGH);
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

  Setpoint0=SetpointArray[0];
  Setpoint1=SetpointArray[1];
  Setpoint2=SetpointArray[2];
for (int k=0;k<3;k++){
  windowStartTimeArray[k] = millis();

  //initialize the variables we're linked to

  
  AlreadyRequestedArray[k]=0;

  //tell the PID to range between 0 and the full window size
  myPID[k].SetOutputLimits(0, WindowSizeArray[k]);


  //turn the PID on
  myPID[k].SetMode(AUTOMATIC);

  }
  
//----------------------------------------------------------------------------------


}//setup




void loop() {
for(int k=0;k<3;k++){
  
  if (millis()-windowStartTimeArray[k]>PIDStepDurationArray[k]/2&&AlreadyRequestedArray[k]==0){
    Serial.print(F("-----------AskingThermocouple "));
    Serial.print(k+1);
    Serial.println(F("-----------"));
    TemperatureArray[k] = GetTemp(thermocouple[k]);    // Take reading in the middle of PIDtimestep.  This will block until complete
    
    TemperatureArray[k]=ThermocoupleCorrection(TemperatureArray[k],k+1);
    Serial.print(F("Corrected temperature= "));
    Serial.println(TemperatureArray[k]);
    AlreadyRequestedArray[k]=1;
  }
  RequestedTemperatureOLDArray[k]=RequestedTemperatureArray[k]; //saving previous request
    if( radio.available()){
                                                                    // If receive radio signal
      while (radio.available()) {                                   // 
        radio.read( &RequestedTemperatureArray, sizeof(RequestedTemperatureArray) );        //read requested temperature  
      }

     
      radio.stopListening();                                        // First, stop listening so we can talk
      //find out actual temperature at sensor and send it
          Serial.println(F("---------------------------------------------"));
    
   start_timeArray[k]=micros();
   // delay(1000);
      if (!radio.write( &TemperatureArray, sizeof(TemperatureArray) )){  //send actual temperature over radio
       Serial.println(F("failed"));
     }
    radio.startListening();

    //now change set point for pid if new requested temperature is different to previous
    if(abs(RequestedTemperatureOLDArray[k]-RequestedTemperatureArray[k])<Eps) SetpointArray[k]=RequestedTemperatureArray[k];

    Serial.println(F("---------------------------------------------"));
    Serial.println(F("Requested Temperatures:    Actual Temperatures from Sensors:    Sent at time:"));
    for (int j=0;j<3;j++){
      Serial.print(RequestedTemperatureArray[j]);
      Serial.print(F("                                "));
      Serial.print(TemperatureArray[j]);
      Serial.print(F("                                  "));
      Serial.println(start_timeArray[j]); 
    }
    Serial.println(F("---------------------------------------------"));      
   }
   
//-------------------------PID CALCULATION and switching relay------------------------------

  currentTimeArray[k]=millis();
  TimeDifArray[k]=currentTimeArray[k]-windowStartTimeArray[k];
  if (TimeDifArray[k]<PIDStepDurationArray[k]){
    if ((float)TimeDifArray[k]/PIDStepDurationArray[k]<RatioArray[k])
      switch (k){
        case 0:   {break;}
        case 1: {digitalWrite(Relay1Pin, LOW);digitalWrite(Relay2Pin, LOW); break;}//switching 2 together
        case 2: {digitalWrite(Relay3Pin, LOW); break;}
        }
      
    else
        switch (k){
        case 0:   { break;}
        case 1: {digitalWrite(Relay2Pin, HIGH);digitalWrite(Relay1Pin, HIGH); break;}//switching 2 together
        case 2: {digitalWrite(Relay3Pin, HIGH); break;}
        }
  }
  else{
    //switching pid parameters if we are close to the target
    //switching back not at the same gap as we do not want to switch back and forth when at the gap
    //exactly
    if (abs(TemperatureArray[k]-RequestedTemperatureArray[k])<gap)
      {
        if (SwitchedToConsArray[k]==0){
          
          myPID[k].SetTunings(consKpArray[k], consKiArray[k], consKdArray[k]);
          SwitchedToConsArray[k]=1;
          SwitchedToNormalArray[k]=0;
          Serial.print(F("SWITCHING TO CONSERVATIVE for PID "));
          Serial.println(k);
        }
      }
     else {
      if (abs(TemperatureArray[k]-RequestedTemperatureArray[k])>1.5*gap&&SwitchedToNormalArray[k]==0) {
          myPID[k].SetTunings(KpArray[k], KiArray[k], KdArray[k]);
         SwitchedToConsArray[k]=0;
         SwitchedToNormalArray[k]=1;
          Serial.print(F("SWITCHING TO NORMAL for PID "));
          Serial.println(k);
      }
      
     }
    AlreadyRequestedArray[k]=0;//resetting temperature request counter
    windowStartTimeArray[k]+=PIDStepDurationArray[k];
    Input0 = TemperatureArray[0];
    Input1 = TemperatureArray[1];
    Input2 = TemperatureArray[2];
    Setpoint0=SetpointArray[0];
    Setpoint1=SetpointArray[1];
    Setpoint2=SetpointArray[2];
    
    
    myPID[k].Compute();

    OutputArray[0]=Output0;
    OutputArray[1]=Output1;
    OutputArray[2]=Output2;

    
    RatioArray[k]=OutputArray[k]/WindowSizeArray[k];//Ratio if end of step matches exactly when last temperatrue was measured
    //need to modify Ratio as most likely during starting part of pid timestep the relaypin is still LOW
    //due to previous relay pin timestep setting it so
    //our pidtimestep is ideally divided into (HIGH__timeratio__LOW)
    //and actuall will most likely be (LOW__startofnewpidtimestep__HIGH__timeration__LOW)
    //we need to adjust timeratio so LOW time at the start + LOW time at the end of actual pidtimestep
    //would be equal to total LOW time for ideal pidtimestep
    //PIDStepDuration*Ratio is duration that PID wants to HIGH time
    //TimeDif-PIDStepDuration  is duration of LOW transfered from previous pidtimestep to current one
    //PIDStepDuration*Ratio+(TimeDif-PIDStepDuration) is when we actually want to swith relay to LOW again
    RatioArray[k]=(PIDStepDurationArray[k]*RatioArray[k]+(TimeDifArray[k]-PIDStepDurationArray[k]))/PIDStepDurationArray[k];
    TimeDifArray[k]=currentTimeArray[k]-windowStartTimeArray[k];
    if ((float)TimeDifArray[k]/PIDStepDurationArray[k]<RatioArray[k])
        switch (k){
        case 0:   { break;}
        case 1: {digitalWrite(Relay2Pin, LOW);digitalWrite(Relay1Pin, LOW); break;}
        case 2: {digitalWrite(Relay3Pin, LOW); break;}
        }
    else
        switch (k){
        case 0:   { break;}
        case 1: {digitalWrite(Relay2Pin, HIGH); digitalWrite(Relay1Pin, HIGH);break;}
        case 2: {digitalWrite(Relay3Pin, HIGH); break;}
        }
      
    }
    if (millis()%2000>1997){//trying to output each 2 seconds
     Serial.println(F("PIDRatios=           currentTime=       currentTime - windowStartTime="));
     for(int j=0;j<3;j++){
       Serial.print(RatioArray[j]);
       Serial.print(F("                "));
       Serial.print(currentTimeArray[k]);
       Serial.print(F("                    "));
       Serial.println(currentTimeArray[k] - windowStartTimeArray[k]);
     }
   }
}// for (k from 0 to 3)
  //----------------------------------------------------------------

} // Loop
