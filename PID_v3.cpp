/**********************************************************************************************
* Arduino PID Library - Version 3.0.0 *** Proposed V 3.0.0 Modifed by ZHomeSlice***
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <PID_v3.h>

#define DPRINTSFN(StrSize,Name,Variable,Spaces,Precision) {char S[max((Spaces + Precision + 3), StrSize)];Serial.print(F(" "));Serial.print(F(Name));Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}//StringSize,Name,Variable,Spaces,Precision
#define DPRINTSTIMER(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!
template <class X, class M, class N, class O, class Q>
X PID::map_Generic(X x, M in_min, N in_max, O out_min, Q out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int Direction){
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	//	myDriveDir = DriveDir;
	inAuto = false;
	SetOutputLimits(0, 255);				//default output limit corresponds to
	//the arduino pwm limits
	SetSampleTime(0);//default Exact Sample time in seconds Default Auto Calculate
	SetControllerDirection(Direction);
	SetTunings(Kp, Ki, Kd);
	lastTime = millis()-SampleTime;
	
}
PID::PID(double* Input, double* Output, double* Setpoint){
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	//	myDriveDir = DriveDir;
	inAuto = false;
	lastTime = millis()-SampleTime;
	
}
/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   PID Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/
// kp -  proportional gain
// ki -  Integral gain
// kd -  derivative gain
// dt -  loop interval time
// outMax - maximum value of manipulated variable
// outMin - minimum value of manipulated variable

PID & PID::Compute(){
	if(!inAuto){
		Initialize();
		if (ComputeManual_CB) {
			ComputeManual_CB(); // Callback Function
		}
//		Serial.print("-");
		return *this;// No Calculations were made

	}
//	Serial.print("+");
	double DeltaTS;
	if(SampleTime) DeltaTS = SampleTime * 0.000001;
	else {
		unsigned long Now;
		Now = micros();
		unsigned long DeltaTuS = (Now - lastTime);
		lastTime = Now;
		DeltaTS = ((double)DeltaTuS * 0.000001);//
	}
	double Input = *myInput;
	double Setpoint = *mySetpoint;
	double error = Setpoint - Input;// Calculate error
	double PTerm = (kp * (double) controllerDirection) * error;// Proportional term

	if(ki){
		ITerm += error *  DeltaTS * (ki * (double)controllerDirection); // Integral term
		ITerm = (ITerm <  outMin )? outMin:((ITerm > outMax)? outMax : ITerm); // Prevents Windup
	}
	double DTerm = 0;
	if(kd){
		/* // Derivative term using angle change
		double kdAveFactor = 0.5; // 4 samples
		kdAverage += kdAveFactor * ((Input - lastInput) - kdAverage) ;  // simple 1st order digital low pass filter
		DTerm = (-kd * (double) controllerDirection) * ((kdAverage)  / DeltaTS); //Derivative term  avoid infinity errors
		*/	// two samples averaged
		DTerm = (-kd * (double) controllerDirection) * (((Input - lastInput) * .5)  / DeltaTS); //Derivative term  avoid infinity errors
		lastInput = lastInputX;// averaging 2 readings for smoothing Derivative
		lastInputX = Input;

	}
	double Output = PTerm + ITerm + DTerm; //Compute PID Output
	Output = (Output < outMin )? outMin:((Output > outMax)? outMax : Output); // Sets Limits
	*myOutput = Output;
	if (Compute_CB) {
		Compute_CB(); // Callback Function
	}
	// Debugging
	
	DPRINTSTIMER(100){
		DPRINTSFN(10,"Input",Input,6,2);
		DPRINTSFN(10,"Setpoint",Setpoint,6,2);
		DPRINTSFN(10,"DeltaTS",DeltaTS,8,6);
		DPRINTSFN(10,"PTerm",PTerm,6,2);
		DPRINTSFN(10,"I",(error *  DeltaTS * (ki * (double)controllerDirection)),6,2);
		DPRINTSFN(10,"ITerm",ITerm,6,2);
		DPRINTSFN(10,"DTerm",DTerm,6,2);
		DPRINTSFN(10,"Output",Output,6,2);
	}
	//
	return *this; // return Pointer to this class
}

PID & PID::onManual(void (*CB)(void)){
	Compute_CB = CB;
	return *this; // return Pointer to this class
}

PID & PID::onCompute(void (*CB)(void)){
	Compute_CB = CB;
	return *this; // return Pointer to this class
}


PID & PID::BalanceBotDrive(float TurnOffset){
	//DPRINTSTIMER(100){
	//DPRINTSFN(10,"TurnOffset",TurnOffset,6,2);
	//DPRINTLN();
	//}
	static bool LastADriveDir,LastBDriveDir;
	double Output = * myOutput;
	double AOutput =  Output - TurnOffset;// Power A motor for both forward and reverse
	double BOutput =  Output + TurnOffset;// Power B motor for both forward and reverse
	bool ADriveDir = AOutput > 0; // Zero or stopped can be either true or false
	bool BDriveDir = BOutput > 0; // Zero or stopped can be either true or false
	double absAOutput = fabs(AOutput);
	double absBOutput = fabs(BOutput);

	absAOutput = (absAOutput > outMax)? outMax : absAOutput; // Limits Output
	absBOutput = (absBOutput > outMax)? outMax : absBOutput; // Limits Output
	double DriveAOutput =  PID::map_Generic(absAOutput,0,outMax,DriveMin - PowerOffset,DriveMax);// Power motor for both forward and reverse
	double DriveBOutput =  PID::map_Generic(absBOutput,0,outMax,DriveMin + PowerOffset,DriveMax);// Power motor for both forward and reverse

	DPRINTSTIMER(100){
		DPRINTSFN(10,"YawInput",YawInput,6,2);
		DPRINTSFN(10,"TurnOffset",TurnOffset,6,2);
		DPRINTSFN(10,"Output",Output,6,2);
		DPRINTSFN(10,"AOutput",AOutput,6,2);
		DPRINTSFN(10,"BOutput",BOutput,6,2);
		DPRINTSFN(10,"ADriveDir",ADriveDir,6,2);
		DPRINTSFN(10,"BDriveDir",BDriveDir,6,2);
		DPRINTSFN(10,"absAOutput",absAOutput,6,2);
		DPRINTSFN(10,"absBOutput",absBOutput,6,2);
		DPRINTSFN(10,"DriveAOutput",DriveAOutput,6,2);
		DPRINTSFN(10,"DriveBOutput",DriveBOutput,6,2);
		Serial.println();
	}

	if (LastADriveDir != ADriveDir) {  // check for no power or change in direction Motor A
		if (CostA_CB) {
			CostA_CB(); // Callback Function
		}
	}
	if (LastBDriveDir != BDriveDir) {  // check for no power or change in direction Motor B
		if (CostB_CB) {
			CostB_CB(); // Callback Function
		}
	}
	LastADriveDir = ADriveDir;
	LastBDriveDir = BDriveDir;
	if (AOutput < 0) {
		if (ForwardA_CB) {
			ForwardA_CB(DriveAOutput); // Callback Function
		}
		} else if (AOutput > 0){
		if (ReverseA_CB) {
			ReverseA_CB(DriveAOutput); // Callback Function
		}
	}
	if (BOutput< 0) {
		if (ForwardB_CB) {
			ForwardB_CB(DriveBOutput); // Callback Function
		}
		} else if (BOutput > 0){
		if (ReverseB_CB) {
			ReverseB_CB(DriveBOutput); // Callback Function
		}
	}
	return *this; // return Pointer to this class
}


PID & PID::onCostA(void (*CB)(void)){
	CostA_CB = CB;
	return *this; // return Pointer to this class
}
PID & PID::onCostB(void (*CB)(void)){
	CostB_CB = CB;
	return *this; // return Pointer to this class
}

PID & PID::onForwardA(void (*CB)(double)){
	ForwardA_CB = CB;
	return *this; // return Pointer to this class
}
PID & PID::onReverseA(void (*CB)(double)){
	ReverseA_CB = CB;
	return *this; // return Pointer to this class
}
PID & PID::onForwardB(void (*CB)(double)){
	ForwardB_CB = CB;
	return *this; // return Pointer to this class
}
PID & PID::onReverseB(void (*CB)(double)){
	ReverseB_CB = CB;
	return *this; // return Pointer to this class
}
PID & PID::SetDriveMin(double Min){
	DriveMin = Min;
	return *this; // return Pointer to this class
}
PID & PID::SetDriveMax(double Max){
	DriveMax = Max;
	return *this; // return Pointer to this class
}

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
PID & PID::SetTunings(double Kp, double Ki, double Kd){
	if (Kp<0 || Ki<0 || Kd<0) return *this; // return Pointer to this class
	kp  = Kp;
	ki = Ki;
	kd = Kd;
	return *this; // return Pointer to this class
}
PID & PID::SetKp(double Kp){
	if (Kp<0 ) return *this; // return Pointer to this class
	kp  = Kp;
	return *this; // return Pointer to this class
}
PID & PID::SetKi(double Ki){
	if (Ki<0 ) return *this; // return Pointer to this class
	ki = Ki;
	return *this; // return Pointer to this class
}
PID & PID::SetKd(double Kd){
	if (Kd<0 ) return *this; // return Pointer to this class
	kd = Kd;
	return *this; // return Pointer to this class
}

/* SetSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
PID & PID::SetSampleTime(unsigned long NewSampleTime){
	SampleTime = NewSampleTime * 1000;
	return *this; // return Pointer to this class
}
PID & PID::SetSampleTimeus(unsigned long NewSampleTime){
	SampleTime = NewSampleTime;
	return *this; // return Pointer to this class
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
PID & PID::SetOutputLimits(double Min, double Max){
	if(Min >= Max) return *this; // return Pointer to this class
	outMin = Min;
	outMax = Max;
	return *this; // return Pointer to this class
}
PID & PID::SetOutputMin(double Min){
	outMin = Min;
	return *this; // return Pointer to this class
}
PID & PID::SetOutputMax(double Max){
	outMax = Max;
	return *this; // return Pointer to this class
}
/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
PID & PID::SetMode(int Mode){
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto == !inAuto) PID::Initialize();  /*we just went from manual to auto*/
	inAuto = newAuto;
	return *this; // return Pointer to this class
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void PID::Initialize(){
	double DesiredOutput = *myOutput;
	PID::PrimeIntegral(DesiredOutput);
	lastInput = *myInput;
	return ; // 
}

/* PrimeIntegral()****************************************************************
*	Forces a value to the output to prime for restart (example Stalled Motor)
******************************************************************************/

PID & PID::PrimeIntegral(double DesiredOutput){
	ITerm = (DesiredOutput < outMin )? outMin:((DesiredOutput > outMax)? outMax : DesiredOutput); // Primes Integral
	return *this; // return Pointer to this class
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.)
******************************************************************************/
PID & PID::SetControllerDirection(int Direction){
	controllerDirection = Direction;
	return *this; // return Pointer to this class
}

/* Status Funcions*************************************************************
******************************************************************************/
double PID::GetKp(){ return  kp;}
double PID::GetKi(){ return  ki;}
double PID::GetKd(){ return  kd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}



