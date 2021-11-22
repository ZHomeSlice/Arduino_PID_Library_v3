extern double YawInput;

#define DEBUG
#ifndef PID_v3_h
#define PID_v3_h
#define LIBRARY_VERSION	3.0.0
static void nothing(void) {
}
class PID
{
	public:
	typedef void (*voidFuncPtr)(void);// Create a type to point to a funciton.
	typedef void (*voidFuncPtrD)(double);// Create a type to point to a funciton.

	//Constants used in some of the functions below
	#define AUTOMATIC	1
	#define MANUAL	0
	#define DIRECT  1
	#define REVERSE  -1

	//commonly used functions **************************************************************************
	PID(double*, double*, double*,           // * constructor.  links the PID to the Input, Output, and
	double, double, double, int);        //   Setpoint.  Initial tuning parameters are also set here
	PID(double*, double*, double*);
	PID & Compute();                         // * performs the PID calculation after receiving new input.
	PID & onManual(void (*CB)(void));        // Callback funstion for Manual mode
	PID & onCompute(void (*CB)(void));		 // Callback function for Compute conplete

	PID & SetMode(int Mode);                 // * sets PID to either Manual (0) or Auto (non-0)
	PID & SetOutputLimits(double, double);   // clamps the output to a specific range. 0-255 by default
	PID & SetOutputMin(double);              // clamps the output to a specific range. 0-255 by default
	PID & SetOutputMax(double);              // clamps the output to a specific range. 0-255 by default
	//available but not commonly used functions ********************************************************

	PID & SetTunings(double, double, double);// * of changing tunings during runtime for Adaptive control
	PID & SetKp(double);                     // * of changing tunings during runtime for Adaptive control
	PID & SetKi(double);                     // * of changing tunings during runtime for Adaptive control
	PID & SetKd(double);                     // * of changing tunings during runtime for Adaptive control
	PID & SetControllerDirection(int);	     // * Sets the Direction, or "Action" of the controller. DIRECT
	//   or REVERSE
	PID & SetSampleTime(unsigned long = 0);  // * sets the EXACT frequency, in Milliseconds, with which
	//   the PID calculation is performed.
	PID & SetSampleTimeus(unsigned long = 0);// * sets the EXACT frequency, in Microseconde, with which
	//   the PID calculation is performed.
	PID & PrimeIntegral(double);             // * sets the Iterm to an offset to assist in startup
	// Balancing Bot Stuff
	PID & PID::BalanceBotDrive(float TurnOffset);
	PID & SetDriveMin(double);
	PID & SetDriveMax(double);
	PID & onCostA(void (*CB)(void));
	PID & onCostB(void (*CB)(void));
	PID & onForwardA(void (*CB)(double));
	PID & onReverseA(void (*CB)(double));
	PID & onForwardB(void (*CB)(double));
	PID & onReverseB(void (*CB)(double));
	//Display functions ********************************************************************************
	double GetKp();						     // These functions query the pid for interal values.
	double GetKi();						     // they were created mainly for the pid front-end,
	double GetKd();						     // where it's important to know what is actually
	int GetMode();						     // inside the PID.
	int GetDirection();					     //
	private:
	void Initialize();
	
	double kp;								 // * (P)roportional Tuning Parameter
	double ki;								 // * (I)ntegral Tuning Parameter
	double kd;								 // * (D)erivative Tuning Parameter
	double kdAverage;
	int controllerDirection;

	double *myInput;						 // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;						 //   This creates a hard link between the variables and the
	double *mySetpoint;						 //   PID, freeing the user from having to constantly tell us
	bool *DriveDir; 						 //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	double ITerm;
	double lastInputX;
	double lastInput;

	unsigned long SampleTime;
	double outMin;
	double outMax;
	double DriveMin;
	double DriveMax;
	bool inAuto;
	voidFuncPtr ComputeManual_CB = nothing;
	voidFuncPtr Compute_CB = nothing;
	// Balancing Bot Stuff
	double PowerOffset = 0;					 // Balancing Bot: for unbalanced motors
	voidFuncPtr CostA_CB = nothing;
	voidFuncPtr CostB_CB = nothing;
	voidFuncPtrD ForwardA_CB = nothing;
	voidFuncPtrD ReverseA_CB = nothing;
	voidFuncPtrD ForwardB_CB = nothing;
	voidFuncPtrD ReverseB_CB = nothing;

	template <class X, class M, class N, class O, class Q>
	X PID::map_Generic(X x, M in_min, N in_max, O out_min, Q out_max);
};
#endif

