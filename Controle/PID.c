//
// Malha de Controle PID
//


double goal;

float kp;                  // * (P)roportional Tuning Parameter
float ki;                  // * (I)ntegral Tuning Parameter
float kd;                  // * (D)erivative Tuning Parameter		

double integral, lastInput;
double outMin, outMax;
unsigned int SampleTime;

void vSetPID(float Kp, float Ki, float Kd) {
   if (Kp<0 || Ki<0 || Kd<0) return;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}


void vConfigurePID (double setpoint, float Kp, float Ki, float Kd, double max, double min) {
    SampleTime = 100;
    goal = setpoint;
    lastInput = 0;
    integral = 0;
    outMin = min;
    outMax = max;
    vSetPID(Kp, Ki, Kd);			
}

double dCalculatePID(double input) {

    /*Compute all the working error variables*/
    double error = goal - input;
    double dInput = (input - lastInput);

    integral += (ki * error);
    if (integral > outMax) integral = outMax;
    else if (integral < outMin) integral = outMin;
    
    /*Compute PID Output*/
    double output = kp * error + integral - kd * dInput;
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    
    lastInput = input;
    return output;
}

// ??

void vSetOutputLimits(double min, double max) {
    if(min >= max) return;
    outMin = min;
    outMax = max;

    if(integral > outMax) integral = outMax;
    else if(integral < outMin) integral = outMin;
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void vSetSampleTime(int NewSampleTime) {
   if (NewSampleTime > 0) {
      double ratio  = (double) NewSampleTime / (double) SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned int) NewSampleTime;
   }
}

