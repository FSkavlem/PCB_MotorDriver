#ifndef PID_h
#define PID_h
#include "Butterworth.h"

class PID
{
private:
    int pidtype;
    bool _motorMaxPowerReached;
    long timerNow, timerLast;
    double filteredError,error,prevError, prevFilteredError, proportional, integral, derivative,filterDerivative, out;
    double calculatePID1(double desiredOut, double current);
    double calculatePID2(double desiredOut, double current);
    double calculatePID3(double desiredOut, double current);
    Butterworth butter;

public:
    double Kp, Ki,Kd, dt;
    void resetPID();
    double calculate(double desiredOut, double current); 

    void updateMotorClamp(bool x){_motorMaxPowerReached = x;}
    double getIntegral(){ return integral;}
    double getDerivative(){ return derivative;}
    double getFilteredDerivative(){ return filterDerivative;}
    void printFilterCoeffs();

    PID(double kp, double ki, double kd, double cycletime){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        dt = cycletime;
        pidtype = 1;
    }
    PID(double kp, double ki, double kd, double cycletime, bool motorMaxPowerReached){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        dt = cycletime;
        _motorMaxPowerReached = motorMaxPowerReached;
        pidtype = 2;
    }

    PID(double kp, double ki, double kd, double cycletime,bool motorMaxPowerReached, double HZcutoff){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        dt = cycletime;
        _motorMaxPowerReached = motorMaxPowerReached;
        butter.InitializeFilter(HZcutoff,1.0/cycletime);
        pidtype = 3;
    }
};

void PID::resetPID(){
  out = 0;
  error = 0;
  prevError = 0;
  proportional = 0;
  integral = 0;
  derivative = 0;
  filterDerivative = 0;
}
void PID::printFilterCoeffs(){
    butter.printCoeffs();
}
double PID::calculate(double desiredOut, double current){
    switch (pidtype)
    {
    case 1:
        return calculatePID1(desiredOut, current);
        break;
    case 2:
        return calculatePID2(desiredOut, current);
        break;
    case 3:
        return calculatePID3(desiredOut, current);
        break;
    default:
        return -1;
        break;
    }
    return 0;
}
double PID::calculatePID1(double desiredOut, double current){
  //PLAIN PID
  error = desiredOut-current;
  proportional = error;
  integral = integral+(error*dt);
  derivative=(error-prevError)/dt;
  out = Kp*proportional + Ki*integral +Kd*derivative;
  prevError = error;
  return out;
}
double PID::calculatePID2(double desiredOut, double current){
  //PID WITH ANTI WINDUP
  error = desiredOut-current;
  proportional = error;
  //Antiwindup
  integral = _motorMaxPowerReached ? integral : integral+(error*dt);
  derivative=(error-prevError)/dt;
  out = Kp*proportional + Ki*integral + Kd*derivative;
  prevError = error;
  return out;
}
double PID::calculatePID3(double desiredOut, double current){
  //PID WITH ANTIWIND UP AND FILTERED DERIVATE
  error = desiredOut-current;
  proportional = error;
  //Antiwindup
  integral = _motorMaxPowerReached ? integral : integral+(error*dt);
  //filtered derivative, lowpass filter, both derivative and filtered here so we can measure the difference
  derivative=((error-prevError)/dt);
  
  filteredError = butter.calculate(error);
  filterDerivative = ((filteredError-prevFilteredError)/dt);
  
  out = Kp*proportional + Ki*integral + Kd*filterDerivative;
  prevError = error;
  prevFilteredError = filteredError;
  return out;
}


#endif