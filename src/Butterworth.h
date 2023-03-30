#ifndef Butterworth_h
#define Butterworth_h

#include <math.h>

class Butterworth
{
private:
    double cutOffFrequency, sampleFrequency;    
    double a[2];
    double b[3];
    double x[3],y[3];
    void CalculateLPKoeff(double cutOffFrequency,double sampleFrequency);
    double LPFilter(double inValue);
public:
    
    Butterworth(double cutOffHZ, double sampleHZ);
    Butterworth();
    
    void InitializeFilter(double _cutOffFrequency,double _sampleFrequency);
    
    void setCutOffFrequency(double x){cutOffFrequency = x;}
    void setSampleFrequency(double x){sampleFrequency = x;}
    double calculate(double inValue){return LPFilter(inValue);}
    void printCoeffs();
};

Butterworth::Butterworth(){}

Butterworth::Butterworth(double _cutOffFrequency,double _sampleFrequency){
    setCutOffFrequency(_cutOffFrequency);
    setSampleFrequency(_sampleFrequency);
    CalculateLPKoeff(_cutOffFrequency, _sampleFrequency);
}
void Butterworth::InitializeFilter(double _cutOffFrequency,double _sampleFrequency){
    setCutOffFrequency(_cutOffFrequency);
    setSampleFrequency(_sampleFrequency);
    CalculateLPKoeff(_cutOffFrequency, _sampleFrequency);
}
void Butterworth::CalculateLPKoeff(double _cutOffFrequency, double _sampleFrequency){
    double lam =1.0/ tan(M_PI*_cutOffFrequency/_sampleFrequency);
    b[0] = 1.0 / (1.0 + sqrt(2.0)*lam + lam*lam);
    b[1]= 2*b[0];
    b[2]= b[0];
    a[0] = (2.0 * (lam*lam - 1.0) * b[0]);
    a[1] = (-(1.0 - sqrt(2.0)*lam + lam*lam) * b[0]);
}
double Butterworth::LPFilter(double inValue){
//y(n)  =  b[0].x(n) + b[1].x(n-1) + b[2].x(n-2) + a[0].y(n-1) + a[1].y(n-2)
    y[0]= b[0]*inValue + b[1]*x[1] + b[2]*x[2] + a[0]*y[1] + a[1]*y[2];
    y[2] = y[1];
    y[1] = y[0];
    x[2] = x[1];
    x[1] = inValue;
    return y[0];
}
void Butterworth::printCoeffs(){
    Serial.print("a1 = ");
    Serial.println(a[0],6);
    Serial.print("a2 = ");
    Serial.println(a[1],6);
    Serial.print("b0 = ");
    Serial.println(b[0],6);
    Serial.print("b1 = ");
    Serial.println(b[1],6);
    Serial.print("b2 = ");
    Serial.println(b[2],6);
}
#endif