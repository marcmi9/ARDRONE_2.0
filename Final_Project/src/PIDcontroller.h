#include <math.h>


#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

// Because it is a small class I will put the code in the header file. This should normally not be done.

class PIDcontroller {
  
  private:
    double integral;
    double lastError;
    bool reset;
  
  public:
    float kp, ki, kd;

    PIDcontroller () : kp(1), ki(0), kd(0), reset(true), integral(0) {};
    PIDcontroller (float P, float I, float D) : kp(P), ki(I), kd(D), reset(true), integral(0) {};

    double Update (double error, double elapsedTime) {
      double res;
      double derivative = 0;
      integral += error * elapsedTime;
      if (elapsedTime > 0 && !reset) {
        derivative = (error - lastError)/elapsedTime;
      }
      res = kp*pow(error,1) + kd*pow(derivative,1) + ki*pow(integral,1);
      lastError = error;
      reset = false; //after first update we set reset variable to false
      return res;
    }
    
    void Reset () {
      integral = 0;
      reset = true;
    } 
    
};

#endif
