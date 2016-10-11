#ifndef LOWPASS_H
#define LOWPASS_H

#include "Filter.h"

class LowPass : public Filter {

  private:
    float num[2] = {0.09776,0.0419}; // inputWindowSize must be 2
    float den[1] = {0.8603}; // outputWindowSize must be 1
  
  public:
    LowPass () : Filter(2,1) {};

 protected:
    double ApplyFilter () {
      double sum = 0.0;
      int i = 0;

      for (int i = 0; i < Filter::inputWindow.size(); i++) {
        sum += num[i]*Filter::inputWindow[i];
      }

      for (int i = 0; i < Filter::outputWindow.size(); i++) {
        sum += den[i]*Filter::outputWindow[i];
      }

      return sum;
    }
};

#endif
