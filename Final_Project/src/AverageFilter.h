#ifndef AVERAGE_FILTER_H
#define AVERAGE_FILTER_H

#include "Filter.h"

class AverageFilter : public Filter {
  
  public:
    AverageFilter () {};
    AverageFilter (int windowSize) : Filter(windowSize) {};

  protected:
    double ApplyFilter () {
      double sum = 0.0;
      for (std::deque<double>::iterator it = Filter::inputWindow.begin(); it != Filter::inputWindow.end(); ++it) {
        sum += *it;
      }
      return sum / Filter::inputWindow.size();
    }
};

#endif
