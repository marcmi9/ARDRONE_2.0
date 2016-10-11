#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "Filter.h"
class MedianFilter : public Filter {
  
  public:
    MedianFilter () {};
    MedianFilter (int windowSize) : Filter(windowSize) {};
  
  protected:
    double ApplyFilter () {
      std::deque<double> aux = Filter::inputWindow;
      sort(aux.begin(), aux.end()); //elements sorted in ascendent order
      int size = aux.size();
      if (size % 2 == 0){
        //size is even, we need to average the two in the middle
        return (aux.at(size/2) + aux.at(size/2-1)) / 2;
      } else {
        //we just take the middle one
        return aux.at(size/2);
      }
    }
};

#endif
