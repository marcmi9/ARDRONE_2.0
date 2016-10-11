#ifndef FILTER_H
#define FILTER_H

#include <deque>

/* Filter class
 * Abstract class that takes care of the filter window. Functionality should be implemented on every child class by overriding ApplyFilter().
 * Data is sorted from new to old in the queue window (new = front, old = back)
 * By default, or if windowSize is not a positive integer, a filter of size 1 is created.
 */
class Filter {
  
  private:
    int inputSize;
    int outputSize;
  
  protected:
    std::deque<double> inputWindow;
    std::deque<double> outputWindow;
    virtual double ApplyFilter() = 0; //Pure virtual function (we need to override it)
  
  public:
    // Constructor
    Filter (int inputWindowSize = 1, int outputWindowSize = 1) {
      if (inputWindowSize <= 0) { //safety check
        inputWindowSize = 1;
      }
      if (outputWindowSize <= 0) { //safety check
        outputWindowSize = 1;
      }
        inputSize = inputWindowSize;
        outputSize = outputWindowSize;
    };

    double Update (double value) {
      double res;
      if (inputWindow.size() >= inputSize) { //it should never be higher
        inputWindow.pop_back(); //throw away oldest element
      }
      inputWindow.push_front(value); //insert new element   
      res = ApplyFilter();
      if (outputWindow.size() >= outputSize) { //it should never be higher
        outputWindow.pop_back(); //throw away oldest element
      }
      outputWindow.push_front(res); //insert new element
      return res;
    }
    
    void Resize(int inputWindowSize, int outputWindowSize) {
      if (inputWindowSize <= 0) { //safety check
        inputWindowSize = 1;
      }
      if(inputWindowSize < inputSize) {
        inputWindow.resize(inputWindowSize); //keeps values at the front when reducing size.
      }
      inputSize = inputWindowSize;
      
      if (outputWindowSize <= 0) { //safety check
        outputWindowSize = 1;
      }
      if(outputWindowSize < outputSize) {
        outputWindow.resize(outputWindowSize); //keeps values at the front when reducing size.
      }
      outputSize = outputWindowSize;
    }

    void Reset () {
      inputWindow.clear();
      outputWindow.clear();
    } 
};

#endif
