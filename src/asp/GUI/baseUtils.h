// MIT License Terms (http://en.wikipedia.org/wiki/MIT_License)
// 
// Copyright (C) 2011 by Oleg Alexandrov
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef BASEUTILS_H
#define BASEUTILS_H

#include <vector>
#include <string>
#include <sstream>

namespace utils{
  
  template<class T>
  const T * vecPtr(const std::vector<T>& X){
    if (X.size() == 0) return NULL;
    else               return &X.front();   
  }

  template<class T>
  T * vecPtr(std::vector<T>& X){
    if (X.size() == 0) return NULL;
    else               return &X.front();   
  }

  template<class T>
  std::string num2str(T num){
    std::ostringstream S;
    S << num;
    return S.str();
  }

  inline double distance(double x0, double y0, double x1, double y1){
    return sqrt( (x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0) );
  }
  
  inline int iround(double x){ return (int)round(x); }
  inline int iceil (double x){ return (int)ceil( x); }
  inline int ifloor(double x){ return (int)floor(x); }
  inline int isign (double x){
    if (x > 0) return  1;
    if (x < 0) return -1;
    return 0;
  }

  template<class T>
  void eraseMarkedElements(std::vector<T> & vals, const std::vector<char> & marks){

    assert(vals.size() == marks.size());
    int n = 0;
    for (int i = 0; i < (int)vals.size(); i++){
      if (marks[i]) continue;
      vals[n] = vals[i];
      n++;
    }

    vals.resize(n);
  }
  
}

#endif
