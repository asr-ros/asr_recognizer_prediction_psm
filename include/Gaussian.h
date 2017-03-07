/**

Copyright (c) 2016, Braun Kai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <boost/shared_ptr.hpp>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>

namespace ASR
{
 
    class Gaussian
    {
    private:
      double mean;
      double var;
      
    public:
      
      /**
       * Creates a Gaussian-Kernel with the given parameters
       * @param mean - the mean of the gaussian kernel
       * @param variance - the variance of the gaussian kernel
       * 
       */
	Gaussian(double mean, double variance)
	{
	  this->mean = mean;
	  this->var = variance;
	}
	
	
	/**
	 * Samples one random number from the gaussian.
	 * Values near the mean are the most likely
	 * @return a random value
	 */
	float sampleRandomValue()
	{
	  std::random_device rd;
	  std::mt19937 gen(rd()); 
	  
	  // values near the mean are the most likely
	  // standard deviation affects the dispersion of generated values from the mean
	  std::normal_distribution<> d(mean, var);
	  
	  return d(gen);
	}
	
	/**
	 * Samples n random values from the gaussian and stores them in a given vector.
	 * @param v - the output vector that gains the generated values
	 * @param number - the number of values that should be generated
	 */
	void sampleRandomValues(std::vector<float> &v, int n)
	{
      assert(n > 0);
      //assert(v != nullptr);
	  
	  std::random_device rd;
	  std::mt19937 gen(rd()); 
	  
	  // values near the mean are the most likely
	  // standard deviation affects the dispersion of generated values from the mean
	  std::normal_distribution<> d(mean, var);
	  
	  for (int i = 0; i < n; i++)
	  {
	    v.push_back( d(gen) );
	  }
	}
	
	
	
    private:
      /**
       * A method for testing the random number generator
       * -- Only for testing --
       */
	void test()
	{
	  std::random_device rd;
	  std::mt19937 gen(rd()); 
	  std::normal_distribution<> d(mean, var);
	  
	  std::map<int, int> hist;
	  for(int n=0; n<10000; ++n) 
	  {
	    ++hist[std::round(d(gen))];
	  }
	  for(auto p : hist) 
	  {
	    std::cout << std::fixed << std::setprecision(1) << std::setw(2)
	    << p.first << ' ' << std::string(p.second/200, '*') << '\n';  
	  }
	}
	
	
    };
    
    typedef boost::shared_ptr<Gaussian> GaussianPtr;
  
  
  
}
