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

#include <boost/random.hpp>
#include <time.h>
#include "MultidimensionalGaussian.h"
#include <Eigen/Core>

namespace ASR
{

/**
  * This class contains a list of multidimensional gaussians.
  * With weights they model a gaussian mixture model.
  */
class GMM
{
private:
    std::vector<ASR::MultidimensionalGaussianPtr> models;
    std::vector<float> weights;
    std::vector<float> histogramm;
    boost::mt19937 rng;


    /**
      * Get a random number between min and max.
      * @param min - min value
      * @param max - max value
      * @return xi € [min, max)
      */
    float gen_random_float(float min, float max)
    {
        boost::uniform_real<float> u(min, max);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(rng, u);
        return gen();
    }

public:
    GMM()
    {
        // Do some initialzations
        rng.seed(time(0));
        histogramm.push_back(0.0f);
    }

    /**
      * Adds a new multidimensional gaussian to the gaussian mixture model.
      * @param weight - the weight of the new gaussian kernel
      * @param mean - mean values of the new gaussian kernel
      * @param covariance - the covariance matrix of the new gaussian kernel
      */
    void addGaussianModel(float weight, std::vector<float> mean, MatrixXf covariance)
    {
        //ROS_INFO("Adding gaussian to gmm. weight=%.2f", weight);

        weights.push_back(weight);
        models.push_back(boost::shared_ptr<ASR::MultidimensionalGaussian>(new ASR::MultidimensionalGaussian(mean, covariance)));
        histogramm.push_back(histogramm.at(histogramm.size()-1) + weight );
    }


    /**
      * Samples a random vector from the gaussian mixture model and stores the values in
      * the given vector x.
      * First a gaussian kernel is randomly selected. "Heavier" weighted kernels are more likely than "lighter"
      * weighted kernels.
      * The dimension of x depends on the dimension of the kernels which were given in the initialization.
      * @param x - the output vector which contains the sampled values.
      */
    void sampleRandomValues(std::vector<float> &x)
    {
        float sum = histogramm.at(histogramm.size()-1);

        if (sum < .99f || sum > 1.01f)
        {
            ROS_INFO("Sum of the gaussian weights not 1. Please normailze the weight of the distibutions!");
            return;
        }

        // Get one vector of samples
        // The models are sampled with the stored weights
        // Get xi € [0,1)
        float xi = gen_random_float(0,1);

        unsigned int index = 0;

        // find the index in the histogeramm at which data > xi
        while (histogramm.at(index) < xi)
        {
            index++;
        }

        // Decrease index by one to get the last element which is smaller than xi
        index--;

        //ROS_INFO("Sampling from model at(%i), xi=%.2f", index, xi);

        if(index > models.size()) index = models.size();

        // Now sample from the gaussian at counter
        models.at(index)->sampleRandomValues(x);
    }

    /**
      * Returns the model at the given index
      * @param index - the index of the model
      * @return the model at the given index
      */
    ASR::MultidimensionalGaussianPtr getModel(unsigned int index)
    {
        ROS_ASSERT(index < models.size());
        return models.at(index);
    }
};

typedef boost::shared_ptr<GMM> GMM_Ptr;

}
