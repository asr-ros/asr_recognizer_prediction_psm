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
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include "Gaussian.h"

using namespace Eigen;
using namespace std;

namespace ASR
{

/**
  This contains only one Gaussian.
  */

    class MultidimensionalGaussian
    {
    private:
        std::vector<float> means;
        MatrixXf covariance;
        MatrixXf L;
        GaussianPtr normal_dist;
        int dimension;

    public:
        /**
         * Creates a multidimensional Gaussian-Kernel with the given parameters.
         * The dimension of the means vector has to be the same as the covariance matrix.
         * @param means - the means of the gaussian kernels
         * @param covariance - the covariance matrix of the gaussian kernel
         */
        MultidimensionalGaussian(std::vector<float> means, MatrixXf covariance)
        {
            this->means = means;
            this->covariance = covariance;
            this->dimension = means.size();

            normal_dist = GaussianPtr(new ASR::Gaussian(0,1));

            // Calculate Cholesky decomposition
            LLT<MatrixXf> lltOfA(covariance); // compute the Cholesky decomposition of A
            this->L = lltOfA.matrixL(); // retrieve factor L  in the decomposition

            // For debugging only
            //test();
        }

        /**
         * Samples a random vector from the gmm and stores them in a given vector.
         * There are sampled dimension of means (given in the constructor) values.
         * @param x - the output vector that gains the generated random values.
         */
        void sampleRandomValues(std::vector<float> &x)
        {
            //ROS_ASSERT(x != nullptr);

            // 1. A = L * L.transpose()
            // 2. Erzeuge N-dim Vector mit standartnormalverteilten Zufallszahlen
            std::vector<float> z;
            normal_dist->sampleRandomValues(z, dimension);
            // 3. transform x = means + L * random
            VectorXf Z = initVectorXf(z);
            VectorXf mu = initVectorXf(means);
            VectorXf X = mu + L * Z;

            // Get the valus of the VectorXf and convert them to a std::vector
            float *t1 = X.data();
            for(int i=0; i < dimension; i++)
            {
                x.push_back(*(t1 + i));
            }
        }

        /**
          * Returns the means of the gaussian.
          * @return the means of the gaussian
          */
        std::vector<float> getMean() {return means;}

    private:
        /**
          * Inits a Eigen::VectorXf with the given values.
          * @Param a list with the values
          * @Return the Eigen::VectorXf with the given values and dimension
          */
        VectorXf initVectorXf(std::vector<float> values)
        {
            VectorXf x(values.size());
            for(unsigned int i=0; i<values.size();i++)
            {
                x(i) = values.at(i);
            }
            return x;
        }


        /**
          * Inits a Eigen::MatrixXf with the given values.
          * @Param a list with the values
          * @Return the Eigen::VectorXf with the given values and dimension
          */
        MatrixXf initMatrixXf(std::vector<float> values, unsigned int rows, unsigned int cols)
        {
            assert (rows > 0);
            assert (cols > 0);

            MatrixXf x(rows, cols);
            for(unsigned int i=0; i < rows;i++)
            {
                for(unsigned int j=0; j < cols; j++)
                {
                    x(i,j) = values.at(i*cols + j);
                }
            }
            return x;
        }

        /**
          * Inits a vector<float> with the given values.
          * The given values have to be COMMA SEPERATED e.g. 1,2,3,4
          * @param a csv string with the values
          * @Return the std::vector<float> with the given values and dimension
          */
        void initVectorFromCSVString(std::vector<float> &x, std::string csv)
        {
            std::stringstream ss(csv);
            float i;
            while (ss >> i)
            {
                x.push_back(i);

                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
        }


        /**
          * Tests the Cholesky decomposition.
          */
        void test()
        {
            cout << "The means are " << endl << initVectorXf(this->means) << endl;
            cout << "The matrix A is" << endl << covariance << endl;
            cout << "The Cholesky factor L is" << endl << L << endl;
            cout << "To check this, let us compute L * L.transpose()" << endl;
            cout << L * L.transpose() << endl;
            cout << "This should equal the matrix A" << endl;
        }

    };

    typedef boost::shared_ptr<MultidimensionalGaussian> MultidimensionalGaussianPtr;

}
