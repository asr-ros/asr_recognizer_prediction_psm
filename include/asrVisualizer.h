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

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include "gnuplot-iostream.h"

#include <string>
#include <set>
#include <utility>

#include <Eigen/Core>
#include "AttributedPoint.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ASR
{

    class asrVisualizer
    {
    public:
        asrVisualizer(int max_number_of_published_vis_msgs = 100);

        /**
          Initializes the GnuPlot.
          @param pPlotTitle - the name of the plot
          @param pxLabel - label of the x-axis
          @param pyLabel - label of the y-axis
          @param pXRange - x-axis range (min, max)
          @param pYRange - y-axis range (min, max)
          @param pDelta - scale of the helper lines
          */
        void initAnimatedPlot(const std::string& pPlotTitle,
                              const std::string& pXLabel,
                              const std::string& pYLabel,
                              const std::pair<float, float>& pXRange,
                              const std::pair<float, float>& pYRange,
                              const std::pair<float, float>& pDelta);



        /**
          Sends the data to the gnuplot and draws the window
          which contains the plot.
          */
        void sendPlotToGnuplot();

        /**
          Adds a new point to the list of found points
          */
        void addPointToFoundBuffer(float x, float y) { addPointToFoundBuffer(Eigen::Vector2f(x,y)); }
        void addPointToFoundBuffer(Eigen::Vector2f point);

        /**
          * Sends visualization messages to rviz.
          * The two lists "hypothesis" and "found_poses" are publish as
          * VISUALIZATION::MSG::MARKER under the topic "pose_predictor/hypothesis3d_markers"
          * This topic is defined in the launch file.
          * Found objects are marked as blue cubes, hypothesis are marked as red spheres.
          * @param hypotheses - the list of hypotheses that should be visualized
          * @param found_poses - the list ob observed objects that should be visualized
          * @param target frame - the frame to which the markers should be mapped to
          * @param id - the id of the markers
          */
        void publishRVizMarker(std::vector<AttributedPoint> hypotheses, std::vector<AttributedPoint> found_poses,
                           const std::string target_frame = "/map", const int id = 1);


        /**
          Adds a new point to the list of unfound points
          */
        void addPointToUnfoundBuffer(Eigen::Vector2f point);

        /**
          Adds two lists of AttributedPoints to the list of unfound points and found points.
          Attributed Points contain a full pose and other additional information.
          @param hypothese - the list that is stored to unfound points
          @param found_poses - the list that is stored to the found points
          */
        void addAttributedPoints(std::vector<AttributedPoint> hypotheses, std::vector<AttributedPoint> found_poses);


        /**
         * Generates a single MarkerArray RVIZ Message for visualization. The hypothesis and found objects
         * are visualized as 3D Objects.
         * @param hypotheses - the list of hypothesis
         * @param found_poses - the list of found objects
         */
        void generateMarkerArray(std::vector<AttributedPoint> hypotheses,
                                 std::vector<AttributedPoint> found_poses);

        /**
         * Publishes the MarkerArray Message if it is not empty.
         * The topic is "/recognizer_prediction_psm/visualization_marker_array"
         */
        void publishMarkerArray();


    private:
        ros::Publisher publisher;
        visualization_msgs::MarkerArray markerArray;

        double marker_lifetime;

        //Interface with which configurations or data is sent to gnuplot.
        boost::shared_ptr<Gnuplot> mGnuplotHandler;

        // the publisher to publish marker msgs to rviz
        ros::Publisher hypothesis3dPublisher;

        //buffers for visualization points
        std::vector<std::pair<double, double> > mUnfoundBuffer;
        std::vector<std::pair<double, double> > mFoundBuffer;
        std::vector<std::pair<float, float> > mVisitedViews;

    };


    typedef boost::shared_ptr<asrVisualizer> asrVisualizerPtr;

}
