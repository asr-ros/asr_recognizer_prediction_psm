/**

Copyright (c) 2016, Braun Kai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "asrVisualizer.h"
#include <Eigen/Geometry>

namespace ASR
{

asrVisualizer::asrVisualizer(int max_number_of_published_vis_msgs)
{
    ros::NodeHandle n("~");
    hypothesis3dPublisher = n.advertise<visualization_msgs::Marker>("hypothesis3d_markers", max_number_of_published_vis_msgs);
    publisher = n.advertise<visualization_msgs::MarkerArray>("/recognizer_prediction_psm/visualization_marker_array", 1);

    if(!n.getParam("/recognizer_prediction_psm/marker_lifetime", marker_lifetime))
        marker_lifetime = 20;
}

/**
  Adds a new point to the list of found points
  */
void asrVisualizer::addPointToFoundBuffer(Eigen::Vector2f point)
{
    mFoundBuffer.push_back(std::make_pair(point.x(), point.y()));
}

/**
  Adds a new point to the list of unfound points
  */
void asrVisualizer::addPointToUnfoundBuffer(Eigen::Vector2f point)
{
    mUnfoundBuffer.push_back(std::make_pair(point.x(), point.y()));
}

/**
  Adds two lists of AttributedPoints to the list of unfound points and found points.
  Attributet Points contain a full pose and other additional information.
  @param hypothese - the list that is stored to unfound points
  @param found_poses - the list that is stored to the found points
  */
void asrVisualizer::addAttributedPoints(std::vector<AttributedPoint> hypotheses, std::vector<AttributedPoint> found_poses)
{
    for(AttributedPoint p : hypotheses)
    {
        geometry_msgs::Pose pose = p.pose;
        mUnfoundBuffer.push_back(std::make_pair(pose.position.x, pose.position.y));
    }

    for(AttributedPoint p : found_poses)
    {
        geometry_msgs::Pose pose = p.pose;
        mFoundBuffer.push_back(std::make_pair(pose.position.x, pose.position.y));
    }
}


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
void asrVisualizer::publishRVizMarker(std::vector<AttributedPoint> hypotheses, std::vector<AttributedPoint> found_poses,
                                      const std::string target_frame, const int id)
{
    float scale_hyp = 0.1f;
    float scale_found = 0.2f;

    visualization_msgs::Marker sample3dList;
    sample3dList.header.frame_id = target_frame;
    sample3dList.header.stamp = ros::Time::now();
    sample3dList.ns = "hypothesis";
    sample3dList.action = visualization_msgs::Marker::ADD;
    sample3dList.type = visualization_msgs::Marker::SPHERE_LIST;
    sample3dList.lifetime = ros::Duration(1.0f);
    sample3dList.id = id;

    sample3dList.scale.x = scale_hyp;
    sample3dList.scale.y = scale_hyp;
    sample3dList.scale.z = scale_hyp;
    sample3dList.color.r = 1;
    sample3dList.color.g = 0;
    sample3dList.color.b = 0;
    sample3dList.color.a = 1.0;

    for(AttributedPoint hyp : hypotheses)
    {
        sample3dList.points.push_back(hyp.pose.position);
    }

    visualization_msgs::Marker found3dList;
    found3dList.header.frame_id = target_frame;
    found3dList.header.stamp = ros::Time::now();
    found3dList.ns = "observed_objects";
    found3dList.action = visualization_msgs::Marker::ADD;
    found3dList.type = visualization_msgs::Marker::CUBE_LIST;
    found3dList.lifetime = ros::Duration(1.0f);
    found3dList.id = id + 1;

    found3dList.scale.x = scale_found;
    found3dList.scale.y = scale_found;
    found3dList.scale.z = scale_found;
    found3dList.color.r = 0;
    found3dList.color.g = 0;
    found3dList.color.b = 1;
    found3dList.color.a = 1.0;

    for(AttributedPoint p : found_poses)
    {
        found3dList.points.push_back(p.pose.position);
    }

    if(!sample3dList.points.empty()) hypothesis3dPublisher.publish(sample3dList);
    if(!found3dList.points.empty()) hypothesis3dPublisher.publish(found3dList);
}

/**
  Initializes the GnuPlot.
  @param pPlotTitle - the name of the plot
  @param pxLabel - label of the x-axis
  @param pyLabel - label of the y-axis
  @param pXRange - x-axis range (min, max)
  @param pYRange - y-axis range (min, max)
  @param pDelta - scale of the helper lines
  */
void asrVisualizer::initAnimatedPlot(const std::string& pPlotTitle,
                                     const std::string& pXLabel, const std::string& pYLabel,
                                     const std::pair<float, float>& pXRange, const std::pair<float, float>& pYRange,
                                     const std::pair<float, float>& pDelta) {

    //Create a clean interface to gnuplot.
    mGnuplotHandler.reset(new Gnuplot);

    //Empty buffers with data for gnuplot.
    //mUnfoundBuffer.clear();
    //mFoundBuffer.clear();

    //Set bar chart title
    *(mGnuplotHandler) << "set title \"" << pPlotTitle << "\"\n";

    //Unit length in plot of both x and y axis are equal.
    *(mGnuplotHandler) << "set size ratio -1\n";

    //Style for points
    *(mGnuplotHandler) << "set border linewidth 0.5\n";
    *(mGnuplotHandler) << "set pointsize .2\n";
    *(mGnuplotHandler) << "set style line 1 lc rgb '#00008b' pt 5\n";
    *(mGnuplotHandler) << "set style line 2 lc rgb '#00ced1' pt 7\n";

    //Set labels for axes
    *(mGnuplotHandler) << "set xlabel \"" << pXLabel << "\"\n";
    *(mGnuplotHandler) << "set ylabel \"" << pYLabel << "\"\n";

    //Set range in both x and y direction
    *(mGnuplotHandler) << "set xrange [" << pXRange.second <<  ":" << pXRange.first << "]\n";
    *(mGnuplotHandler) << "set yrange [" << pYRange.first <<  ":" << pYRange.second << "]\n";

    //Ask for a grid for highres ticks
    *(mGnuplotHandler) << "set grid x2tics lc rgb \"#bbbbbb\"\n";
    *(mGnuplotHandler) << "set grid y2tics lc rgb \"#bbbbbb\"\n";

    //Lowres ticks
    *(mGnuplotHandler) << "set xtics " << pXRange.first << "," << pDelta.first * 4 << "," << pXRange.second <<"rotate\n";
    *(mGnuplotHandler) << "set ytics " << pYRange.first << "," << pDelta.second * 4 << "," << pYRange.second <<"\n";

    //Highres tics
    *(mGnuplotHandler) << "set x2tics " << pXRange.first << "," << pDelta.first << "," << pXRange.second <<" scale 0\n";
    *(mGnuplotHandler) << "set y2tics " << pYRange.first << "," << pDelta.second << "," << pYRange.second <<" scale 0\n";

    //Highres ticks with labels please
    *(mGnuplotHandler) << "set format y2 \"\"\n";
    *(mGnuplotHandler) << "set format x2 \"\"\n";
}


/**
  Sends the data to the gnuplot and draws the window
  which contains the plot.
  */
void asrVisualizer::sendPlotToGnuplot()
{
    ROS_ASSERT(mGnuplotHandler);
    //Prevent system from trying to send data to non-initialized gnuplot handler
    if(!mGnuplotHandler)
      throw std::runtime_error("Cannot show non-existing gnuplot visualization.");
    
    //*(mGnuplotHandler) << "set object 1 ellipse center 4.,2. size 2.,1.\n";


    //Every data combination has in common it wants to be plotted.
    *(mGnuplotHandler) << "plot ";

    //This should be later called in a loop with type and id as parameters
    *(mGnuplotHandler) << "'-' with points ls 1 title \"Poses\"";
    //This should be later called in a loop with type and id as parameters
    *(mGnuplotHandler) << ", '-' with points ls 2 title \"Object Hypotheses\"";
    //if(mVisitedViews.size() > 0) *(mGnuplotHandler) << ", '-' with linespoints ls 3 title \"Camera Movement\" ";

    //End of gnuplot instructions for defining how bars are to be plotted
    *(mGnuplotHandler) << "\n";

    //Later I should call .send(mUnfoundBuffer), on the gnuplothandler handler it returns, as often as the number of objects that I search for.

    //Push bar chart data to gnuplot.
    mGnuplotHandler->send(mFoundBuffer).send(mUnfoundBuffer);
    //if(mVisitedViews.size() > 0) mGnuplotHandler->send(mVisitedViews);
    mGnuplotHandler->flush();
}


/**
 * Generates a MarkerArray for visualization with RViz.
 * The hypotheses and the found objects are visualized as 3D models.
 * @brief asrVisualizer::generateMarkerArray generates a MarkerArray for RViz visualization
 * @param hypotheses the pose hypothesis that are generated
 * @param found_poses the found objects
 */
void asrVisualizer::generateMarkerArray(std::vector<AttributedPoint> hypotheses,
                                        std::vector<AttributedPoint> found_poses)

{
    int i = 0;
/*
    // Standart markers (points)
    for (ASR::AttributedPoint p : hypotheses)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "\map";
        marker.type = marker.SPHERE;
        marker.action = marker.ADD;
        marker.id = i;
        marker.lifetime = ros::Duration(marker_lifetime);
        marker.ns = p.objectType;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 0.7;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        if(p.objectType.compare("Cup") == 0)
        {
            marker.color.r = 0.0;
            marker.color.g = 150/255.0;
            marker.color.b = 130/255.0;
        } else if(p.objectType.compare("CeylonTea") == 0)
        {
            marker.color.r = 252/255.0;
            marker.color.g = 229/255.0;
            marker.color.b = 0;
        } else if(p.objectType.compare("PlateDeep") == 0)
        {
            marker.color.r = 140/255.0;
            marker.color.g = 182/255.0;
            marker.color.b = 60/255.0;
        } else if(p.objectType.compare("Marker_1") == 0)
        {
            marker.color.r = 162/255.0;
            marker.color.g = 34/255.0;
            marker.color.b = 35/255.0;
        }

        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = p.pose.position.x;
        marker.pose.position.y = p.pose.position.y;
        marker.pose.position.z = p.pose.position.z;
        markerArray.markers.push_back(marker);
        i = i+1;
    }
*/

    // 3D Markers
    for (ASR::AttributedPoint p : hypotheses)
    {
        std::string object_path = "";

        if(p.objectType.compare("PlateDeep") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/PlateDeep/object.dae";
        else if(p.objectType.compare("Cup") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/Cup/object.dae";
        else if(p.objectType.compare("Smacks") == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae";
        else if(p.objectType.compare(("VitalisSchoko")) == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/VitalisSchoko/VitalisSchoko.dae";
        else if(p.objectType.compare("CeylonTea") == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/CeylonTea/CeylonTea.dae";
        else if(p.objectType.compare("Marker_1") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/SpoonBig/object_rotated.dae";

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = object_path;
        marker.mesh_use_embedded_materials = true;
        marker.action = marker.ADD;
        marker.id = i;
        marker.lifetime = ros::Duration(marker_lifetime);
        marker.ns = p.objectType;
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
        marker.pose.orientation.w = p.pose.orientation.w;
        marker.pose.orientation.x = p.pose.orientation.x;
        marker.pose.orientation.y = p.pose.orientation.y;
        marker.pose.orientation.z = p.pose.orientation.z;
        marker.pose.position.x = p.pose.position.x;
        marker.pose.position.y = p.pose.position.y;
        marker.pose.position.z = p.pose.position.z;

        markerArray.markers.push_back(marker);
        i = i+1;
    }

    for (ASR::AttributedPoint p : found_poses)
    {
        std::string object_path = "";

        if(p.objectType.compare("PlateDeep") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/PlateDeep/object.dae";
        else if(p.objectType.compare("Cup") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/Cup/object.dae";
        else if(p.objectType.compare("Smacks") == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae";
        else if(p.objectType.compare(("VitalisSchoko")) == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/VitalisSchoko/VitalisSchoko.dae";
        else if(p.objectType.compare("CeylonTea") == 0)
            object_path = "package://asr_object_database/rsc/databases/textured_objects/CeylonTea/CeylonTea.dae";
        else if(p.objectType.compare("Marker_1") == 0)
            object_path = "package://asr_object_database/rsc/databases/segmentable_objects/SpoonBig/object_rotated.dae";

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = object_path;
        marker.mesh_use_embedded_materials = true;
        marker.action = marker.ADD;
        marker.id = i;
        marker.lifetime = ros::Duration(marker_lifetime);
        marker.ns = p.objectType;
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;
        marker.pose.orientation.w = p.pose.orientation.w;
        marker.pose.orientation.x = p.pose.orientation.x;
        marker.pose.orientation.y = p.pose.orientation.y;
        marker.pose.orientation.z = p.pose.orientation.z;
        marker.pose.position.x = p.pose.position.x;
        marker.pose.position.y = p.pose.position.y;
        marker.pose.position.z = p.pose.position.z;

        markerArray.markers.push_back(marker);
        i = i+1;
    }
}

/**
 * Publishs the MarkerArray that has been generated by {@code generateMarkerArray(..)}.
 * The topic is "/recognizer_prediction_psm".
 * If the MarkerArray is empty nothing is published.
 * @brief asrVisualizer::publishMarkerArray publishs the MarkerArray
 */
void asrVisualizer::publishMarkerArray()
{
    if(!markerArray.markers.empty()) publisher.publish(markerArray);
}

}
