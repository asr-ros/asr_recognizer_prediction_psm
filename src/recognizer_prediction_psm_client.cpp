/**

Copyright (c) 2016, Braun Kai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ros/ros.h"
#include "asr_recognizer_prediction_psm/recognizer_prediction_psm.h"
#include <cstdlib>
#include <asr_msgs/AsrObject.h>

#include <ros/package.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "recognizer_prediction_psm_client");
    if (argc != 2)
    {
        ROS_INFO("usage: recognizer_prediction_psm_client integer");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<asr_recognizer_prediction_psm::recognizer_prediction_psm>("recognizer_prediction_psm");
    asr_recognizer_prediction_psm::recognizer_prediction_psm srv;


    /**
     * Adding some test data
     */
    std::string path = ros::package::getPath("asr_recognizer_prediction_psm") + "/models/dome_scene1.xml";
    std::vector<std::string> scenes;
    scenes.push_back("background");
    scenes.push_back("marker_scene1");
    scenes.push_back("marker_scene2");

    std::vector<float> probs;
    probs.push_back(0.25f);
    probs.push_back(0.5f);
    probs.push_back(0.25);
    std::vector<asr_msgs::AsrObject> objects;

    // Test object
    {
        geometry_msgs::Point point;
        point.x = 0.31f;
        point.y = 0.35f;
        point.z = 1.37f;

        geometry_msgs::Quaternion q;
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;

        //Quickfix for new AsrObject format
        asr_msgs::AsrObject pObj;
        geometry_msgs::PoseWithCovariance newPose;
        pObj.type = "VitalisSchoko";
        pObj.identifier = "/map";
        pObj.header.frame_id = "/map";
        newPose.pose.position = point;
        newPose.pose.orientation = q;
        pObj.sampledPoses.push_back(newPose);
        objects.push_back(pObj);


        pObj.sampledPoses.clear();
        pObj.type = "Cup";
        pObj.identifier = "/map";
        pObj.header.frame_id = "/map";

        newPose.pose.position = point;
        newPose.pose.orientation = q;

        //pObj.sampledPoses.push_back(newPose);
        //objects.push_back(pObj);
    }


    /**
      * Set the parameter for the service call
      */
    srv.request.pathToXML = path;
    srv.request.scenes = scenes;
    srv.request.scene_probabilities = probs;
    srv.request.num_votes = atoll(argv[1]);
    srv.request.objects = objects;
    srv.request.base_frame_id = "/map";

    /**
      * Call the service call an collect the results
      */
    if (client.call(srv))
    {
        for(auto e : srv.response.pose_hypothesis.elements)
        {
            ROS_INFO("Hypothesis type \"%s\" at position (%.2f, %.2f, %.2f), orientation (%.2f, %.2f, %.2f, %.2f), base_frame \"%s\"",
                     e.type.c_str(),
                     e.pose.position.x,
                     e.pose.position.y,
                     e.pose.position.z,
                     e.pose.orientation.w,
                     e.pose.orientation.x,
                     e.pose.orientation.y,
                     e.pose.orientation.z,
                     srv.request.base_frame_id.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service recognizer_prediction_psm");
        return 1;
    }

    return 0;
}
