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
#include <sensor_msgs/PointCloud2.h>
#include <asr_msgs/AsrAttributedPointCloud.h>
#include <asr_msgs/AsrObject.h>
#include "asrPosePredictionEngine.h"
#include "AttributedPoint.h"
#include "asrVisualizer.h"



/**
 * This method generates the hypothesis.
 * @brief generate_hyps
 * @param req - The paramters for the service call
     * string path                          - input: the path to the xml file that contains the scenes (eg. models/breakfast.xml)
     * string[] scenes                      - input: the name of the scenes. It gerneates hypothesis only for these scenes.
     * float32[] scene_probabilities 		- input: contains the probability of the scene
     * uint32 num_votes 					- input: the overall number of hypothesis that should be generated
     * asr_msgs/AsrObject[] objects			- input: the list of all oberved objects
     * string base_frame_id					- input: the base frame to which the observed objects and the hypothesis
 * @param res - the response of the service call
     * next_best_view/AttributedPointCloud pose_hypothesis	- output: the generated hypothesis
 * @return the generated hypothesis
 */
bool generate_hyps(asr_recognizer_prediction_psm::recognizer_prediction_psm::Request  &req,
                   asr_recognizer_prediction_psm::recognizer_prediction_psm::Response &res)
{
    ros::NodeHandle n;
    bool visualize_hypothesis = true;

    if(!n.getParam("/recognizer_prediction_psm/visualize_hypothesis", visualize_hypothesis))
        visualize_hypothesis = true;

    // Load all data
    std::string path = req.pathToXML;
    std::vector<std::string> scene_list = req.scenes;
    std::vector<float> scene_probabilities = req.scene_probabilities;
    std::vector<int> votes_per_scene;
    uint32_t num_votes = req.num_votes;
    std::vector<asr_msgs::AsrObject> objects = req.objects;
    std::vector<boost::shared_ptr<asr_msgs::AsrObject>> object_pointers;

    // Some sanity checks
    {
        if(scene_list.size() != scene_probabilities.size())
        {
            ROS_ERROR("The number of scenes has to be equal to the number of scene probabilities!");
            return false;
        }

        if(objects.size() < 1)
        {
            ROS_ERROR("There has to be at least one observed object!");
            return false;
        }

        ROS_INFO("Overall number of hypothesis that should be generated: %i", num_votes);

        for(unsigned int i=0;i<scene_list.size();i++)
        {
            votes_per_scene.push_back((int) (scene_probabilities.at(i) * num_votes));

            ROS_INFO("Loaded scene %s with probability %.2f. Number of hypothesis for this scene %i",
                     scene_list.at(i).c_str(),
                     scene_probabilities.at(i),
                     votes_per_scene.at(i));
        }

        // Convert the objects msgs in the correct format
        for(asr_msgs::AsrObject o : objects)
        {

	  if(!o.sampledPoses.size()){
        std::cerr << "Got a AsrObject without poses." << std::endl;
	    std::exit(1);    
	  }

      boost::shared_ptr<asr_msgs::AsrObject> p = boost::shared_ptr<asr_msgs::AsrObject>(new asr_msgs::AsrObject());
	  p->type = o.type;
	  p->identifier = o.identifier;
	  p->header.frame_id = o.header.frame_id;
	  p->sampledPoses.push_back(o.sampledPoses.at(0));
	  object_pointers.push_back(p);

        }
    }
    // / Sanity checks





    // Now start the pose prediction engine with the given data
    std::vector<ASR::AttributedPoint> pose_hypotheses;
    std::vector<ASR::AttributedPoint> found_objects;
    bool all_scene_objects_found = true;

    printf("\n\n");

    for(unsigned int i=0;i<scene_list.size();i++)
    {
        ROS_INFO("\nPath: %s\nCurrent Scene: %s\n", path.c_str(), scene_list.at(i).c_str());

        /**
          * Load the scene and integrate the observed objects
          */
        ASR::asrPosePredictionEnginePtr engine = boost::shared_ptr<ASR::asrPosePredictionEngine>(
                    new ASR::asrPosePredictionEngine(path,
                                                     scene_list.at(i),
                                                     object_pointers,
                                                     votes_per_scene.at(i),
                                                     req.base_frame_id)
                                                     );

        /**
          * Compute all hypothesis
          */
        engine->calcHypotheses();


        /**
          * Collect all generated hypothesis and observed objects positions
          */
        engine->collectPoseHypothesesRecursive(pose_hypotheses, found_objects);


        /**
          * Check if all objects are found already.
          * The pose prediction finished when all objects are found
          */
        if(engine->getNumberOfMissingObjects() > 0 && scene_list.at(i).compare("background") != 0)
            all_scene_objects_found = false;
    }






    /**
     * Generate the result msg
     */
    asr_msgs::AsrAttributedPointCloud msg;

    for(unsigned int i=0; i<pose_hypotheses.size();i++)
    {
        asr_msgs::AsrAttributedPoint element;

        element.type = pose_hypotheses.at(i).objectType;
        element.pose = pose_hypotheses.at(i).pose;

        msg.elements.push_back(element);
    }
    res.pose_hypothesis = msg;
    res.all_scene_objects_found = all_scene_objects_found;

    printf("\n\n");


    /**
     * Visualize the response
     */
    if(visualize_hypothesis)
    {
        ASR::asrVisualizerPtr vis = boost::shared_ptr<ASR::asrVisualizer>(new ASR::asrVisualizer(num_votes));
        vis->generateMarkerArray(pose_hypotheses, found_objects );


        for(int i=0;i<5;i++) {
            ROS_INFO("Publishing MarkerArray");
            vis->publishMarkerArray();
            sleep(1);
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_hypothesis_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("recognizer_prediction_psm", generate_hyps);
    ROS_INFO("Ready to generate hypothesis\n");


    ros::spin();

    return 0;
}
