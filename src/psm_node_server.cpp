/**

Copyright (c) 2016, Braun Kai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <asr_msgs/AsrAttributedPointCloud.h>
#include <asr_msgs/AsrObject.h>
#include <inference/SceneInferenceEngine.h>
#include <inference/model/SceneIdentifier.h>
#include <asr_recognizer_prediction_psm/recognizer_prediction_psm.h>
#include <asr_recognizer_prediction_psm/psm_node.h>


std::vector<ProbabilisticSceneRecognition::SceneIdentifier> doPSMInference(asr_recognizer_prediction_psm::psm_node::Request  &req)
{
    std::string base_frame_id = "/map";
  ros::NodeHandle nh;
  nh.getParam("/psm_node/base_frame_id", base_frame_id);
  nh.setParam("/js_probabilistic_scene_inference_engine/base_frame_id", base_frame_id);

  std::vector<asr_msgs::AsrObject> objects = req.objects;
  std::vector<boost::shared_ptr<asr_msgs::AsrObject>> object_pointers;
  
  // convert the object msgs
  for(asr_msgs::AsrObject o : objects)
  {
    boost::shared_ptr<asr_msgs::AsrObject> p = boost::shared_ptr<asr_msgs::AsrObject>(new asr_msgs::AsrObject());
    p->type = o.type;
    p->identifier = o.identifier;
    p->header.frame_id = o.header.frame_id;
    if(!o.sampledPoses.size()){
      std::cerr << "Got a AsrObject without poses." << std::endl;
      std::exit(1);    
    }
    p->sampledPoses.push_back(o.sampledPoses.front());
    object_pointers.push_back(p);

  }
        
        





  /**
   * Create and execute inference engine.
   */
  ProbabilisticSceneRecognition::SceneInferenceEngine ie;
  
  // add the objects of the service call to the evidence buffer
  for(boost::shared_ptr<asr_msgs::AsrObject> o : object_pointers)
  {
    ie.newObservationCallback(o);
  }

  // update the model and calculate the probabilities
  ie.update();
  
  /**
   * get the probabilities
   */
  std::vector<ProbabilisticSceneRecognition::SceneIdentifier> pSceneList;
  ie.getModel().getSceneListWithProbabilities(pSceneList);
  







  printf("\nThis are the scene probabilities:\n");
    for(ProbabilisticSceneRecognition::SceneIdentifier i : pSceneList)
      printf(" -> %s (%s): %f (%f)\n", i.mDescription.c_str(), i.mType.c_str(), i.mLikelihood, i.mPriori);
  printf("\n");
  
  return pSceneList;
}

void doPrediction(std::vector<ProbabilisticSceneRecognition::SceneIdentifier> &pSceneList,
                  asr_recognizer_prediction_psm::psm_node::Request  &req,
                  asr_recognizer_prediction_psm::psm_node::Response &res)
{
    ros::NodeHandle n;
    bool vis = false;

    n.getParam("/psm_node/visualize_hypothesis", vis);
    n.setParam("/recognizer_prediction_psm/visualize_hypothesis", vis);

    ros::ServiceClient client = n.serviceClient<asr_recognizer_prediction_psm::recognizer_prediction_psm>("recognizer_prediction_psm");
    asr_recognizer_prediction_psm::recognizer_prediction_psm srv;



    /**
      * Set the parameter for the service call
      */
    std::string path;
    int votes;
    std::string base_frame_id;
    n.getParam("/js_probabilistic_scene_inference_engine/scene_model_filename", path);
    n.getParam("/psm_node/num_hypothesis", votes);
    n.getParam("/psm_node/base_frame_id", base_frame_id);
    srv.request.num_votes = votes;
    srv.request.objects = req.objects;
    srv.request.base_frame_id = base_frame_id;
    srv.request.pathToXML = path;

    std::vector<std::string> scenes;
    std::vector<float> scene_probabilities;

    for(ProbabilisticSceneRecognition::SceneIdentifier i : pSceneList)
    {
        scenes.push_back(i.mDescription.c_str());
        scene_probabilities.push_back(i.mLikelihood);
    }

    srv.request.scenes = scenes;
    srv.request.scene_probabilities = scene_probabilities;


    /**
      * Call the service call an collect the results
      */
    if (client.call(srv))
    {
        /*
        for(auto e : srv.response.pose_hypothesis.elements)
        {
            ROS_INFO("Hypothesis type \"%s\" at position (%.2f, %.2f, %.2f), orientation (%.2f, %.2f, %.2f, %.2f), base_frame \"%s\"",
                     e.object_type.c_str(),
                     e.pose.position.x,
                     e.pose.position.y,
                     e.pose.position.z,
                     e.pose.orientation.w,
                     e.pose.orientation.x,
                     e.pose.orientation.y,
                     e.pose.orientation.z,
                     srv.request.base_frame_id.c_str());
        }*/

        /**
          * Set the response msg
          */
        res.pose_hypothesis = srv.response.pose_hypothesis;
        res.all_scene_objects_found = srv.response.all_scene_objects_found;
    }
    else
    {
        ROS_ERROR("Failed to call service recognizer_prediction_psm");
        return;
    }

    return;


}


bool run(asr_recognizer_prediction_psm::psm_node::Request  &req,
         asr_recognizer_prediction_psm::psm_node::Response &res)
{
    // The recognition result
    std::vector<ProbabilisticSceneRecognition::SceneIdentifier> pSceneList;

    // do the scene recognition
    pSceneList = doPSMInference(req);

    // do the prediction
    doPrediction(pSceneList, req, res);


    return true;
}

bool getNumberOfUnobservedObjects()
{

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "psm_node_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("psm_node", run);
    ROS_INFO("Ready to do the inference\n");


    ros::spin();

    return 0;
}
