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
#include <rapidxml.hpp>
#include <fstream>
#include <cstddef>

#include "OcmSceneGraphNode.h"
#include "GMM.h"
#include <string>
#include <helper/ObjectTransformation.h>
#include <asr_msgs/AsrObject.h>
#include <asr_msgs/AsrAttributedPoint.h>

namespace ASR
{


using namespace rapidxml;
using namespace std;


/**
  This class provides a interface to build a psm scene graph and to generate
  hypothesis for the poses of the unobserved objects in the scene.
  Observed objects can be marked as explored and the scene graph and the hypothesis
  are then updated.
  */
class asrPosePredictionEngine
{
private:

    // path to xml file
    std::string path;
    // name of the reference object
    std::string reference_object;
    // the name of the current scene
    std::string scene_name;

    // the node of the reference object
    ASR::asrSceneGraphNodePtr root;

    int number_obsereved_objects;
    int number_objects_in_scene;
    int number_of_votes;                // overall available votes
    int votes_per_node;                 // votes that are available to one unobserved node
    //std::string pPbdObjectTopic;        // The ROS topic of obsererved Objects to listen to.
    //ros::Subscriber mObjectListener;    // A callback handler listening to objects found by an object detection system.
    std::string baseFrameId;            // The frame to which the geometry:msgs are transformed into

    // A transformer for objects into the target coordinate frame.
    ProbabilisticSceneRecognition::ObjectTransformation mObjectTransform;


    bool publish_hypothesis;
    bool print_debug_messages;
    std::string publish_hypothesis_topic;   // the topic to which the hypotheses are published
    ros::Publisher hypothesis_publisher;    // the publisher that publishs the generated hypothesis
    std::vector<boost::shared_ptr<asr_msgs::AsrObject>> evidence_buffer; // A buffer that stores all observed objects that occured since the last hypotheses update

public:

    /**
      * Builds a scene graph which is described in the scene model xml file. The given objects are integreted into the scene graph.
      * @param pathToXML - the path to the scene model xml file. The model has to be formatted like the models from the probabilistic scene model.
      * @param sceneName - the name of the scene. Only hypothesis for this scene are generated.
      * @param objects - the list of the observed objects which should be integrated. The first object is the reference object.
      * @param votes - the overall number of hypothesis that should be generated. The amount of votes is distributed over all unobserved objects.
      * @param base_frame - the id of the base coordinate system all objects and hypothesis should be transformed to
      */
    asrPosePredictionEngine(std::string pathToXML,
                            std::string sceneName,
                            std::vector<boost::shared_ptr<asr_msgs::AsrObject>> objects,
                            int votes,
                            std::string base_frame);


    /**
      * Builds a scene graph which is described in the scene model xml file.
      * @param pathToXML - the path to the scene model xml file. The model has to be formatted like the models from the probabilistic scene model.
      * @param sceneName - the name of the scene. Only hypothesis for this scene are generated.
      * @param reference_object_name - the name of the reference object. This can be the first observed object.
      * @param reference_object_position - the position of the reference object. All generated hypotheses are ralative to
      * the position of the reference object.
      * @param votes - the overall number of hypothesis that should be generated. The amount of votes is distributed over all unobserved objects.
      * @param base_frame - the id of the base coordinate system all objects and hypothesis should be transformed to
      */
    asrPosePredictionEngine(std::string pathToXML,
                            std::string sceneName,
                            std::string reference_object_name,
                            Eigen::Vector3f reference_object_position,
                            Eigen::Vector4f reference_object_orientation,
                            int votes,
                            std::string base_frame);


    /**
      * Returns the number of unobserved objects in the scene.
      * @return number of unobserved objects in the scene
      */
    unsigned int getNumberOfMissingObjects() {return this->number_objects_in_scene - this->number_obsereved_objects;}


    /**
      * Returns the root node of the scene graph
      * @return the root node of the scene graph
      */
    ASR::asrSceneGraphNodePtr getSceneGraphRoot() {return root;}


    /**
      * Publishs the given points as
      * GEOMETRY::MSG::POSE under the in the launch file defined topic.
      * This standart topic name is "pose_prediction_psm_hypothesis"
      * @param hypothesis_list - the list of AttributedPoints that should be published.
      */
    void publishHypothesisMessages(std::vector<ASR::AttributedPoint> hypothesis_list);


    /**
      * Collects all pose hypotheses that are generated previously via @code{ calcHypotheses() }
      * @param out_hypotheses the list the hypotheses are stored into.
      * @param out_found_objects the list to which the poses of all observed objects are stored into.
      */
    void collectPoseHypothesesRecursive(std::vector<ASR::AttributedPoint> &out_hypotheses,
                                        std::vector<ASR::AttributedPoint> &out_found_objects);


    /**
      * Calculates n hypotheses for the whole scene graph.
      * all n votes (given in the launch file) are distributed over the unobserved objects.
      * The objects are transformed into the base_frame_id fram (/PTU)
      */
    void calcHypotheses();

private:

    /**
      * Calculates the number of votes (number of hypothesis) which are available to each unobserved object node.
      * @return the number of votes available to each unobserved object node.
      */
    unsigned int distributeVotes();

    /**
      * Call this method if a object was observed at the given position.
      * If an object with the given name exists in the scene and was not observed previosly
      * the node of the given object is marked as observed.
      * Then the number of hypothesis is updated.
      * @return @code{true} if the object was integrated into the scene graph,
      * @code{false} if the observed object doesnt exist in the scene or it was already observed.
      */
    bool observeObject(std::string object_name, Eigen::Vector3f position, Eigen::Vector4f orientation);

    /**
      * Called when a new object was observed.
      * Pushs the object to a buffer to keep the callback as short as possible.
      */
    void onObjectMsg(const boost::shared_ptr<asr_msgs::AsrObject>& pObject);


    /**
      * Transformes a given AsrObject to another frame.
      * If the tranformation fails the object is not changed and false is returned.
      * @param in_out_object - the object that should be transformed. The transformed object is stored at that pointer
      * @param targetFrame - the name of the frame the object should be transformed into
      * @return true if the transformation was successful, false otherwise
      */
    bool transformObject(boost::shared_ptr<asr_msgs::AsrObject> in_out_object, std::string targetFrame);


    /**
      * Loads the xml file and builds the scene graph
      */
    void init(std::string sceneName,
              std::string reference_object_name,
              Eigen::Vector3f reference_object_position,
              Eigen::Vector4f reference_object_orientation,
              std::string base_frame);


    /**
      * Builds the intern tree recursivly
      */
    void traverseSceneShapeRecursive(std::string parent_name, xml_node<> * parent_node, ASR::asrSceneGraphNodePtr root);


    /**
      * Inits a vector<float> with the given values.
      * The given values have to be COMMA SEPERATED e.g. 1,2,3,4
      * @param a csv string with the values
      * @Return the std::vector<float> with the given values and dimension
      */
    void initVectorFromCSVString(std::vector<float> &x, std::string csv);


    /**
      * Inits a Eigen::MatrixXf with the given values.
      * @Param a list with the values
      * @Return the Eigen::VectorXf with the given values and dimension
      */
    MatrixXf initMatrixXf(std::vector<float> values, unsigned int rows, unsigned int cols);
};

typedef boost::shared_ptr<asrPosePredictionEngine> asrPosePredictionEnginePtr;

}











