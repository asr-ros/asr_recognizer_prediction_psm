/**

Copyright (c) 2016, Braun Kai, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "asrPosePredictionEngine.h"

namespace ASR
{

asrPosePredictionEngine::asrPosePredictionEngine(std::string pathToXML,
                                                 std::string sceneName,
                                                 std::vector<boost::shared_ptr<asr_msgs::AsrObject>> objects,
                                                 int votes = 100,
                                                 std::string base_frame = "/map")
      : path(pathToXML),
        reference_object(objects.at(0)->type.c_str()),
        number_obsereved_objects(1),
        number_of_votes(votes),
        publish_hypothesis(true),
        publish_hypothesis_topic("pose_prediction_psm_hypothesis")
{
  if (objects.size() < 1)
      ROS_ERROR("There has to be at least one reference object");

  // get the name of the base frame
  ros::NodeHandle n("~");

  if(!n.getParam("base_frame_id", baseFrameId))
      baseFrameId = base_frame;

  std::string reference_object_name;
  unsigned int counter = 0;

  do
  {
      boost::shared_ptr<asr_msgs::AsrObject> tempObject = objects.at(counter);

      if(!tempObject->sampledPoses.size()){
    std::cerr << "Got a AsrObject without poses." << std::endl;
	std::exit(1);    
      }

      geometry_msgs::PoseWithCovariance currentPose = tempObject->sampledPoses.front();

      ROS_INFO("\nStarted pose prediciton. Reference object name %s", tempObject->type.c_str());


      // Transform the reference object to the base_frame
      transformObject(tempObject, baseFrameId.c_str());

      Eigen::Vector3f position = Eigen::Vector3f(currentPose.pose.position.x,currentPose.pose.position.y, currentPose.pose.position.z);

      Eigen::Vector4f ori = Eigen::Vector4f(currentPose.pose.orientation.w,
					    currentPose.pose.orientation.x,
					    currentPose.pose.orientation.y,
					    currentPose.pose.orientation.z);


      // Load the scene descritiption xml and build the scene graph
      init(sceneName, tempObject->type.c_str(), position, ori, base_frame);

      reference_object_name = tempObject->type;

      counter++;
  } while(!getSceneGraphRoot() && counter < objects.size());


  // calculate initial hypothesis
  if(getSceneGraphRoot())
      getSceneGraphRoot()->calcChildPoseHypothesesRecursive();

  // integrate other observed objects
  for(unsigned int i=0;i<objects.size();i++)
  {
      if(reference_object_name.compare(objects.at(i)->type) != 0)
          onObjectMsg(objects.at(i));
  }
}


asrPosePredictionEngine::asrPosePredictionEngine(std::string pathToXML,
                                                 std::string sceneName,
                                                 std::string reference_object_name,
                                                 Eigen::Vector3f reference_object_position,
                                                 Eigen::Vector4f reference_object_orientation,
                                                 int votes = 100,
                                                 std::string base_frame = "/map")
    : path(pathToXML),
      reference_object(reference_object_name),
      number_obsereved_objects(1),
      number_of_votes(votes),
      publish_hypothesis(true),
      publish_hypothesis_topic("pose_prediction_psm_hypothesis")

{
  init( sceneName, reference_object_name, reference_object_position, reference_object_orientation, base_frame);
  if(getSceneGraphRoot())
      getSceneGraphRoot()->calcChildPoseHypothesesRecursive();
}


void asrPosePredictionEngine::init(std::string sceneName,
                                   std::string reference_object_name,
                                   Eigen::Vector3f reference_object_position,
                                   Eigen::Vector4f reference_object_orientation,
                                   std::string base_frame = "/map")
{
    ros::NodeHandle n("~");

    if(!n.getParam("publish_hypothesis", publish_hypothesis))
        publish_hypothesis = false;

    if(!n.getParam("publish_hypothesis_topic", publish_hypothesis_topic))
        publish_hypothesis_topic = "pose_prediction_psm_hypothesis";

    if(!n.getParam("print_debug_messages", print_debug_messages))
        print_debug_messages = false;

    if(!n.getParam("base_frame_id", baseFrameId))
        baseFrameId = base_frame;
/*
    if(!n.getParam("object_topic", pPbdObjectTopic))
         throw std::runtime_error("Please specify parameter " + std::string("object_topic") + " when starting this node.");
*/
    if(publish_hypothesis)
        hypothesis_publisher = n.advertise<asr_msgs::AsrAttributedPoint>(publish_hypothesis_topic, number_of_votes);
/*
    if(!print_debug_messages)
        ROS_INFO("-- Debug mode is disabled. Additional Messages are only printed to console if debug mode is enabled.\n");
*/

    // Tell node how to react on messages from objects that could belong to scenes being looked for.
    //printf("Object topic: %s\n", pPbdObjectTopic.c_str());
    //mObjectListener = n.subscribe(pPbdObjectTopic, 100, &asrPosePredictionEngine::newObservationCallback, this);

    // Initialize the transformations of objects into the given frame.
    mObjectTransform.setBaseFrame(baseFrameId);

    this->scene_name = sceneName;

    xml_document<> doc;
    xml_node<> * root_node;
    // Read the xml file into a vector

    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    std::string content(buffer.str());

    if(content.size() < 2)
    {
        ROS_ERROR("Loading XML failed");
        return;
    }

    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&content[0]);

    // Find our root node
    root_node = doc.first_node("psm");

    // Iterate over the psm attributes
    for (xml_node<> * brewery_node = root_node->first_node("scenes"); brewery_node; brewery_node = brewery_node->next_sibling())
    {

        // Interate over the scenes
        for(xml_node<> * scene_node = brewery_node->first_node("scene"); scene_node; scene_node = scene_node->next_sibling())
        {
            std::string name = scene_node->first_attribute("name")->value();

            if(print_debug_messages)
            {
                printf("Scene: %s, name: %s, priori %s \n",
                scene_node->first_attribute("type")->value(),
                name.c_str(),
                scene_node->first_attribute("priori")->value());
            }


            // Iterate over all scenes in the xml until the right scene is found
            if(name.compare(sceneName) == 0)
            {
                // // Interate over the objects
                for(xml_node<> * object_node = scene_node->first_node("object"); object_node; object_node = object_node->next_sibling())
                {

                    // If object.name == reference_object_name then start building the scene shape
                    std::string object_name = object_node->first_attribute("name")->value();

                    if(object_name.compare(reference_object_name) == 0)
                    {
                        if(print_debug_messages)
                        {
                            printf("\nLoaded reference object. name: %s, type: %s, priori %s \n",
                            object_node->first_attribute("name")->value(),
                            object_node->first_attribute("type")->value(),
                            object_node->first_attribute("priori")->value());
                        }

                        // get the number of objects in the scene
                        number_objects_in_scene = (int)atof(object_node->first_node("slots")->first_attribute("number")->value());
                        if(print_debug_messages) printf("Number of objects in the scene %i\n", number_objects_in_scene);
                        if(print_debug_messages) printf("Number of observed objects: %i\n", number_obsereved_objects);

                        // calculate the number of votes for each unobserved node
                        votes_per_node = distributeVotes();
                        //if(print_debug_messages)  printf("Votes per node %i \n", votes_per_node);

                        // start building scene shape beginning with the root
                        root = ASR::asrSceneGraphNodePtr(new ASR::OcmSceneGraphNode(object_name, reference_object_position,reference_object_orientation, votes_per_node, print_debug_messages));

                        // Interate over the object attributes
                        for(xml_node<> * object_attributes = object_node->first_node("shape"); object_attributes; object_attributes = object_attributes->next_sibling())
                        {
                            // Iterate over the shape term
                            for(xml_node<> * shape_attributes = object_attributes->first_node("root"); shape_attributes; shape_attributes = shape_attributes->next_sibling())
                            {


                                // Intialize the roots children
                                for(xml_node<> * root_children = shape_attributes->first_node("child"); root_children; root_children = root_children->next_sibling())
                                {
    /*
                                    if(print_debug_messages)  printf("Child_name: %s, parent_name: %s \n", root_children->first_attribute("name")->value(),
                                           "root");
    */
                                    ASR::GMM_Ptr gmm = boost::shared_ptr<ASR::GMM>(new ASR::GMM());


                                    // get position kernels
                                    for(xml_node<> * position_node = root_children->first_node("position"); position_node; position_node = position_node->next_sibling())
                                    {
                                        for(xml_node<> * kernel_node = position_node->first_node("kernel"); kernel_node; kernel_node = kernel_node->next_sibling())
                                        {
                                            //if(print_debug_messages) printf("position %s \n", kernel_node->first_attribute("mean")->value());

                                            float kernel_weight = atof(kernel_node->first_attribute("weight")->value());
                                            std::vector<float> pos;
                                            std::vector<float> cov;
                                            initVectorFromCSVString(pos, kernel_node->first_attribute("mean")->value());
                                            initVectorFromCSVString(cov, kernel_node->first_attribute("covariance")->value());

                                            if (pos.size() == 3 && cov.size() == 9)
                                            {
                                                MatrixXf mat = initMatrixXf(cov, 3,3);
                                                gmm->addGaussianModel(kernel_weight, pos, mat);
                                            }
                                        }

                                    }

                                    ASR::GMM_Ptr orientationGMM = boost::shared_ptr<ASR::GMM>(new ASR::GMM());

                                    // get orientation kernels
                                    for(xml_node<> * orientation_node = root_children->first_node("orientation"); orientation_node; orientation_node = orientation_node->next_sibling())
                                    {
                                        for(xml_node<> * kernel_node2 = orientation_node->first_node("kernel"); kernel_node2; kernel_node2 = kernel_node2->next_sibling())
                                        {
                                            //if(print_debug_messages) printf("orientation  %s node %s \n", kernel_node2->first_attribute("mean")->value(),
                                            //                                root_children->first_attribute("name")->value());

                                            float kernel_weight = atof(kernel_node2->first_attribute("weight")->value());
                                            std::vector<float> pos;
                                            std::vector<float> cov;
                                            initVectorFromCSVString(pos, kernel_node2->first_attribute("mean")->value());
                                            initVectorFromCSVString(cov, kernel_node2->first_attribute("covariance")->value());

                                            if (pos.size() == 4 && cov.size() == 16)
                                            {
                                                MatrixXf mat = initMatrixXf(cov, 4, 4);
                                                orientationGMM->addGaussianModel(kernel_weight, pos, mat);
                                            }
                                        }
                                    }



                                   root->initNewChild(root_children->first_attribute("name")->value(), root, gmm, orientationGMM);

                                   // traverse tree recursive to add children to the roots children
                                   std::string name = root_children->first_attribute("name")->value();
                                   traverseSceneShapeRecursive(name, root_children, root);

                                }
                            }
                        }
                    }
                }
            }
        }
    }
/*
    if(!getSceneGraphRoot())
        ROS_INFO("Warning there ist no root object. Object %s is probably not part of the scene. Building Scene Graph failed!\n\n", reference_object_name.c_str());
*/
}


unsigned int asrPosePredictionEngine::distributeVotes()
{
    // guard agains division through zero
    if(getNumberOfMissingObjects() != 0)
        return this->number_of_votes / getNumberOfMissingObjects();
    else
        return 0;
}


bool asrPosePredictionEngine::observeObject(std::string object_name, Eigen::Vector3f position, Eigen::Vector4f orientation)
{
    if(!getSceneGraphRoot())
    {
        ROS_INFO("Warning there ist no root object. Bobserving obj failed!");
        return false;
    }

    bool integrated_evidence = false;

    asrSceneGraphNodePtr observed;

    //ROS_INFO("Searching node %s", object_name.c_str());

    // Check if a node with the given name exists and if it was observed previously
    if(getSceneGraphRoot()->findNode(object_name, observed) && !observed->isObjectObserved())
    {
        if(print_debug_messages) printf("\n");

        number_obsereved_objects++;
        observed->observe(position, orientation);
        int n = distributeVotes();
        getSceneGraphRoot()->setVotesPerNode(n);

        integrated_evidence = true;
        if(print_debug_messages) printf("Observed object %s, Set votes per node to %i\n\n", object_name.c_str(), n);
    } else
    {
        // The observed object doesnt exist in the scene or it was already observed.
        integrated_evidence = false;
        //ROS_ERROR("The observed object doesnt exist in the scene or it was already observed!\n");
    }

    return integrated_evidence;
}


void asrPosePredictionEngine::publishHypothesisMessages(std::vector<ASR::AttributedPoint> hypothesis_list)
{
    if (!publish_hypothesis || hypothesis_list.empty())
        return;

    for(auto p : hypothesis_list)
    {
        asr_msgs::AsrAttributedPoint point;
        point.type = p.objectType;
        point.pose = p.pose;
        hypothesis_publisher.publish(point);
    }
}


void asrPosePredictionEngine::calcHypotheses()
{
    if(print_debug_messages) ROS_INFO("Calling calcHypothesis(). scene_name: %s", scene_name.c_str());

    // ignore the background scene because there is no scene graph
    if(scene_name.compare("background") == 0)
        return;

    if(!getSceneGraphRoot())
    {
        ROS_ERROR("Warning: calcHypothesis: There is no Scene Graph build. Return.");
        return;
    }

    bool new_evidence_integrated = false;

    for(boost::shared_ptr<asr_msgs::AsrObject> pObject : evidence_buffer)
    {

      if(!pObject->sampledPoses.size()){
    std::cerr << "Got a AsrObject without poses." << std::endl;
	std::exit(1);    
      }

      auto position = pObject->sampledPoses.front().pose.position;

      printf("A new was object observed! object %s at (%.2f, %.2f, %.2f) %s\n",
	     pObject->type.c_str(),
	     position.x, position.y, position.z,
	     pObject->header.frame_id.c_str());

      auto evidence = pObject;

      // Transform object to the base koordinate system
      transformObject(pObject, this->baseFrameId.c_str());


      auto orientation = pObject->sampledPoses.front().pose.orientation;

      /**
       * The object was observed and transformed, now integrate it into the scene graph.
       */
      bool b = observeObject(evidence->type,
			     Eigen::Vector3f(position.x, position.y, position.z),
			     Eigen::Vector4f(orientation.w, orientation.x, orientation.y, orientation.z));
      new_evidence_integrated |= b;

      if(new_evidence_integrated)
	{
	  ROS_INFO ("Object %s transformed to frame %s (%.2f, %.2f, %.2f) (w: %.2f, x: %.2f, y: %.2f, z: %.2f,)",
		    evidence->type.c_str(), this->baseFrameId.c_str(),
		    position.x, position.y, position.z,
		    orientation.w, orientation.x, orientation.y, orientation.z);
	  ROS_INFO("Object %s will be integrated into the scene graph", evidence->type.c_str());

	} else
	{
	  //ROS_INFO("Integrating obj failed!");
	}
        
    }


    /**
      * If the obseved object provides new information, compute the new hypotheses.
      * Start calculation at the root node to traverse the whole scene graph
      */
    if(new_evidence_integrated)
    {
        if(getSceneGraphRoot())
        {
            getSceneGraphRoot()->calcChildPoseHypothesesRecursive();
            ROS_INFO("Evidence(s) successfully integrated into scene graph");
        }
    }

    // clear the evidence buffer
    evidence_buffer.clear();
}

void asrPosePredictionEngine::collectPoseHypothesesRecursive(std::vector<ASR::AttributedPoint> &out_hypotheses,
                                    std::vector<ASR::AttributedPoint> &out_found_objects)
{
    if(print_debug_messages) ROS_INFO("Calling collectHypothesis(). scene_name: %s", scene_name.c_str());

    // ignore the background scene because there is no scene graph
    if(scene_name.compare("background") == 0)
        return;

    if(!getSceneGraphRoot())
    {
        ROS_ERROR("Warning: collectHypothesis: There is no Scene Graph build. Return.");
        return;
    }
    getSceneGraphRoot()->collectPoseHypothesesRecursive(out_hypotheses, out_found_objects);
}


void asrPosePredictionEngine::onObjectMsg(const boost::shared_ptr<asr_msgs::AsrObject>& pObject)
{
    // Check if the object is already in the buffer
    for(auto o : evidence_buffer)
    {
        if(o->type.compare(pObject->type) == 0)
        {
            if(print_debug_messages) printf("Observation callback. Object %s already in buffer.\n", pObject->type.c_str());
            return;
        }
    }

    if(print_debug_messages)
        printf("Observation callback. Put object %s to evidence buffer.\n", pObject->type.c_str());

    evidence_buffer.push_back(pObject);
}


bool asrPosePredictionEngine::transformObject(boost::shared_ptr<asr_msgs::AsrObject> in_out_object, std::string targetFrame)
{
    // Check if the object is already in the target frame
    if(in_out_object->header.frame_id.compare(targetFrame) == 0)
        return true;

    try
    {
        mObjectTransform.setBaseFrame(targetFrame);

        // transform the object to the target frame
        mObjectTransform.transform(in_out_object);

    } catch(std::exception e)
    {
        ROS_ERROR("Failed to resolve transformation in target coordinate frame. Object: %s frame: %s",
                  in_out_object->type.c_str(), targetFrame.c_str());
        return false;
    }

    if(!in_out_object->sampledPoses.size()){
      std::cerr << "Got a AsrObject without poses." << std::endl;
      std::exit(1);    
    }

    // get the transformed position
    auto position = in_out_object->sampledPoses.front().pose.position;
    auto orientation = in_out_object->sampledPoses.front().pose.orientation;

    ROS_INFO ("Object %s transformed to frame %s (%.2f, %.2f, %.2f) (w: %.2f, x: %.2f, y: %.2f, z: %.2f,)",
	      in_out_object->type.c_str(), targetFrame.c_str(),
	      position.x, position.y, position.z,
	      orientation.w, orientation.x, orientation.y, orientation.z);

    return true;
}


void asrPosePredictionEngine::traverseSceneShapeRecursive(std::string parent_name, xml_node<> * parent_node, ASR::asrSceneGraphNodePtr root)
{
    // Intialize the roots children
    for(xml_node<> * child = parent_node->first_node("child"); child; child = child->next_sibling())
    {
        ASR::GMM_Ptr gmm = boost::shared_ptr<ASR::GMM>(new ASR::GMM());
        ASR::GMM_Ptr orientationGMM = boost::shared_ptr<ASR::GMM>(new ASR::GMM());

        for(xml_node<> * position_node = child->first_node("position"); position_node; position_node = position_node->next_sibling())
        {
            for(xml_node<> * kernel_node = position_node->first_node("kernel"); kernel_node; kernel_node = kernel_node->next_sibling())
            {
                //if(print_debug_messages) printf("position %s \n", kernel_node->first_attribute("mean")->value());

                float kernel_weight = atof(kernel_node->first_attribute("weight")->value());
                std::vector<float> pos;
                std::vector<float> cov;
                initVectorFromCSVString(pos, kernel_node->first_attribute("mean")->value());
                initVectorFromCSVString(cov, kernel_node->first_attribute("covariance")->value());

                if (pos.size() == 3 && cov.size() == 9)
                {
                    MatrixXf mat = initMatrixXf(cov, 3,3);
                    gmm->addGaussianModel(kernel_weight, pos, mat);
                }
            }
        }

        // get orientation kernels
        for(xml_node<> * orientation_node = child->first_node("orientation"); orientation_node; orientation_node = orientation_node->next_sibling())
        {
            for(xml_node<> * kernel_node2 = orientation_node->first_node("kernel"); kernel_node2; kernel_node2 = kernel_node2->next_sibling())
            {
                //if(print_debug_messages) printf("orientation %s \n", kernel_node2->first_attribute("mean")->value());

                float kernel_weight = atof(kernel_node2->first_attribute("weight")->value());
                std::vector<float> pos;
                std::vector<float> cov;
                initVectorFromCSVString(pos, kernel_node2->first_attribute("mean")->value());
                initVectorFromCSVString(cov, kernel_node2->first_attribute("covariance")->value());

                if (pos.size() == 4 && cov.size() == 16)
                {
                    MatrixXf mat = initMatrixXf(cov, 4, 4);
                    orientationGMM->addGaussianModel(kernel_weight, pos, mat);
                }

            }
        }
/*
        if(print_debug_messages) printf("Current Child_name: %s, parent_name: %s \n", child->first_attribute("name")->value(),
               parent_name.c_str());
*/
        ASR::asrSceneGraphNodePtr node;
        if(root->findNode(parent_name.c_str(), node))
        {
            //if(print_debug_messages) printf("Found parent node %s, adding child %s\n", parent_name.c_str(), child->first_attribute("name")->value());
            node->initNewChild(child->first_attribute("name")->value(), node, gmm, orientationGMM);
        } else
        {
            if(print_debug_messages) printf("Error node %s not found\n", parent_name.c_str());
        }

        std::string child_name = child->first_attribute("name")->value();
        traverseSceneShapeRecursive(child_name, child, root);
    }
}


void asrPosePredictionEngine::initVectorFromCSVString(std::vector<float> &x, std::string csv)
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


MatrixXf asrPosePredictionEngine::initMatrixXf(std::vector<float> values, unsigned int rows, unsigned int cols)
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



}
