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


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <math.h>
#include "AttributedPoint.h"

#include "GMM.h"

namespace ASR
{

class OcmSceneGraphNode
{
private:
    bool isObserved;
    std::vector<Eigen::Vector3f> samples;               // Samples for position
    std::vector<Eigen::Vector4f> samplesOrientation;    // Samples for orientation
    std::vector<boost::shared_ptr<OcmSceneGraphNode> > childs;
    std::vector<GMM_Ptr> childsRelativePoseGmm;         // gmm for the position
    std::vector<GMM_Ptr> childsRelativeRotationGmm;     // gmm for the orientation

    std::string name;

    boost::shared_ptr<OcmSceneGraphNode> parent;
    unsigned int votes_per_node;


    bool print_debug_messages;


public:

    /**
      * Create an empty node in the graph.
      */
    OcmSceneGraphNode(bool enable_debug_mode = true)
    {
        this->name = "";
        isObserved = false;

        // Get a parameter from the parameter server
        print_debug_messages = enable_debug_mode;
    }

    /**
      * A root node with the given name at the observed global position.
      */
    OcmSceneGraphNode(std::string name, Eigen::Vector3f root_position, Eigen::Vector4f root_orientation, int votes = 100, bool enable_debug_mode = true)
    {
        this->print_debug_messages = enable_debug_mode;

        this->name = name;
        this->votes_per_node = votes;
        observe(root_position, root_orientation);
    }

    /**
      * Finds a node with the given name.
      * If no node with this name is found @code false is returned
      * @param key - the name of the node that is searched
      * @param out_pointer - the pointer to which the node is stored to
      * @return @code true if the node was found @code false otherwise
      */
    bool findNode(const std::string key, boost::shared_ptr<ASR::OcmSceneGraphNode> &out_pointer)
    {
        for(unsigned int i=0;i<childs.size(); i++)
        {
            //printf("Comparing '%s' to '%s' = %i\n", key.c_str(), childs.at(i)->getName().c_str(), getChild(i)->getName().compare(key) );

            if(getChild(i)->getName().compare(key) == 0)
            {
                //printf("Object %s found\n", key.c_str());
                out_pointer = getChild(i);
                return true;
            }
        }

        for(unsigned int i=0;i<childs.size(); i++)
        {
           if(childs.at(i)->findNode(key, out_pointer))
               return true;
        }

        if(print_debug_messages) printf("Object %s not found\n", key.c_str());
        return false;
    }

    /**
      * Adds a new child with the given name, parent node and pose gmm.
      */
    void initNewChild(std::string name, boost::shared_ptr<OcmSceneGraphNode> parent, GMM_Ptr gmm, GMM_Ptr orientationGMM)
    {
        boost::shared_ptr<OcmSceneGraphNode> node = boost::shared_ptr<OcmSceneGraphNode>(new OcmSceneGraphNode(print_debug_messages));
        node->setParent(parent);
        childsRelativePoseGmm.push_back(gmm);
        childsRelativeRotationGmm.push_back(orientationGMM);
        node->setName(name);
        node->setVotesPerNode(parent->getVotesPerNode());

        childs.push_back(node);
        if(print_debug_messages) printf("Child %s added to node %s\n", name.c_str(), parent->getName().c_str());
    }

    /**
      * Returns the absolute pose of the node.
      * If it is observed the observed postion is returned.
      * If not one of the hypotheses is randomly chosen and returned.
      */
    Eigen::Vector3f getPose()
    {
        if(isObjectObserved())
            return samples.at(0);
        else
        {
            std::random_device rd;
            std::mt19937 mt(rd());

            std::uniform_real_distribution<float> rand_id(1, samples.size());
            int id = floor(rand_id(mt));
            return samples.at(id);
        }
    }

    Eigen::Vector4f getOrientation()
    {
        if(isObjectObserved())
            return samplesOrientation.at(0);
        else
        {
            std::random_device rd;
            std::mt19937 mt(rd());

            std::uniform_real_distribution<float> rand_id(1, samplesOrientation.size());
            int id = floor(rand_id(mt));
            return samplesOrientation.at(id);
        }
    }

    /**
      * The position of the object is observed. Delete all position hypotheses
      * and add the observed position.
      */
    void observe(Eigen::Vector3f position, Eigen::Vector4f orientation)
    {
        isObserved = true;

        samples.clear();
        samples.push_back(position);

        samplesOrientation.clear();
        samplesOrientation.push_back(orientation);

        //ROS_INFO("Object %s observed at %.2f, %.2f, %.2f.", getName().c_str(), position.x(), position.y(), position.z());
    }

    /**
      * Calculates n hypotheses for the childs. The tree is traversed recursivly
      * from this node. Call root->calcChildHypothesesRecursive to get Hypotheses for
      * the whole graph.
      */
    void calcChildPoseHypothesesRecursive()
    {
        ROS_ASSERT(childs.size() == childsRelativePoseGmm.size());
        if(print_debug_messages) ROS_INFO("Current Node %s, Number of childs: %i, Number of GMMs: %i, #Votes %i", name.c_str(), (int)childs.size(), (int)childsRelativePoseGmm.size(), getVotesPerNode());

        for(unsigned int i=0;i<childs.size();i++)
        {
            if (!childs.at(i)->isObjectObserved())
            {
                // Clear all old samples
                childs.at(i)->samples.clear();

                //printf("Votes per node %i\n", getVotesPerNode());
                for(unsigned int j=0; j < getVotesPerNode() ;j++)
                {
                    std::vector<float> v;
                    Eigen::Vector3f hyp;
                    childsRelativePoseGmm.at(i)->sampleRandomValues(v);
                    hyp.x() = v.at(0);
                    hyp.y() = v.at(1);
                    hyp.z() = v.at(2);

                    // The child poses are relative to the parents pose
                    hyp += this->getPose();
                    //ROS_INFO("Hyp (%s): %.2f %.2f %.2f", getChild(i)->getName().c_str(), hyp.x(), hyp.y(), hyp.z());

                    v.clear();
                    Eigen::Vector4f orientationSample;
                    childsRelativeRotationGmm.at(i)->sampleRandomValues(v);

                    orientationSample.w() = v.at(0);
                    orientationSample.x() = v.at(1);
                    orientationSample.y() = v.at(2);
                    orientationSample.z() = v.at(3);

                    Eigen::Quaternion<float> q(orientationSample);
                    Eigen::Quaternion<float> q_parent(getOrientation());
                    // Rotate the childs orientation with the parents orientation
                    q = q_parent * q;

                    orientationSample.w() = q.w();
                    orientationSample.x() = q.x();
                    orientationSample.y() = q.y();
                    orientationSample.z() = q.z();

                    childs.at(i)->addSample(hyp, orientationSample);
                }
            }
            // Traverse tree recursivly
            childs.at(i)->calcChildPoseHypothesesRecursive();
        }
    }

    /**
      * Collects all pose hypotheses for the childs. The tree is traversed recursivly
      * from this node. Call rootNode->collectPoseHypothesesRecursive(...,...)
      * to traverse the whole graph.
      * @param out_hypotheses the list the hypotheses are stored into.
      * @param out_found_objects the list to which the poses of all observed objects are stored into.
      */
    void collectPoseHypothesesRecursive(std::vector<ASR::AttributedPoint> &out_hypotheses,
                                        std::vector<ASR::AttributedPoint> &out_found_objects)
    {
        ROS_ASSERT(out_hypotheses);
        ROS_ASSERT(out_found_objects);

        // Add the nodes position to the samples
        if(this->isObjectObserved())
        {
            Eigen::Vector3f v = this->getPose();
            Eigen::Vector4f ori = this->getOrientation();
            out_found_objects.push_back(AttributedPoint(getName().c_str(),
                                                        v.x(), v.y(), v.z(),
                                                        ori.w(), ori.x(), ori.y(), ori.z()));
        }

        for(unsigned int i=0;i<childs.size();i++)
        {
            //for (Vector3f sample : getChild(i)->getPoseSamples())
            for(unsigned int s=0; s < getChild(i)->getPoseSamples().size(); s++)
            {
                Eigen::Vector3f pos_sample = getChild(i)->getPoseSamples().at(s);
                Eigen::Vector4f ori_sample = getChild(i)->getOrientationSamples().at(s);

                if(print_debug_messages) ROS_INFO("type %s, pos: (%.2f %.2f %.2f), ori: (%.2f %.2f %.2f %.2f) ",
                                                  getChild(i)->getName().c_str(),
                                                  pos_sample.x(), pos_sample.y(), pos_sample.z(),
                                                  ori_sample.w(), ori_sample.x(), ori_sample.y(), ori_sample.z());

                ASR::AttributedPoint hyp = AttributedPoint(getChild(i)->getName().c_str(),
                                                           pos_sample.x(), pos_sample.y(), pos_sample.z(),
                                                           ori_sample.w(), ori_sample.x(), ori_sample.y(), ori_sample.z());


                if(getChild(i)->isObjectObserved())
                {
                    out_found_objects.push_back(hyp);
                } else
                {
                    out_hypotheses.push_back(hyp);
                }
            }

            // Traverse tree recursivly
            childs.at(i)->collectPoseHypothesesRecursive(out_hypotheses, out_found_objects);
        }
    }



    /**
      * Setters and getters
      */

    void setName(std::string name) { this->name = name;}
    std::string getName() {return this->name;}

    //void setGMMs(std::vector<GaussianMixtureModel> childsRelativePoseGmm) { this->childsRelativePoseGmm = childsRelativePoseGmm;}

    bool isObjectObserved() {return isObserved;}

    std::vector<Eigen::Vector3f> getPoseSamples() { return samples; }
    std::vector<Eigen::Vector4f> getOrientationSamples() { return samplesOrientation; }

    void setParent(boost::shared_ptr<OcmSceneGraphNode> p ) {this->parent = p;}

    void addSample(Eigen::Vector3f positionSample, Eigen::Vector4f orientationSample) { samples.push_back(positionSample); samplesOrientation.push_back(orientationSample); }

    boost::shared_ptr<OcmSceneGraphNode> getChild(unsigned int index) {
        ROS_ASSERT(index < childs.size());
        return childs.at(index);
    }

    unsigned int getNumberOfChilds() {return childs.size();}

    unsigned int getVotesPerNode() { return this->votes_per_node;}

    Eigen::Vector3f getMeanOfChildPosition(unsigned int index)
    {
        ROS_ASSERT(index < childsRelativePoseGmm.size());
        std::vector<float> means = childsRelativePoseGmm.at(index)->getModel(0)->getMean();
        Eigen::Vector3f mean;
        mean.x() = means.at(0);
        mean.y() = means.at(1);
        mean.z() = means.at(2);

        return mean;
    }

    /**
      * Updates the nodes per node this node and all child nodes.
      * @param n - the number of the new votes per node
      */
    void setVotesPerNode(unsigned int n)
    {
        //printf("Set votes per node from node %s to %i\n", this->getName().c_str(), n);
        this->votes_per_node = n;
        for(unsigned int i=0;i<childs.size();i++)
        {
            childs.at(i)->setVotesPerNode(n);
        }
    }

};

typedef boost::shared_ptr<OcmSceneGraphNode> asrSceneGraphNodePtr;

}
