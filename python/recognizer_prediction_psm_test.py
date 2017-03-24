'''
Copyright (c) 2016, Braun Kai, Meissner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from asr_msgs.msg import AsrObject
from asr_recognizer_prediction_psm.srv import *
import roslib
import rospy
import os
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import signal
import sys
import rospkg

from geometry_msgs.msg import PoseWithCovariance

def main():
    objects = []
    
    obj = AsrObject()
    obj.type = 'VitalisSchoko'
    obj.identifier = '/map'
    obj.header.frame_id = '/map'
    poseWithCovariance = PoseWithCovariance()
    poseWithCovariance.pose.position.x = -0.3
    poseWithCovariance.pose.position.y = -1
    poseWithCovariance.pose.position.z = -0.6
    poseWithCovariance.pose.orientation.w = 0.5
    poseWithCovariance.pose.orientation.x = 0
    poseWithCovariance.pose.orientation.y = 0
    poseWithCovariance.pose.orientation.z = -0.5
    obj.sampledPoses.append(poseWithCovariance)
    objects.append(obj)
    
    obj2 = AsrObject()
    obj2.type = 'PlateDeep'
    obj2.identifier = '/map'
    obj2.header.frame_id = '/map'
    poseWithCovariance.pose.position.x = -0.51
    poseWithCovariance.pose.position.y = -1.2
    poseWithCovariance.pose.position.z = -0.65
    poseWithCovariance.pose.orientation.w = 0.5
    poseWithCovariance.pose.orientation.x = 0.0
    poseWithCovariance.pose.orientation.y = 0.0
    poseWithCovariance.pose.orientation.z = -0.5
    obj2.sampledPoses.append(poseWithCovariance)
    #objects.append(obj2)

    obj2 = AsrObject()
    obj2.type = 'Cup'
    obj2.identifier = '/map'
    obj2.header.frame_id = '/map'
    poseWithCovariance.pose.position.x = 0
    poseWithCovariance.pose.position.y = -1.13
    poseWithCovariance.pose.position.z = -0.65
    poseWithCovariance.pose.orientation.w = 0.5
    poseWithCovariance.pose.orientation.x = 0.0
    poseWithCovariance.pose.orientation.y = 0.0
    poseWithCovariance.pose.orientation.z = -0.5
    obj2.sampledPoses.append(poseWithCovariance)
    #objects.append(obj2)
    


    path = rospkg.RosPack().get_path('asr_recognizer_prediction_psm') + '/models/dome_scene1.xml'

    scenes = []
    scenes.append('background')
    scenes.append('marker_scene1')
    scenes.append('marker_scene2')


    scene_probabilities = []
    scene_probabilities.append(0.25)
    scene_probabilities.append(0.5)
    scene_probabilities.append(0.25)
    
    num_overall_hypothesis = 10
    
    base_frame = '/map'
    
    
    try:
        rospy.wait_for_service('/recognizer_prediction_psm', timeout=5)
        
    except rospy.exceptions.ROSException, e:
        rospy.loginfo('Could not reach service')

    try:
	generate_hypothesis = rospy.ServiceProxy('/recognizer_prediction_psm', recognizer_prediction_psm)
	
	# call the service
        # the parameters are: 	the path to the xml file,
        #                       a list of the scenes,
	# 			a list of the scene probabilities,
	#			the number of hypothesis that should be generated,
	#			a list of already observed objects, 
	#			the frame to which the objects and the hypothesis should be transformed into
        response = generate_hypothesis(path, 
				       scenes, 
				       scene_probabilities, 
				       num_overall_hypothesis, 
				       objects, 
				       base_frame)
	
	# print the results
	#print(response)
	print('All scene objects found: ' + str(response.all_scene_objects_found))
	
	return response.pose_hypothesis.elements
      

	
    except rospy.ServiceException, e:
        rospy.loginfo('Could not generate hyps')
        
    return []

# This method catches the abort signal crtl + c
def signal_handler(signal, frame):
    sys.exit(0)

        

if __name__ == '__main__':
    # catch ctrl + c
    signal.signal(signal.SIGINT, signal_handler)
    
    
    topic = '/recognizer_prediction_psm/hypothesis_psm_marker_array'
    hypothesis_cloud = []
    
    
    try:
        hypothesis_cloud = main()
    except rospy.ROSInterruptException: pass
    
    
    
    
    
    
    
    
    
