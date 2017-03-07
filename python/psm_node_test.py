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

def main():
    objects = []
    
    # detect some objects
    obj = AsrObject()
    obj.type = 'Cup'
    obj.identifier = '/map'
    obj.header.frame_id = '/map'
    obj.poseEstimation.pose.position.x = 0.10103984720538714 
    obj.poseEstimation.pose.position.y = -0.0140818815746599 
    obj.poseEstimation.pose.position.z = 0.16235541952861962
    obj.poseEstimation.pose.orientation.w = 0.99553314478114419 
    obj.poseEstimation.pose.orientation.x = -0.0074066082083501595 
    obj.poseEstimation.pose.orientation.y = -0.030613037027407385
    obj.poseEstimation.pose.orientation.z =  0.037845601112626276
    objects.append(obj)

    obj2 = AsrObject()
    obj2.type = 'PlateDeep'
    obj2.identifier = '/map'
    obj2.header.frame_id = '/map'
    obj2.poseEstimation.pose.position.x = 0
    obj2.poseEstimation.pose.position.y = 0
    obj2.poseEstimation.pose.position.z = 0
    obj2.poseEstimation.pose.orientation.w = 1
    obj2.poseEstimation.pose.orientation.x = 0.0
    obj2.poseEstimation.pose.orientation.y = 0.0
    obj2.poseEstimation.pose.orientation.z = 0.0
    objects.append(obj2)

    obj2 = AsrObject()
    obj2.type = 'Smacks'
    obj2.identifier = '/map'
    obj2.header.frame_id = '/map'
    obj2.poseEstimation.pose.position.x = 0.076435975386262803  
    obj2.poseEstimation.pose.position.y = -0.11043452060606049
    obj2.poseEstimation.pose.position.z = 0.30012156632996601
    obj2.poseEstimation.pose.orientation.w = 0.99788836565656736
    obj2.poseEstimation.pose.orientation.x = -0.020827786229582479
    obj2.poseEstimation.pose.orientation.y = -0.011928087026733992
    obj2.poseEstimation.pose.orientation.z = 0.01032709707676762
    #objects.append(obj2)
    
    
    
    try:
        rospy.wait_for_service('/psm_node', timeout=5)
        
    except rospy.exceptions.ROSException, e:
        rospy.loginfo('Could not reach service')

    try:
	service_proxy = rospy.ServiceProxy('/psm_node', psm_node)
	
	# call the service
	# the parameters are:	a list of already observed objects, 
	response = service_proxy(objects)
	
	# print the results
	print("Number of generated hypothesis: " + str(len(response.pose_hypothesis.elements)))
	print("All scene objects found: " + str(response.all_scene_objects_found));
	#print(response)
	
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
    
    
    hypothesis_cloud = []
    
    
    try:
        hypothesis_cloud = main()
    except rospy.ROSInterruptException: pass
    
    
    
    
    
    
    
    
    
