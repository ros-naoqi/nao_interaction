#!/usr/bin/env python

#
# ROS node to serve NAOqi Vision functionalities
# This code is currently compatible to NaoQI version 1.6
#
# Copyright 2014 Manos Tsardoulias, CERTH/ITI
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the CERTH/ITI nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import naoqi

from naoqi import ( 
    ALModule, 
    ALBroker, 
    ALProxy)
    
from naoqi_driver.naoqi_node import NaoqiNode

#~ ROS msgs
from std_msgs.msg import (
    String, 
    Float32, 
    Int32)
    
from std_srvs.srv import (
    EmptyResponse,
    Empty)
    
from geometry_msgs.msg import (
    Point)

#~ nao-ros msgs
from nao_interaction_msgs.msg import (
    FaceDetected,
    MovementDetected,
    LandmarkDetected)
    
from nao_interaction_msgs.srv import (
    LearnFace,
    LearnFaceResponse,
    VisionMotionSensitivity)

class Constants:
    NODE_NAME = "nao_vision_interface"

class NaoVisionInterface(ALModule, NaoqiNode):
    "sss"
    def __init__(self, moduleName):
        # ROS initialization
        NaoqiNode.__init__(self, Constants.NODE_NAME )
        
        # NAOQi initialization
        self.ip = ""
        self.port = 10601
        self.moduleName = moduleName
        self.init_almodule()
        
        #~ Variable initialization
        self.faces = FaceDetected()  
        self.face_detection_enabled = False
        self.motion_detection_enabled = False
        self.landmark_detection_enabled = False
             
        #~ ROS initializations
        self.subscribeFaceSrv = rospy.Service("nao_vision/face_detection/enable", Empty, self.serveSubscribeFaceSrv)
        self.unsubscribeFaceSrv = rospy.Service("nao_vision/face_detection/disable", Empty, self.serveUnsubscribeFaceSrv)
        self.subscribeMotionSrv = rospy.Service("nao_vision/motion_detection/enable", Empty, self.serveSubscribeMotionSrv)
        self.unsubscribeMotionSrv = rospy.Service("nao_vision/motion_detection/disable", Empty, self.serveUnsubscribeMotionSrv)
        self.subscribeLandmarkSrv = rospy.Service("nao_vision/landmark_detection/enable", Empty, self.serveSubscribeLandmarkSrv)
        self.unsubscribeLandmarkSrv = rospy.Service("nao_vision/landmark_detection/disable", Empty, self.serveUnsubscribeLandmarkSrv)
        self.learnFaceSrv = rospy.Service('nao_vision/face_detection/learn_face', LearnFace, self.learnFaceSrv)
        self.forgetPersonSrv = rospy.Service('nao_vision/face_detection/forget_person', LearnFace, self.forgetPersonSrv)
        
        self.subscribe()
        
        rospy.loginfo("nao_vision_interface initialized")

    def init_almodule(self):
        # before we can instantiate an ALModule, an ALBroker has to be created
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        try:
            self.broker = ALBroker("%sBroker" % self.moduleName, self.ip, self.port, self.pip, self.pport)
        except RuntimeError,e:
            print("Could not connect to NaoQi's main broker")
            exit(1)
        ALModule.__init__(self, self.moduleName)
        
        self.memProxy = ALProxy("ALMemory",self.pip,self.pport)
        if self.memProxy is None:
            rospy.logerror("Could not get a proxy to ALMemory on %s:%d", self.pip, self.pport)
            exit(1)
            
        self.landmarkDetectionProxy = ALProxy("ALLandMarkDetection",self.pip,self.pport)
        if self.landmarkDetectionProxy is None:
            rospy.logerror("Could not get a proxy to ALLandMarkDetection on %s:%d", self.pip, self.pport)
            exit(1)
        
        self.movementDetectionProxy = ALProxy("ALMovementDetection",self.pip,self.pport)
        if self.movementDetectionProxy is None:
            rospy.logerror("Could not get a proxy to ALMovementDetection on %s:%d", self.pip, self.pport)
            exit(1)

        self.faceDetectionProxy = ALProxy("ALFaceDetection",self.pip,self.pport)
        if self.faceDetectionProxy is None:
            rospy.logerror("Could not get a proxy to ALFaceDetection on %s:%d", self.pip, self.pport)
            exit(1)

    def shutdown(self): 
        self.unsubscribe()

    def subscribe(self):
        #~ Subscriptions are performed via srv calls to save cpu power 
        pass

    def unsubscribe(self):
        if self.landmark_detection_enabled:
            self.memProxy.unsubscribeToEvent("LandmarkDetected", self.moduleName)
        if self.face_detection_enabled:
            self.memProxy.unsubscribeToEvent("FaceDetected", self.moduleName)
        if self.motion_detection_enabled:
            self.memProxy.unsubscribeToEvent("MovementDetection/MovementDetected", self.moduleName)
        

    def onLandmarkDetected(self, strVarName, value, strMessage):
        "Called when landmark was detected"
        if len(value) == 0:
            return
            
        # For the specific fields in the value variable check here:
        # https://community.aldebaran-robotics.com/doc/1-14/naoqi/vision/allandmarkdetection-api.html#landmarkdetected-value-structure
        
        msg = LandmarkDetected()  
        
        for i in range (0, len(value[1])):
            msg.shape_alpha.append(Float32(value[1][i][0][1]))
            msg.shape_beta.append(Float32(value[1][i][0][2]))
            msg.shape_sizex.append(Float32(value[1][i][0][3]))
            msg.shape_sizey.append(Float32(value[1][i][0][4]))
            msg.mark_ids.append(Int32(value[1][i][1][0]))
            
        msg.camera_local_pose.position.x = value[2][0]
        msg.camera_local_pose.position.y = value[2][1]
        msg.camera_local_pose.position.z = value[2][2]
        msg.camera_local_pose.orientation.x = value[2][3]
        msg.camera_local_pose.orientation.y = value[2][4]
        msg.camera_local_pose.orientation.z = value[2][5]
        
        msg.camera_world_pose.position.x = value[3][0]
        msg.camera_world_pose.position.y = value[3][1]
        msg.camera_world_pose.position.z = value[3][2]
        msg.camera_world_pose.orientation.x = value[3][3]
        msg.camera_world_pose.orientation.y = value[3][4]
        msg.camera_world_pose.orientation.z = value[3][5]
        
        msg.camera_name = String(value[4])
        
        self.landmarkPub.publish(msg)

    def onFaceDetected(self, strVarName, value, strMessage):
        "Called when a face was detected"
        
        if len(value) == 0:
          return
        
        # For the specific fields in the value variable check here:
        #https://community.aldebaran-robotics.com/doc/1-14/naoqi/vision/alfacedetection.html
        
        self.faces.camera_id.data = int(value[4]);
        
        self.faces.camera_0_pose.position.x = float(value[2][0])
        self.faces.camera_0_pose.position.y = float(value[2][1])
        self.faces.camera_0_pose.position.z = float(value[2][2])
        self.faces.camera_0_pose.orientation.x = float(value[2][3])
        self.faces.camera_0_pose.orientation.y = float(value[2][4])
        self.faces.camera_0_pose.orientation.z = float(value[2][5])
        
        self.faces.camera_1_pose.position.x = float(value[3][0])
        self.faces.camera_1_pose.position.y = float(value[3][1])
        self.faces.camera_1_pose.position.z = float(value[3][2])
        self.faces.camera_1_pose.orientation.x = float(value[3][3])
        self.faces.camera_1_pose.orientation.y = float(value[3][4])
        self.faces.camera_1_pose.orientation.z = float(value[3][5])
        
        self.faces.shape_alpha.data = float(value[1][0][0][1])
        self.faces.shape_beta.data = float(value[1][0][0][2])
        self.faces.shape_sizeX.data = float(value[1][0][0][3])
        self.faces.shape_sizeY.data = float(value[1][0][0][4])
        
        self.faces.face_id.data = float(value[1][0][1][0])
        self.faces.score_reco.data = float(value[1][0][1][1])
        self.faces.face_label.data = str(value[1][0][1][2])
        
        self.faces.left_eye_eyeCenter_x.data = float(value[1][0][1][3][0])
        self.faces.left_eye_eyeCenter_y.data = float(value[1][0][1][3][1])
        self.faces.left_eye_noseSideLimit_x.data = float(value[1][0][1][3][2])
        self.faces.left_eye_noseSideLimit_y.data = float(value[1][0][1][3][3])
        self.faces.left_eye_earSideLimit_x.data = float(value[1][0][1][3][4])
        self.faces.left_eye_earSideLimit_y.data = float(value[1][0][1][3][5])
        self.faces.left_eye_topLimit_x.data = float(value[1][0][1][3][6])
        self.faces.left_eye_topLimit_y.data = float(value[1][0][1][3][7])
        self.faces.left_eye_bottomLimit_x.data = float(value[1][0][1][3][8])
        self.faces.left_eye_bottomLimit_y.data = float(value[1][0][1][3][9])
        self.faces.left_eye_midTopEarLimit_x.data = float(value[1][0][1][3][10])
        self.faces.left_eye_midTopEarLimit_y.data = float(value[1][0][1][3][11])
        self.faces.left_eye_midTopNoseLimit_x.data = float(value[1][0][1][3][12])
        self.faces.left_eye_midTopNoseLimit_y.data = float(value[1][0][1][3][13])
        
        self.faces.right_eye_eyeCenter_x.data = float(value[1][0][1][4][0])
        self.faces.right_eye_eyeCenter_y.data = float(value[1][0][1][4][1])
        self.faces.right_eye_noseSideLimit_x.data = float(value[1][0][1][4][2])
        self.faces.right_eye_noseSideLimit_y.data = float(value[1][0][1][4][3])
        self.faces.right_eye_earSideLimit_x.data = float(value[1][0][1][4][4])
        self.faces.right_eye_earSideLimit_y.data = float(value[1][0][1][4][5])
        self.faces.right_eye_topLimit_x.data = float(value[1][0][1][4][6])
        self.faces.right_eye_topLimit_y.data = float(value[1][0][1][4][7])
        self.faces.right_eye_bottomLimit_x.data = float(value[1][0][1][4][8])
        self.faces.right_eye_bottomLimit_y.data = float(value[1][0][1][4][9])
        self.faces.right_eye_midTopEarLimit_x.data = float(value[1][0][1][4][10])
        self.faces.right_eye_midTopEarLimit_y.data = float(value[1][0][1][4][11])
        self.faces.right_eye_midTopNoseLimit_x.data = float(value[1][0][1][4][12])
        self.faces.right_eye_midTopNoseLimit_y.data = float(value[1][0][1][4][13])
        
        self.faces.left_eyebrow_noseSideLimit_x.data = float(value[1][0][1][5][0])
        self.faces.left_eyebrow_noseSideLimit_y.data = float(value[1][0][1][5][1])
        self.faces.left_eyebrow_center_x.data = float(value[1][0][1][5][2])
        self.faces.left_eyebrow_center_y.data = float(value[1][0][1][5][3])
        self.faces.left_eyebrow_earSideLimit_x.data = float(value[1][0][1][5][4])
        self.faces.left_eyebrow_earSideLimit_y.data = float(value[1][0][1][5][5])
        
        self.faces.right_eyebrow_noseSideLimit_x.data = float(value[1][0][1][6][0])
        self.faces.right_eyebrow_noseSideLimit_y.data = float(value[1][0][1][6][1])
        self.faces.right_eyebrow_center_x.data = float(value[1][0][1][6][2])
        self.faces.right_eyebrow_center_y.data = float(value[1][0][1][6][3])
        self.faces.right_eyebrow_earSideLimit_x.data = float(value[1][0][1][6][4])
        self.faces.right_eyebrow_earSideLimit_y.data = float(value[1][0][1][6][5])
        
        self.faces.nose_bottomCenterLimit_x.data = float(value[1][0][1][7][0])
        self.faces.nose_bottomCenterLimit_y.data = float(value[1][0][1][7][1])
        self.faces.nose_bottomLeftLimit_x.data = float(value[1][0][1][7][2])
        self.faces.nose_bottomLeftLimit_y.data = float(value[1][0][1][7][3])
        self.faces.nose_bottomRightLimit_x.data = float(value[1][0][1][7][4])
        self.faces.nose_bottomRightLimit_y.data = float(value[1][0][1][7][5])
        
        self.faces.mouth_leftLimit_x.data = float(value[1][0][1][8][0])
        self.faces.mouth_leftLimit_y.data = float(value[1][0][1][8][1])
        self.faces.mouth_rightLimit_x.data = float(value[1][0][1][8][2])
        self.faces.mouth_rightLimit_y.data = float(value[1][0][1][8][3])
        self.faces.mouth_topLimit_x.data = float(value[1][0][1][8][4])
        self.faces.mouth_topLimit_y.data = float(value[1][0][1][8][5])
        self.faces.mouth_bottomLimit_x.data = float(value[1][0][1][8][6])
        self.faces.mouth_bottomLimit_y.data = float(value[1][0][1][8][7])
        self.faces.mouth_midTopLeftLimit_x.data = float(value[1][0][1][8][8])
        self.faces.mouth_midTopLeftLimit_y.data = float(value[1][0][1][8][9])
        self.faces.mouth_midTopRightLimit_x.data = float(value[1][0][1][8][10])
        self.faces.mouth_midTopRightLimit_y.data = float(value[1][0][1][8][11])
        self.faces.mouth_midBottomRightLimit_x.data = float(value[1][0][1][8][12])
        self.faces.mouth_midBottomRightLimit_y.data = float(value[1][0][1][8][13])
        self.faces.mouth_midBottomLeftLimit_x.data = float(value[1][0][1][8][14])
        self.faces.mouth_midBottomLeftLimit_y.data = float(value[1][0][1][8][15])

        self.facesPub.publish(self.faces)

    def handleSensitivityChangeSrv(self, req):
        if (req.sensitivity.data < 0.0) or (req.sensitivity.data > 1.0):
            return
        
        self.movementDetectionProxy.setSensitivity(req.sensitivity.data)
        rospy.loginfo("Sensitivity of nao_movement_detection changed to %f", req.sensitivity.data)

    def onMovementDetected(self, strVarName, value, strMessage):
        "Called when movement was detected"
        datavar = self.memProxy.getData("MovementDetection/MovementInfo")
        
        movement = MovementDetected()  
        
        movement.gravity_center.x = datavar[0][0][0]
        movement.gravity_center.y = datavar[0][0][1]

        movement.mean_velocity.x = datavar[0][1][0]
        movement.mean_velocity.y = datavar[0][1][1]

        for i in range(0, len(datavar[0][2])):
          movement.points_poses.append(Point())
          movement.points_poses[i].x = datavar[0][2][i][0]
          movement.points_poses[i].y = datavar[0][2][i][1]
          
          movement.points_speeds.append(Point())
          movement.points_speeds[i].x = datavar[0][3][i][0]
          movement.points_speeds[i].y = datavar[0][3][i][1]
        
        self.movementPub.publish(movement)

    def serveSubscribeFaceSrv(self, req):
        self.facesPub = rospy.Publisher("nao_vision/faces_detected", FaceDetected)
        self.memProxy.subscribeToEvent("FaceDetected", self.moduleName, "onFaceDetected")
        self.face_detection_enabled = True
        
    def serveUnsubscribeFaceSrv(self, req):
        if self.face_detection_enabled:
            self.memProxy.unsubscribeToEvent("FaceDetected", self.moduleName)
            self.facesPub.unregister()
            self.face_detection_enabled = False
        
    def serveSubscribeMotionSrv(self, req):
        self.movementPub = rospy.Publisher("nao_vision/movement_detected", MovementDetected)
        self.sensitivitySrv = rospy.Service("nao_vision/movement_detection_sensitivity", VisionMotionSensitivity, self.handleSensitivityChangeSrv )
        self.memProxy.subscribeToEvent("MovementDetection/MovementDetected", self.moduleName, "onMovementDetected")
        self.motion_detection_enabled = True
        
    def serveUnsubscribeMotionSrv(self, req):
        if self.motion_detection_enabled:
            self.memProxy.unsubscribeToEvent("MovementDetection/MovementDetected", self.moduleName)
            self.movementPub.unregister()
            self.sensitivitySrv.shutdown()
            self.motion_detection_enabled = False
        
    def serveSubscribeLandmarkSrv(self, req):
        self.landmarkPub = rospy.Publisher("nao_vision/landmark_detected", LandmarkDetected)
        self.memProxy.subscribeToEvent("LandmarkDetected", self.moduleName, "onLandmarkDetected")
        self.landmarkDetectionProxy.updatePeriod("ROSNaoVisionModuleLandmarkDetected", 200)
        self.landmark_detection_enabled = True
        
    def serveUnsubscribeLandmarkSrv(self, req):
        if self.landmark_detection_enabled:
            self.memProxy.unsubscribeToEvent("LandmarkDetected", self.moduleName)
            self.landmarkPub.unregister()
            self.landmark_detection_enabled = False

    def learnFaceSrv(self, req):
        res = LearnFaceResponse()
        res.result.data = self.faceDetectionProxy.learnFace(req.name.data)
        return res

    def forgetPersonSrv(self, req):
        res = LearnFaceResponse()
        res.result.data = self.faceDetectionProxy.forgetPerson(req.name.data)
        return res
        
if __name__ == '__main__':
  
    ROSNaoVisionModule = NaoVisionInterface("ROSNaoVisionModule")
    rospy.spin()

    rospy.loginfo("Stopping ROSNaoVisionModule ...")
    ROSNaoVisionModule.shutdown();        
    rospy.loginfo("ROSNaoVisionModule stopped.")
    exit(0)
