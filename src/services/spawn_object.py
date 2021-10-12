#!/usr/bin/env python

from uml_hri_nerve_pick_and_place.srv import SpawnObject, SpawnObjectResponse
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_object(req):
    print("Spawning object")

    xml_file = open(req.file_path,'r')
    model_xml = xml_file.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')

    spawn_model_srv = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_srv(req.model_name, model_xml, req.robot_namespace, req.object_pose, req.reference_frame)

    return SpawnObjectResponse(1)

def spawn_object_server():
    rospy.init_node('spawn_object_server')
    s = rospy.Service('spawn_object', SpawnObject, spawn_object)
    print("Ready to spawn object.")
    rospy.spin()

if __name__ == "__main__":
    spawn_object_server()
