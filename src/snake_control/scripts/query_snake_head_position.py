#! /usr/bin/env python
import rospy
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from snake_control.msg import snake_head_rel_pos
import numpy as np

rospy.init_node('snake_head_pos_pub')

pos_pub=rospy.Publisher('/snake_head_pos', snake_head_rel_pos)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

pos = snake_head_rel_pos()

model = GetModelStateRequest()
model.model_name = 'robot'
model.relative_entity_name = 'target::link'

model2 = GetModelStateRequest()#to get coordinate of snake head
model2.model_name='robot'
model2.relative_entity_name = ''
r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)
    result2 = get_model_srv(model2)

    x_rel_world = -result.pose.position.x
    y_rel_world = -result.pose.position.y
    z_rel_world = -result.pose.position.z

    qx = result.pose.orientation.x
    qy = result.pose.orientation.y
    qz = result.pose.orientation.z
    qw = result.pose.orientation.w

    rotation_matrix = np.array([[1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
	[2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz,2*qy*qz-2*qx*qw],
	[2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]])  
    
    pos_rel = np.dot(rotation_matrix.T,np.array([x_rel_world, y_rel_world, z_rel_world]))
    pos.x_rel = pos_rel[0]
    pos.y_rel = pos_rel[1] 


    pos_pub.publish(pos)

    r.sleep()
