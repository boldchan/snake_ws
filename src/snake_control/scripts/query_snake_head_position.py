#! /usr/bin/env python
import rospy
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from snake_control.msg import snake_head_rel_pos

rospy.init_node('snake_head_pos_pub')

pos_pub=rospy.Publisher('/snake_head_pos', snake_head_rel_pos)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

pos = snake_head_rel_pos()

model = GetModelStateRequest()
model.model_name = 'robot'
model.relative_entity_name = 'target::link'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    pos.x_rel = result.pose.position.x
    pos.y_rel = result.pose.position.y

    pos_pub.publish(pos)

    r.sleep()
