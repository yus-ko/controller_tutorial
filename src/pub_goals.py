#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
from std_msgs.msg import Empty

def empty_callback(msg):
    publish_pose()

def subscribe_to_empty_topic():
    rospy.init_node('empty_subscriber', anonymous=True)
    rospy.Subscriber('/robot_0/goal', PoseStamped, empty_callback)
    rospy.spin()  # ノードが終了するまで実行を続ける

def rpy_to_quaternion(roll, pitch, yaw):
    # rpyからクオータニオンを作成
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    # geometry_msgs/Quaternion型に変換
    quaternion_msg = Quaternion()
    quaternion_msg.x = quaternion[0]
    quaternion_msg.y = quaternion[1]
    quaternion_msg.z = quaternion[2]
    quaternion_msg.w = quaternion[3]
    return quaternion_msg

def publish_pose():
    pub1 = rospy.Publisher('/robot_1/goal', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/goal', PoseStamped, queue_size=10)
    pub3 = rospy.Publisher('/robot_3/goal', PoseStamped, queue_size=10)

        # PoseStampedメッセージの作成
    pose_msg1 = PoseStamped()
    pose_msg1.header.stamp = rospy.Time.now()  # 現在の時間
    pose_msg1.header.frame_id = "robot_1/odom"  # フレームID
    pose_msg1.pose.position.x = 2.0  # x座標
    pose_msg1.pose.position.y = 1.0  # y座標
    pose_msg1.pose.position.z = 0.0  # z座標
    pose_msg1.pose.orientation = rpy_to_quaternion(0,0,1.57)

    pose_msg2 = PoseStamped()
    pose_msg2.header.stamp = pose_msg1.header.stamp
    pose_msg2.header.frame_id = "robot_2/odom"  # フレームID
    pose_msg2.pose.position.x = 1.3  # x座標
    pose_msg2.pose.position.y = 0.0  # y座標
    pose_msg2.pose.position.z = 0.0  # z座標
    pose_msg2.pose.orientation = rpy_to_quaternion(0,0,1.57)

    pose_msg3 = PoseStamped()
    pose_msg3.header.stamp = pose_msg1.header.stamp
    pose_msg3.header.frame_id = "robot_3/odom"  # フレームID
    pose_msg3.pose.position.x = 2.7  # x座標
    pose_msg3.pose.position.y = 0.0  # y座標
    pose_msg3.pose.position.z = 0.0  # z座標
    pose_msg3.pose.orientation = rpy_to_quaternion(0,0,1.57)


    # メッセージをパブリッシュ
    pub1.publish(pose_msg1)
    pub2.publish(pose_msg2)
    pub3.publish(pose_msg3)

if __name__ == '__main__':
    try:
        subscribe_to_empty_topic()
    except rospy.ROSInterruptException:
        pass
