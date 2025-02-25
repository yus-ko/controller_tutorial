#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf
import math
from std_msgs.msg import Empty
import yaml

class PublishGoals:
    def __init__(self):
        rospy.init_node('publish_goals', anonymous=True)
        
        yaml_dir = rospy.get_param("~yaml_dir", "example.yaml")
        # YAMLファイルの読み込み
        with open(yaml_dir, 'r') as file:
            self.goals = yaml.safe_load(file)

            for goal in self.goals:
                goal["publisher"] = rospy.Publisher(goal["topic_name"], PoseStamped, queue_size=10, latch=True)
        
        rate = rospy.Rate(1)  
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
            break
        
        # self.publish()
        # rospy.sleep(2)
        # rospy.signal_shutdown("Published once, shutting down.")

    def rpy_to_quaternion(self, roll, pitch, yaw):
        # rpyからクオータニオンを作成
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        # geometry_msgs/Quaternion型に変換
        quaternion_msg = Quaternion()
        quaternion_msg.x = quaternion[0]
        quaternion_msg.y = quaternion[1]
        quaternion_msg.z = quaternion[2]
        quaternion_msg.w = quaternion[3]
        return quaternion_msg

    def publish(self):
        
        for goal in self.goals:
            
            # PoseStampedメッセージの作成
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()  # 現在の時間
            pose_msg.header.frame_id = goal["frame_id"]  # フレームID
            pose_msg.pose.position.x = goal["x"]  # x座標
            pose_msg.pose.position.y = goal["y"]  # y座標
            pose_msg.pose.position.z = 0.0  # z座標
            pose_msg.pose.orientation = self.rpy_to_quaternion(0,0,goal["yaw"]/180.0*math.pi)
            goal["publisher"].publish(pose_msg)

            str = "%s :published" % goal["topic_name"]
            rospy.loginfo(str)

class SubscribeStartPublishGoals:
    def __init__(self):
        rospy.init_node('publish_goals', anonymous=True)
        
        yaml_dir = rospy.get_param("~yaml_dir", "example.yaml")
        # YAMLファイルの読み込み
        with open(yaml_dir, 'r') as file:
            self.goals = yaml.safe_load(file)

            for goal in self.goals:
                goal["publisher"] = rospy.Publisher(goal["topic_name"], PoseStamped, queue_size=10)

            rospy.Subscriber('/start', Empty, self.publish)
            rospy.spin()  # ノードが終了するまで実行を続ける
        

    def rpy_to_quaternion(self, roll, pitch, yaw):
        # rpyからクオータニオンを作成
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        # geometry_msgs/Quaternion型に変換
        quaternion_msg = Quaternion()
        quaternion_msg.x = quaternion[0]
        quaternion_msg.y = quaternion[1]
        quaternion_msg.z = quaternion[2]
        quaternion_msg.w = quaternion[3]
        return quaternion_msg

    def publish(self, msg):
        
        for goal in self.goals:
            
            # PoseStampedメッセージの作成
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()  # 現在の時間
            pose_msg.header.frame_id = goal["frame_id"]  # フレームID
            pose_msg.pose.position.x = goal["x"]  # x座標
            pose_msg.pose.position.y = goal["y"]  # y座標
            pose_msg.pose.position.z = 0.0  # z座標
            pose_msg.pose.orientation = self.rpy_to_quaternion(0,0,goal["yaw"]/180.0*math.pi)
            goal["publisher"].publish(pose_msg)

            str = "%s :published" % goal["topic_name"]
            rospy.loginfo(str)

if __name__ == '__main__':
    try:
        PublishGoals()
    except rospy.ROSInterruptException:
        pass
