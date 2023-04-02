#!/usr/bin/env python3

import rospy
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander
from std_msgs.msg import String, Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospkg
import yaml
from actionlib_msgs.msg import GoalStatusArray

class Hand():

    def __init__(self):
        rospy.init_node('hand_detect_node', anonymous=True)  
        rospy.loginfo("Hand Detect...")
        self._check_moveit_ready()
        self.hand_subscriber = rospy.Subscriber('/plan', Int32MultiArray, self.hand_callback)
        self.hand_finder = HandFinder()
        self.hand_parameters = self.hand_finder.get_hand_parameters()
        self.hand_serial = list(self.hand_parameters.mapping.keys())[0]      
        self.hand_commander = SrHandCommander(hand_parameters=self.hand_parameters,
                                 hand_serial=self.hand_serial)
    
        self.hand_mapping = self.hand_parameters.mapping[self.hand_serial]

        # Hand joints are detected
        self.joints = self.hand_finder.get_hand_joints()[self.hand_mapping]

        self.prev_plan = []
        # get the path to your config file
        self.th_map = {}
        self.ff_map = {}
        self.mf_map = {}
        self.rf_map = {}
        self.lf_map = {}
        
        hand_grasps_path = rospkg.RosPack().get_path('hand_robot') + f"/config/hand_grasps.yaml"
        self._open_yaml(hand_grasps_path)

    def _check_moveit_ready(self):
        moveit_msg = None
        rospy.loginfo("Checking Moveit...")
        while moveit_msg is None and not rospy.is_shutdown():
            try:
                moveit_msg = rospy.wait_for_message("/move_group/status", GoalStatusArray, timeout=1.0)
                rospy.logdebug("Current /move_group/status READY=>" + str(moveit_msg))

            except:
                rospy.logerr("Current /move_group/status not ready yet, retrying for getting moveit")
        rospy.loginfo("Checking Moveit...DONE")

    def _open_yaml(self, path):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        self.th_map = config['th_map']
        self.ff_map = config['ff_map']
        self.mf_map = config['mf_map']
        self.rf_map = config['rf_map']
        self.lf_map = config['lf_map']

    def hand_callback(self, msg):
        if self.prev_plan != msg.data:
            self.prev_plan = msg.data
            th = self.th_map[msg.data[0]]
            ff = self.ff_map[msg.data[1]]
            mf = self.mf_map[msg.data[2]]
            rf = self.rf_map[msg.data[3]]
            lf = self.lf_map[msg.data[4]]

            combined_dict = {}

            for d in [ff, mf, rf, lf, th]:
                combined_dict.update(d)
            self._run(combined_dict)
    
    def _construct_trajectory_point(self, joint_traj, posture, duration):
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.time_from_start = rospy.Duration.from_sec(float(duration))
        for key in joint_traj.joint_names:
            trajectory_point.positions.append(posture[key])
        return trajectory_point


    def _run(self, traj):
        #trajectory_start_time = 1.0
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration.from_sec(float(trajectory_start_time))
        joint_trajectory.joint_names = list(traj.keys())
        joint_trajectory.points = []
        this_trajectory_point = self._construct_trajectory_point(joint_trajectory, traj, 0.05)
        joint_trajectory.points.append(this_trajectory_point)
        self.hand_commander.run_joint_trajectory_unsafe(joint_trajectory, False)

if __name__ == '__main__':
    hand = Hand()
    rospy.spin()



