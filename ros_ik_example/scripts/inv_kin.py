#!/usr/bin/env python3
import pdb
from urdf_parser_py.urdf import URDF
import PyKDL as kdl
import numpy as np
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Header
import rospy


def trigger_response(request):
    return TriggerResponse(
        success=True,
        message="Hey, roger that; we'll be right there!"
    )


class Solver:
    """ Class that insntatientes
    - Listener of joint_sates
    - Listener of desired_pose
    - Service provider solve_ik
    - Publisher desired_joint_states
    """

    def __init__(self):

        rospy.init_node("ik_solver")
        rospy.Subscriber("joint_states", JointState, self.joint_sate_listener)
        rospy.Subscriber('desired_pose', Pose, self.pose_listener)

        self.js_pub_ = rospy.Publisher(
            'desired_joint_states', JointState, queue_size=10)
        _ = rospy.Service('solve_ik', Trigger, self.ik_solver)
        self.urdf_ = URDF.from_parameter_server()

        self.actual_joint_state_ = JointState()
        self.desired_pose_ = JointState()

        self.tree_ = kdl_tree_from_urdf_model(self.urdf_)

        self.chain_ = self.tree_.getChain('iiwa_link_0', 'iiwa_link_ee')

        self.fk_calculator_ = kdl.ChainFkSolverPos_recursive(self.chain_)

        self.jacobian_calculator_ = kdl.ChainJntToJacSolver(self.chain_)

    def joint_sate_listener(self, _msg: JointState):
        """ Retrive the initial joint state of the robot"""
        self.actual_joint_state_ = _msg

    def pose_listener(self, _msg: Pose):
        """ Retrive the pose desired for the robot"""
        self.desired_pose_ = _msg

    def get_jac(self, _q: np.array):
        """Compute the Jacobian as numpy object"""

        q_kdl = kdl.JntArray(len(_q))
        for i, q_i in enumerate(_q):
            q_kdl[i] = q_i

        j_kdl = kdl.Jacobian(len(_q))
        self.jacobian_calculator_.JntToJac(q_kdl, j_kdl)

        result = np.zeros((j_kdl.rows(), j_kdl.columns()))
        for i in range(result.shape[0]):
            for j in range(result.shape[1]):
                result[i, j] = j_kdl[i, j]

        return result

    def ik_solver(self, _msg):
        """Solve the inverse kinematics"""

        x_des = np.array([getattr(self.desired_pose_.position, attr)
                          for attr in 'xyz'])
        q_prev = np.array(self.actual_joint_state_.position)

        # pos = self.fk_calculator_.JntToCart(

        js_msg = JointState()
        js_msg.header = Header()
        js_msg.name = self.actual_joint_state_.name

        rate = rospy.Rate(1)  # 10hz
        for _ in range(20):
            x_prev = self.forward_kinematics(q_prev)[:3, -1]
            print('x prev', x_prev)
            jac = self.get_jac(q_prev)

            jac_pos = jac[:3, :]

            d_q = -1*np.linalg.pinv(jac_pos).dot(x_des-x_prev)
            print('d_q', d_q)

            q_next = q_prev + d_q

            js_msg.header.stamp = rospy.Time.now()

            js_msg.position = q_next

            self.js_pub_.publish(js_msg)

            print('--------')
            rate.sleep()
            q_prev = q_next

        return TriggerResponse(
            success=True,
            message="Ok"
        )

    def forward_kinematics(self, _q):
        q_kdl = kdl.JntArray(len(_q))
        for i, q_i in enumerate(_q):
            q_kdl[i] = q_i
        q_kdl
        endeffec_frame = kdl.Frame()
        _ = self.fk_calculator_.JntToCart(q_kdl,
                                          endeffec_frame,
                                          6)
        p = endeffec_frame.p
        M = endeffec_frame.M
        result = np.array([[M[0, 0], M[0, 1], M[0, 2], p.x()],
                           [M[1, 0], M[1, 1], M[1, 2], p.y()],
                           [M[2, 0], M[2, 1], M[2, 2], p.z()],
                           [0,      0,      0,     1]])
        return result


if __name__ == "__main__":
    solver = Solver()
    rospy.spin()
