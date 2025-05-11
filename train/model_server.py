#!/usr/bin/env python3

import os
import time
import torch
import rospy
import numpy as np
import rlkit.torch.pytorch_util as ptu

from std_msgs.msg import Float32
from rlkit.torch.networks import ConcatMlp
from cbf_tracking.srv import QuerySAC, QuerySACResponse
from rlkit.torch.sac.policies import TanhGaussianPolicy, MakeDeterministic

policy = None


class ModelServer:
    def __init__(self):

        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.hidden_dim = rospy.get_param("/train/hidden_dims", 256)
        self.action_dim = rospy.get_param("/train/action_dim", 1)
        self.state_dim = rospy.get_param("/train/state_dim", 9)

        self.min_alpha_dot = rospy.get_param("/train/min_alpha_dot", -2.0)
        self.max_alpha_dot = rospy.get_param("/train/max_alpha_dot", 2.0)

        self.is_eval = rospy.get_param("/train/is_eval", True)

        self.model_file = rospy.get_param("/train/model_file", "./sac_policy.pth")

        self.theta_min = rospy.get_param("/train/theta_min", -np.pi)
        self.theta_max = rospy.get_param("/train/theta_max", np.pi)
        self.velocity_min = rospy.get_param("/train/velocity_min", -2.0)
        self.velocity_max = rospy.get_param("/train/velocity_max", 2.0)
        self.acc_min = rospy.get_param("/train/acc_min", -5.0)
        self.acc_max = rospy.get_param("/train/acc_max", 5.0)
        self.angvel_min = rospy.get_param("/train/angvel_min", -3.141)
        self.angvel_max = rospy.get_param("/train/angvel_max", 3.141)
        self.dist_to_obs_min = float(rospy.get_param("/train/dist_to_obs_min", -0.2))
        self.dist_to_obs_max = float(rospy.get_param("/train/dist_to_obs_max", 100.0))
        self.head_to_obs_min = rospy.get_param("/train/head_to_obs_min", -np.pi)
        self.head_to_obs_max = rospy.get_param("/train/head_to_obs_max", np.pi)
        self.progress_min = rospy.get_param("/train/progress_min", 0.0)
        self.progress_max = rospy.get_param("/train/progress_max", 1.0)
        self.h_value_min = rospy.get_param("/train/h_value_min", -100.0)
        self.h_value_max = rospy.get_param("/train/h_value_max", 100.0)
        self.min_alpha = rospy.get_param("/train/min_alpha", 0.1)
        self.max_alpha = rospy.get_param("/train/max_alpha", 10.0)

        self.q_pub = rospy.Publisher("q_value", Float32, queue_size=1)

        self.stoch_policy = TanhGaussianPolicy(
            obs_dim=self.state_dim,
            action_dim=self.action_dim,
            hidden_sizes=[self.hidden_dim, self.hidden_dim],
        ).to(ptu.device)
        self.qf1 = ConcatMlp(
            input_size=self.state_dim + self.action_dim,
            output_size=1,
            hidden_sizes=[self.hidden_dim, self.hidden_dim],
        ).to(ptu.device)
        self.qf2 = ConcatMlp(
            input_size=self.state_dim + self.action_dim,
            output_size=1,
            hidden_sizes=[self.hidden_dim, self.hidden_dim],
        ).to(ptu.device)

        # if file exists, load the model. Otherwise use the default model
        model_dir = os.path.dirname(self.model_file)
        fname = self.model_file
        if os.path.exists(fname):
            rospy.loginfo("Loading model from " + fname)
            self.policy.load_state_dict(torch.load(fname))
            # self.qf1.load_state_dict(torch.load(os.path.join(model_dir, "qf1.pth")))
            # self.qf2.load_state_dict(torch.load(os.path.join(model_dir, "qf2.pth")))
        else:
            print("Model file not found at", fname, " using default model")

        self.policy = self.stoch_policy

        if self.is_eval:
            self.policy = MakeDeterministic(self.stoch_policy).to(ptu.device)
            rospy.loginfo("*******************************")
            rospy.loginfo("Policy is in deterministic mode")
            rospy.loginfo("*******************************")

        self.policy.eval().to(ptu.device)
        # self.qf1.eval().to(ptu.device)
        # self.qf2.eval().to(ptu.device)

        rospy.Service("query_sac", QuerySAC, self.query_sac)

    def spin(self):
        rospy.spin()

    def scale_action(self, action, low, high):
        return low + (high - low) * (action + 1) / 2

    def query_sac(self, req):
        obs = np.array(
            [
                self.normalize(req.theta, self.theta_min, self.theta_max),
                self.normalize(req.vel, self.velocity_min, self.velocity_max),
                self.normalize(req.acc, self.acc_min, self.acc_max),
                self.normalize(req.ang_vel, self.angvel_min, self.angvel_max),
                self.normalize(
                    req.obs_dist, self.dist_to_obs_min, self.dist_to_obs_max
                ),
                self.normalize(
                    req.heading_dist, self.head_to_obs_min, self.head_to_obs_max
                ),
                self.normalize(req.progress, self.progress_min, self.progress_max),
                req.h_val,
                self.normalize(req.alpha, self.min_alpha, self.max_alpha),
            ]
        )

        obs = torch.FloatTensor(obs).to(ptu.device)

        start = time.time()
        action_np = self.policy.get_action(obs)

        dist = self.stoch_policy(obs)
        end = time.time()

        print("inference time is:", end - start)
        # print("DIST IS", dist)
        print("DIST PARAMS ARE:", dist.normal_mean, dist.normal_std)
        # action_tensor = torch.FloatTensor(action_np[0]).to(ptu.device)
        # input_tensor = torch.cat([obs, action_tensor], dim=0)

        # q1_val = self.qf1(obs.unsqueeze(0), action_tensor.unsqueeze(1))
        # q2_val = self.qf2(obs.unsqueeze(0), action_tensor.unsqueeze(1))
        # q_val = min(q1_val, q2_val)

        # msg = Float32()
        # msg.data = q_val.item()
        # self.q_pub.publish(msg)

        action = action_np[0]

        resp = QuerySACResponse()
        resp.alpha_dot = self.scale_action(
            action, self.min_alpha_dot, self.max_alpha_dot
        )
        resp.success = True

        return resp

    def normalize(self, value, min_val, max_val):
        return (value - min_val) / (max_val - min_val)


def main():
    rospy.init_node("model_server")
    ptu.set_gpu_mode(True)
    model_server = ModelServer()
    model_server.spin()


if __name__ == "__main__":
    main()
