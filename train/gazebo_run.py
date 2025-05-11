import os
import sys
import time
import yaml
import torch
import rospy
import signal
import rospkg
import argparse
import subprocess
import numpy as np

from tqdm import tqdm
from os.path import join
from gazebo_simulation import GazeboSimulation
from train_realworld import TrainManager, CustomEnv

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [10.0, 0]  # relative to the initial position

bag_process = None
gazebo_process = None
controller_process = None
planner_process = None

bag_dir = None


def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def shutdown():
    global gazebo_process, controller_process, planner_process

    rospy.loginfo("*************** shutting nodes down now! ***************")
    if bag_process is not None:
        bag_process.terminate()
        bag_process.wait()

    if gazebo_process is not None:
        gazebo_process.terminate()
        gazebo_process.wait()

    if controller_process is not None:
        controller_process.terminate()
        controller_process.wait()

    if planner_process is not None:
        planner_process.terminate()
        planner_process.wait()


def myexcepthook(type, value, tb):
    global mvsim_process, controller_process, planner_process, bag_process

    print("*************** shutting nodes down now! ***************")
    if bag_process is not None:
        bag_process.terminate()
        bag_process.wait()

    if gazebo_process is not None:
        gazebo_process.terminate()
        gazebo_process.wait()

    if controller_process is not None:
        controller_process.terminate()
        controller_process.wait()

    if planner_process is not None:
        planner_process.terminate()
        planner_process.wait()

    # print the error
    print(type)
    print(value)


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


def run_sim(args):
    global gazebo_process, controller_process, planner_process, bag_process, INIT_POSITION, GOAL_POSITION

    rospack = rospkg.RosPack()
    base_path = rospack.get_path("cbf_tracking")
    world_path = rospack.get_path("jackal_helper")
    world_name = f"BARN/world_{args.world_idx}.world"

    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

    print(
        ">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<"
        % (world_name)
    )

    #######################################
    # Put your launch file here!
    #######################################
    launch_file = join(world_path, "launch", "gazebo_launch.launch")
    world_name = join(world_path, "worlds", world_name)

    gazebo_process = subprocess.Popen(
        [
            "roslaunch",
            launch_file,
            "world_name:=" + world_name,
            "gui:=" + ("true" if args.gui else "false"),
        ]
    )
    time.sleep(5)

    collision_topic = "/mpc_done"
    gazebo_sim = GazeboSimulation(
        init_position=INIT_POSITION, collision_topic=collision_topic
    )

    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    # due to map / world transform, flip goal_pos coords...
    goal_coor = (
        INIT_POSITION[0] + GOAL_POSITION[1],
        INIT_POSITION[1] + GOAL_POSITION[0],
    )

    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True

    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset()  # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)

    bag_fname = None
    bag_dir = None

    if args.bag:

        from datetime import datetime

        bag_dir = rospy.get_param("/train/bag_dir", "./")
        dt_str = "{:%Y_%m_%d-%H-%M-%S}".format(datetime.now())
        bag_fname = f"world_{args.world_idx}_{dt_str}.bag"
        bag_launch_file = os.path.join(base_path, "launch/record_bag.launch")

        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)

        print("opening bag file at", (os.path.join(bag_dir, bag_fname)))

        bag_process = subprocess.Popen(
            [
                "roslaunch",
                bag_launch_file,
                "path:=" + bag_dir,
                "bag_name:=" + bag_fname,
            ],
            # executable="/usr/bin/zsh",
        )

    time.sleep(5)

    nav_stack_launch_file = None
    nav_stack_launch_file = join(base_path, "launch/training_mpc.launch")
    params = "db_filename:=" + rospy.get_param("train/db_file", "./rl_learning.db")
    if not args.train and not args.eval:
        nav_stack_launch_file = join(base_path, "launch/jackal_mpc_track.launch")
        params = ""

    controller_process = subprocess.Popen(
        ["roslaunch", nav_stack_launch_file, params],
        # executable="/usr/bin/zsh",
    )
    time.sleep(3)

    planner_launch_file = join(base_path, "launch/planner_gurobi.launch")
    planner_process = subprocess.Popen(
        [
            "roslaunch",
            planner_launch_file,
            "barn:=true",
            "barn_dist:=10.0",
            "enable_recover:=false",
        ],
        # executable="/usr/bin/zsh",
    )

    ##########################################################################
    # 2. Start navigation
    ##########################################################################

    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    prog_crash = False
    start_time = curr_time
    # check whether the robot started to move
    while (
        compute_distance(init_coor, curr_coor) < 0.1
        or planner_process.poll() is not None
    ):

        if curr_time - start_time > 60:
            prog_crash = True
            break

        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        time.sleep(0.01)

    # start navigation, check position, time and collision
    start_time = curr_time
    collided = False
    timeout_time = 100

    if planner_process.poll() is not None or prog_crash:
        collided = True

    while (
        compute_distance(goal_coor, curr_coor) > 1
        and not collided
        and curr_time - start_time < timeout_time
    ):
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position

        curr_coor = (pos.x, pos.y)
        print(
            "Time: %.2f (s), x: %.2f (m), y: %.2f (m)"
            % (curr_time - start_time, *curr_coor),
            end="\r",
        )
        collided = gazebo_sim.get_hard_collision()
        while rospy.get_time() - curr_time < 0.1:
            time.sleep(0.01)

    print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")

    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= timeout_time:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" % (status, curr_time - start_time))

    nav_metric = 1 if success else 0
    print("Navigation metric: %.4f" % (nav_metric))

    with open(args.out, "a") as f:
        f.write(
            "%d %d %d %d %.4f %.4f\n"
            % (
                args.world_idx,
                success,
                collided,
                (curr_time - start_time) >= timeout_time,
                curr_time - start_time,
                nav_metric,
            )
        )

    if bag_process is not None:
        bag_process.terminate()
        bag_process.wait()

    gazebo_process.terminate()
    gazebo_process.wait()
    controller_process.terminate()
    controller_process.wait()
    planner_process.terminate()
    planner_process.wait()

    time.sleep(2)

    return success


if __name__ == "__main__":

    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, shutdown)
    sys.excepthook = myexcepthook

    parser = argparse.ArgumentParser(description="test BARN navigation challenge")
    parser.add_argument("--world_idx", type=int, default=0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--out", type=str, default="out.txt")
    parser.add_argument("--bag", action="store_true")
    parser.add_argument("--train", action="store_true")
    parser.add_argument("--eval", action="store_true")
    parser.add_argument("--test_suite", action="store_true")
    args = parser.parse_args()

    if args.train and args.eval:
        print("Cannot train and evaluate at the same time")
        sys.exit(-1)

    rospy.init_node("gym", anonymous=True)  # , log_level=rospy.FATAL)
    # rospy.set_param("/use_sim_time", True)
    rospy.on_shutdown(shutdown)

    # load parameter from yaml file
    rospack = rospkg.RosPack()
    base_path = rospack.get_path("cbf_tracking")

    # test writing to output file to make sure it works
    with open(args.out, "w") as f:
        f.write("Below are the simulation results for the test trials\n")

    trainer = None
    rospy.set_param("/train/is_eval", args.eval)
    if args.train:
        yaml_file = "train.yaml"

        with open(os.path.join(base_path, "params", yaml_file), "r") as f:
            params = yaml.load(f, Loader=yaml.FullLoader)
            for key, value in params.items():
                # set param with namespace /train
                rospy.set_param("/train/" + key, value)

        batch_size = rospy.get_param("/train/batch_size", 512)
        num_updates = rospy.get_param("/train/num_updates", 1000)
        db_filename = rospy.get_param("/train/db_file", "./rl_learning.db")
        model_file = rospy.get_param("/train/model_file", "./sac_policy.pth")
        log_dir = rospy.get_param("/train/log_dir", "./logs")

        state_dim = rospy.get_param("/train/state_dim", 7)
        theta_min = rospy.get_param("/train/theta_min", -np.pi)
        theta_max = rospy.get_param("/train/theta_max", np.pi)
        velocity_min = rospy.get_param("/train/velocity_min", -2.0)
        velocity_max = rospy.get_param("/train/velocity_max", 2.0)
        acc_min = rospy.get_param("/train/acc_min", -5.0)
        acc_max = rospy.get_param("/train/acc_max", 5.0)
        angvel_min = rospy.get_param("/train/angvel_min", -3.141)
        angvel_max = rospy.get_param("/train/angvel_max", 3.141)
        dist_to_obs_min = float(rospy.get_param("/train/dist_to_obs_min", -0.2))
        dist_to_obs_max = float(rospy.get_param("/train/dist_to_obs_max", 100.0))
        head_to_obs_min = rospy.get_param("/train/head_to_obs_min", -np.pi)
        head_to_obs_max = rospy.get_param("/train/head_to_obs_max", np.pi)
        progress_min = rospy.get_param("/train/progress_min", 0.0)
        progress_max = rospy.get_param("/train/progress_max", 1.0)
        h_value_min = rospy.get_param("/train/h_value_min", -100.0)
        h_value_max = rospy.get_param("/train/h_value_max", 100.0)
        alpha_min = rospy.get_param("/train/min_alpha", 0.1)
        alpha_max = rospy.get_param("/train/max_alpha", 10)

        # set logging param to true
        rospy.set_param("/train/logging", True)

        env = CustomEnv()
        env.state_dim = state_dim
        env.theta_min = theta_min
        env.theta_max = theta_max
        env.velocity_min = velocity_min
        env.velocity_max = velocity_max
        env.acc_min = acc_min
        env.acc_max = acc_max
        env.angvel_min = angvel_min
        env.angvel_max = angvel_max
        env.distance_to_obstacle_min = dist_to_obs_min
        env.distance_to_obstacle_max = dist_to_obs_max
        env.heading_to_obstacle_min = head_to_obs_min
        env.heading_to_obstacle_max = head_to_obs_max
        env.progress_min = progress_min
        env.progress_max = progress_max
        env.h_value_min = h_value_min
        env.h_value_max = h_value_max
        env.alpha_min = alpha_min
        env.alpha_max = alpha_max

        trainer = TrainManager(db_filename, log_dir, batch_size, env)
    else:
        rospy.set_param("/train/logging", False)

    # train every 6 worlds
    train_worlds = np.arange(0, 300, 6)

    # random set of 50 eval worlds
    eval_worlds = np.random.choice(300, 50, replace=False)

    # print(world_list)
    if args.test_suite:
        # sample 50 worlds out of 300 excluding 200-229
        eval_worlds = np.sort(eval_worlds)
        for world in eval_worlds:

            for i in range(5):
                args.world_idx = world
                run_sim(args)

    elif args.world_idx >= 0:
        count = 0
        for i in range(1, 40):
            success = run_sim(args)

            if success == 1:
                count += 1
            elif success == 0:
                count = 0
            else:
                print("simulation error occured on trial", i)

            if args.train:
                rospy.loginfo("TRAINING SAC POLICY")
                for _ in tqdm(range(num_updates)):
                    trainer.train_sac()

                trainer.trainer.end_epoch(i)

                # save the trained policy
                torch.save(trainer.policy.state_dict(), model_file)

            if count >= 10:
                break

    else:
        for i in train_worlds:
            args.world_idx = i

            for i in range(5):
                run_sim(args)

                if args.train:
                    rospy.loginfo("TRAINING SAC POLICY")
                    for _ in tqdm(range(num_updates)):
                        trainer.train_sac()

                    trainer.trainer.end_epoch(i)

                    # save the trained policy
                    log_dir = os.path.dirname(model_file)
                    torch.save(trainer.policy.state_dict(), model_file)
                    torch.save(
                        trainer.qf1.state_dict(), os.path.join(log_dir, "qf1.pth")
                    )
                    torch.save(
                        trainer.qf2.state_dict(), os.path.join(log_dir, "qf2.pth")
                    )

        rospy.set_param("/train/logging", False)

        # now that training is done, evaluate the policy
        args.train = False
        args.eval = True

        with open(args.out, "w") as f:
            f.write("Below are the simulation results for the test trials\n")

        eval_worlds = np.sort(eval_worlds)
        for world in eval_worlds:

            for i in range(5):
                args.world_idx = world
                run_sim(args)

    if args.train:
        trainer.close()

    rospy.signal_shutdown("finished")
