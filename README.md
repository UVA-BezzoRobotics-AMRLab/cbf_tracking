# CBF Tracker: A Soft Actor-Critic-based Control Barrier Adaptation Scheme

[![Jackal_Snake](https://img.youtube.com/vi/lvjgE2bBMsc/0.jpg)](https://www.youtube.com/watch?v=lvjgE2bBMsc)

# Citation
```bibtex
@misc{mohammad2025softactorcriticbasedcontrolbarrier,
      title={Soft Actor-Critic-based Control Barrier Adaptation for Robust Autonomous Navigation in Unknown Environments}, 
      author={Nicholas Mohammad and Nicola Bezzo},
      year={2025},
      eprint={2503.08479},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2503.08479}, 
}
```


# Installation
Since there are several required pacakges and solvers needed to support everything in this repository, it is suggested to install with Docker as everything will be done automatically (with the exception of simulation nodes, since Gazebo is not easy to setup in Docker).

To build the image, run the following command in the top level directory of this repository:

```bash
docker build --tag=cbf_tracking .
```

To run the trajectory generation nodes, you will need to get a WLS Gurobi license file from the [Gurobi Web License Manager](https://license.gurobi.com/manager/licenses). Place the `gurobi.lic` file in the top level directory of the repository and then execute the following command to spin up a container:

```bash
docker run --volume=$PWD/gurobi.lic:/opt/gurobi/gurobi.lic --network=host -it amr_ros_planner
```

Although this MPC code was designed to run with this specific planner, you can hook up your own so long as it publishes a `JointTrajectory` message to the `/reference_trajectory` topic.

# Running Nodes
To run the planner, simply run the following launch file:

```bash
roslaunch cbf_tracking planner_gurobi.launch
```

The planner will wait until an occupancy map is provided to the `/map` topic and an initial start value is published to `/gmapping/odometry`.

To run the MPC tracker, run:
```bash
roslaunch cbf_tracking jackal_mpc_track.launch
```

There are 2 primary MPC implementations for the CBF that can be run. The first is `track_acc_nlopt`, which is the MPC used in the paper and works the best when using the CBF. The second is `track_acc_ipopt`, which works the best when NOT using the CBF. Although the `track_acc_ipopt` does also support the CBFs, IPOPT does not work super well with external SDFs so it can be a bit finicky. 

There is also a trajectory tracking MPC which operates with velocity as input instead of acceleration, `track_vel_ipopt`, but it doesn't command as smooth of motion as `track_vel_ipopt`. 

# Training
If you are interested in training a model to adapt the CBF at runtime, the script `./train/gazebo_run.py` can be used. I ran all of my training using the [BARN dataset](https://github.com/Daffan/the-barn-challenge), so that will need to be installed to use the script out of the box. Alternatively, you can change the logic in `run_sim()` to run any launch file you wish to start up your simulations so long as it is in Gazebo.

Training:
```bash
python gazebo_run.py --train
```

Evaluation:
```bash
python gazebo_run.py --eval
```

You can also optionally use the `--world_idx` to pass in a world number for a specific training setup. 

## Adjusting Parameters
There are several knobs than can be tuned for the trajectory planning process, and they can be found in [planner.yaml](./params/planner.yaml).
