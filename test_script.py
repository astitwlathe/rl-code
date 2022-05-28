import pybullet
import os
import sys
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from termcolor import cprint


HERE = os.path.dirname(__file__)
print("Present Dir: ", HERE)
# pp = pybullet.connect(pybullet.GUI)
UR_ROBOT_URDF = os.path.join(HERE, 'robot.urdf')
print("Present Dir + File ", UR_ROBOT_URDF)
# robot = pybullet.loadURDF(UR_ROBOT_URDF)
# load_pybullet(workspace_path, fixed_base=True)
viewer = True
connect(use_gui=viewer)
robot = load_pybullet(UR_ROBOT_URDF, fixed_base=True)
ik_joints = get_movable_joints(robot)
ik_joint_names = get_joint_names(robot, ik_joints)
cprint('Joint {} \ncorresponds to:\n{}'.format(ik_joints, ik_joint_names), 'green')
sample_fn = get_sample_fn(robot, ik_joints)
print("robot = ", robot)
sampled_conf = sample_fn()
# set_joint_positions(robot, ik_joints, [1]*6)


# for i in range(5):
#     # randomly sample a joint conf
#     sampled_conf = sample_fn()
#     set_joint_positions(robot, ik_joints, sampled_conf)
#     cprint('#{} | Conf sampeld: {}'.format(i, sampled_conf), 'green')



print("Initial Configuration: ", sampled_conf, len(sampled_conf))
wait_for_user()


sampled_conf = sample_fn()
path = plan_joint_motion(robot, ik_joints, sampled_conf, self_collisions=False)
if path is not None:        
    print("final Configuration:", sampled_conf)
print("check")
wait_for_user()

if path is None:
    cprint('no plan found', 'red')
    
else:
    wait_for_user('a motion plan is found! Press enter to start simulating!')

# adjusting this number will adjust the simulation speed
time_step = 0.03
for conf in path:
    set_joint_positions(robot, ik_joints, conf)
    wait_for_duration(time_step)

wait_for_user()
