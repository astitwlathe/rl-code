import gym
from gym import error, spaces, utils
from gym.utils import seeding

import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import random
import math
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, wait_for_user, load_pybullet, get_joint_positions, link_from_name
from pybullet_planning import Pose, Point, set_pose, Euler, RED, create_box, apply_alpha, create_attachment

class AumrTahoma(gym.Env):
    def __init__(self):
        '''
        z = 1.947(-.125) <-> 2.131(-.125)
        x = .167 (-.125) <-> .36
        y = .13 (-.125) <-> 28 (-.125)

        '''
        p.connect(p.GUI)
        p.resetSimulation()
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=[3.5,-1,2.5])
        # self.block = create_box(1, 1, 1, color=apply_alpha(RED, 0.5))
        # block_x = 2
        # block_y = 2.
        # block_z = 1.5
        # set_pose(self.block, Pose(Point(x=block_x, y=block_y, z=block_z)))
        urdfRootPath = pybullet_data.getDataPath()        

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0, 0])
        
        self.objectUID0 = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"))
        block_x = .1
        block_y = .1
        block_z = 1.82
        set_pose(self.objectUID0, Pose(Point(x=block_x, y=block_y, z=block_z)))
        
        self.objectUID1 = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/001/001.urdf"))
        block_x = 2
        block_y = .1
        block_z = 1.84
        set_pose(self.objectUID1, Pose(Point(x=block_x, y=block_y, z=block_z)))
        
        self.objectUID2 = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/002/002.urdf"))
        block_x = 3
        block_y = .1
        block_z = 1.82
        set_pose(self.objectUID2, Pose(Point(x=block_x, y=block_y, z=block_z)))

        self.action_space = spaces.Box(np.array([-1]*4), np.array([1]*4))
        self.observation_space = spaces.Box(np.array([-1]*5), np.array([1]*5))
        self.podUID = p.loadURDF("./pod1bin.urdf", useFixedBase=True)    
        # self.robotUID = p.loadURDF("./tahoma.urdf", basePosition=[0.5,-1,0], baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi/2]))
        self.robotUID = p.loadURDF("shelf.urdf", basePosition=[0.5,-1,0], baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi/2]))
        
        self.ik_joints = get_movable_joints(self.robotUID)
        print("Joints", self.ik_joints, "robot uid", self.robotUID)
        p.setGravity(0,0,-10)

    def reset(self):
        
        urdfRootPath = pybullet_data.getDataPath()        
        
        state_object= [random.uniform(1.5,1.8),random.uniform(-0.2,0.2),0.05]
        self.objectUID = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=state_object)
        # ik_joints = get_movable_joints(self.robotUID)
        
        sample_fn = get_sample_fn(self.robotUID, self.ik_joints)
        conf = sample_fn()
        print("Joints", self.ik_joints, "sample configuration", conf,  "Robot UID", self.robotUID)
        set_joint_positions(self.robotUID, self.ik_joints, conf)

    def close(self):
        p.disconnect()

envs = AumrTahoma()
wait_for_user()
envs.reset()
wait_for_user()        
        
# envs.block = create_box(1, 1, 1, color=apply_alpha(RED, 0.5))
# block_x = .1
# block_y = .1
# block_z = 1.83
# set_pose(envs.objectUID, Pose(Point(x=block_x, y=block_y, z=block_z)))

conf = p.calculateInverseKinematics(envs.robotUID, 42, (0.1, -0.1, 1.87))
set_joint_positions(envs.robotUID, envs.ik_joints, conf)
attach = create_attachment(envs.robotUID, 42, envs.objectUID)
attach.assign()
conf = p.calculateInverseKinematics(envs.robotUID, 42, (0.3, -0.7, 1.9))
set_joint_positions(envs.robotUID, envs.ik_joints, conf)
attach.assign()