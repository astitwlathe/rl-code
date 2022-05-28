import pybullet
robot = pybullet.connect(pybullet.GUI)
robot = pybullet.loadURDF("./pod1.urdf")
robot = pybullet.loadURDF("./tahoma.urdf", basePosition=[0,-1,0])