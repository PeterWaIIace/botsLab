import pybullet as p
import pybullet_data
import time 

# pybullet settings
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set world parameters
p.setGravity(0,0,-9.8*10)

# load world plane
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")

#load box object
p.loadSDF("world.sdf")

#simulate
for _ in range(100000):
    p.stepSimulation()
    time.sleep(0.01)

p.disconnect()