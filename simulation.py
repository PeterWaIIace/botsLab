from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
import pybullet as p
import pybullet_data
import numpy as np
import time 
import sys

from sensor import Sensor
from actuator import Actuator
from robot import Robot

class World:

    def __init__(self,directOrGUI):
        # pybullet settings
        if directOrGUI == "GUI":
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # set world parameters
        p.setGravity(0,0,-9.8*10)

        # load world plane
        planeId = p.loadURDF("plane.urdf")
        robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(robotId)
        #load box object
        p.loadSDF("world.sdf")

        self.robot = Robot(robotId)

    def RUN(self):
        #simulate
        for t in range(100):
            p.stepSimulation()

            self.robot.sense(t)
            self.robot.think()
            self.robot.act()

            self.Get_Fitness()
            time.sleep(0.01)

        p.disconnect()

    def Get_Fitness(self):
        self.robot.Get_Fitness()
        pass

if __name__ == "__main__":
    directOrGUI = sys.argv[1]
    simulation = World(directOrGUI)
    simulation.RUN()


