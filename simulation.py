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

    def __init__(self,directOrGUI,WorldID):
        self.WorldID = WorldID
        self.directOrGUI = directOrGUI
        # pybullet settings
        if self.directOrGUI == "GUI":
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # set world parameters
        p.setGravity(0,0,-9.8*10)

        # load world plane
        planeId = p.loadURDF("plane.urdf")
        robotId = p.loadURDF(f"body{self.WorldID}.urdf")
        pyrosim.Prepare_To_Simulate(robotId)
        #load box object
        p.loadSDF(f"world{self.WorldID}.sdf")

        self.robot = Robot(robotId,self.WorldID)

    def RUN(self):
        #simulate
        fitness = 10000.0 # some arbitrary max fitness
        for t in range(100):
            p.stepSimulation()

            self.robot.sense(t)
            self.robot.think()
            self.robot.act()

            fitness = self.Get_Fitness()

            if self.directOrGUI == "GUI":
                time.sleep(0.01)

        p.disconnect()
        return fitness

    def Get_Fitness(self):
        self.robot.Get_Fitness()
        pass

def runWorld(directOrGUI,ID):
    directOrGUI = directOrGUI
    simulation = World(directOrGUI,ID)
    fitness = simulation.RUN()
    return fitness

if __name__ == "__main__":
    directOrGUI = sys.argv[1]
    ID = sys.argv[2]
    simulation = World(directOrGUI,ID)
    simulation.RUN()


