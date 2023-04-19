from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
import pybullet as p
import pybullet_data
import numpy as np
import time 

from sensor import Sensor
from Actuator import Actuator

class Robot:

    def __init__(self, robotId):
        self.robotId = robotId
        self.backLegTouch = 0
        self.frontLegTouch = 0
        self.nn = NEURAL_NETWORK("brain.nndf")
        self.sensors = {}
        self.actuators = {}
        self.prepareToSense()
        self.prepareToAct()

    def prepareToAct(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.actuators[jointName] = Actuator(jointName,0,0,0)

    def prepareToSense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = Sensor(linkName)

    def sense(self,t):
        for n,linkName in enumerate(self.sensors.keys()):
            self.sensors[linkName].getValue(t)

    def think(self):
        self.nn.Update()
        self.nn.Print()

    def act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName    = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.actuators[jointName].setValue(desiredAngle,self.robotId,p.POSITION_CONTROL)

class World:

    def __init__(self):
        # pybullet settings
        physicsClient = p.connect(p.GUI)
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

            time.sleep(0.01)

        p.disconnect()

if __name__ == "__main__":
    simulation = World()
    simulation.RUN()


