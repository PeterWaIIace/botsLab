from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
import pybullet as p
import numpy as np

from sensor import Sensor
from actuator import Actuator

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
