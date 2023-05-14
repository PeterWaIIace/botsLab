from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
import constants as c
import pybullet as p
import numpy as np
import os

from sensor import Sensor
from actuator import Actuator

class Robot:

    def __init__(self, robotId,procId):
        self.robotId = robotId
        self.procId = procId
        self.backLegTouch = 0
        self.frontLegTouch = 0
        self.cpg = 3
        self.cpg_freq = 10
        self.nn = NEURAL_NETWORK(f"{c.path}brain{procId}.nndf")
        os.system(f"del {c.path}brain{procId}.nndf")
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
            if self.cpg == n:
                self.sensors[linkName].setValue(t,np.sin(t*self.cpg_freq))
            else:
                self.sensors[linkName].getValue(t)

    def think(self):
        self.nn.Update()
        # self.nn.Print()

    def act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName    = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.actuators[jointName].setValue(desiredAngle,self.robotId,p.POSITION_CONTROL)

    def Save_Fitness(self,fitness):
        with open(f"{c.path}tmp_fitness{self.procId}.txt","w+") as f:
            f.write(f"{fitness}")

        os.rename(f"{c.path}tmp_fitness{self.procId}.txt" , f"{c.path}fitness{self.procId}.txt")
        return fitness

    def Get_Fitness(self):
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition  = basePositionAndOrientation[0]
        xPosition  = basePosition[0]
        # zPosition  = basePosition[2]

        return self.Save_Fitness(xPosition)
        pass