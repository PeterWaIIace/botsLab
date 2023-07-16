import pyrosim.pyrosim as pyrosim
import numpy as np
import robot.constants as c
import time
import os

from world.simulation import runWorld

class Solution:

    def __init__(self,myID,vector):
        self.hidden_neurons_size = 10

        #vector2brain
        #TODO: it may require normalization to have same form as previous version `np.random.rand(self.hidden_neurons_size,8) * 2  - 1`
        #TODO: move this code to other function so it is bit cleaner
        self.input_weights  = np.reshape(vector[:9*self.hidden_neurons_size],(9,self.hidden_neurons_size))
        self.output_weights = np.reshape(vector[9*self.hidden_neurons_size:9*self.hidden_neurons_size + self.hidden_neurons_size*8],(self.hidden_neurons_size,8))

        self.fitness = 100000.0
        self.myID = myID

        self.Create_Body()
        self.Create_Brain()
        self.Create_World()

    def Create_Body(self):
        pyrosim.Start_URDF(f"{c.path}body{self.myID}.urdf")

        scale = 1
        length,width,height = 1*scale,1*scale,1*scale
        vertDist = height/2

        x,y,z = 0,0.5,1

        pyrosim.Send_Cube(name=f"Torso", pos=[x,y,z] , size=[length,width,height])
        pyrosim.Send_Cube(name=f"FrontLeg", pos=[0,y,0] , size=[0.2,1,0.2])
        pyrosim.Send_Cube(name=f"BackLeg", pos=[0,-y,0] , size=[0.2,1,0.2])
        pyrosim.Send_Cube(name=f"LeftLeg", pos=[-y,0,0] , size=[1.0,0.2,0.2])
        pyrosim.Send_Cube(name=f"RightLeg", pos=[y,0,0] , size=[1.0,0.2,0.2])
        pyrosim.Send_Cube(name=f"LowerFrontLeg", pos=[0,y,0] , size=[0.2,1,0.2])
        pyrosim.Send_Cube(name=f"LowerBackLeg", pos=[0,-y,0] , size=[0.2,1,0.2])
        pyrosim.Send_Cube(name=f"LowerLeftLeg", pos=[-y,0,0] , size=[1.0,0.2,0.2])
        pyrosim.Send_Cube(name=f"LowerRightLeg", pos=[y,0,0] , size=[1.0,0.2,0.2])
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,y*2,z], jointAxis = "1 0 0")
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,0,z], jointAxis = "1 0 0")
        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-y,y,z], jointAxis = "0 1 0")
        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [y,y,z], jointAxis = "0 1 0")
        pyrosim.Send_Joint(name = "FrontLeg_LowerFrontLeg" , parent= "FrontLeg" , child = "LowerFrontLeg" , type = "revolute", position = [0,y*2,0], jointAxis = "1 0 0")
        pyrosim.Send_Joint(name = "BackLeg_LowerBackLeg" ,   parent= "BackLeg" ,  child = "LowerBackLeg" , type = "revolute", position = [0,-y*2,0], jointAxis = "1 0 0")
        pyrosim.Send_Joint(name = "LeftLeg_LowerLeftLeg" ,   parent= "LeftLeg" ,  child = "LowerLeftLeg" , type = "revolute", position = [-y*2,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Joint(name = "RightLeg_LowerRightLeg" , parent= "RightLeg" , child = "LowerRightLeg" , type = "revolute", position = [y*2,0,0], jointAxis = "0 1 0")

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork(f"{c.path}brain{self.myID}.nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = f"Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = f"FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = f"BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = f"LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = f"RightLeg")
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = f"LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = f"LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 7 , linkName = f"LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 8 , linkName = f"LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name = 9 , jointName  = f"Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 8 , jointName  = f"Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 10 , jointName  = f"Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 11 , jointName  = f"Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 12 ,  jointName = f"FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron(name = 13 , jointName = f"BackLeg_LowerBackLeg")
        pyrosim.Send_Motor_Neuron(name = 14 , jointName = f"LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron(name = 15 , jointName = f"RightLeg_LowerRightLeg")

        for n in range(0,self.hidden_neurons_size):
            pyrosim.Send_Hidden_Neuron( name = 16 + n )

        for n in range(0,self.input_weights.shape[0]):
            for hidden_neuron_index in range(0,self.hidden_neurons_size):
                pyrosim.Send_Synapse( sourceNeuronName = n , targetNeuronName = 16+hidden_neuron_index, weight = self.input_weights[n][hidden_neuron_index])

        for n in range(0,self.output_weights.shape[1]):
            for hidden_neuron_index in range(0,self.hidden_neurons_size):
                pyrosim.Send_Synapse( sourceNeuronName = 16+hidden_neuron_index , targetNeuronName = self.input_weights.shape[0] + n, weight = self.output_weights[hidden_neuron_index][n])

        pyrosim.End()