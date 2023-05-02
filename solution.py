import pyrosim.pyrosim as pyrosim
import numpy as np
import os 
class Solution:

    def __init__(self,myID):
        self.weights = np.random.rand(3,2) * 2  - 1
        self.fitness = 100000.0
        self.myID = myID

        # creating initial fitness file
        with open(f"fitness{self.myID}.txt","w+") as f:
            f.write(f"{self.fitness}")

        pass

    def evolve(self):
        pass

    def evaluate(self,display):
        self.Create_Body()
        self.Create_Brain()
        self.Create_World()

        print(f"run {self.myID}")
        os.system(f"START /B python3 simulation.py {display} {self.myID}")
        with open(f"fitness{self.myID}.txt","r+") as f:
            self.fitness = float(f.read())

    def Create_Body(self):
        pyrosim.Start_URDF(f"body{self.myID}.urdf")

        scale = 1
        length,width,height = 1*scale,1*scale,1*scale
        vertDist = height/2

        pyrosim.Send_Cube(name=f"Torso", pos=[0,0,height+vertDist] , size=[length,width,height])
        pyrosim.Send_Cube(name=f"FrontLeg", pos=[length/2,0,-vertDist] , size=[length,width,height])
        pyrosim.Send_Cube(name=f"BackLeg", pos=[-length/2,0,-vertDist] , size=[length,width,height])
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [length/2,0,height])
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-length/2,0,height])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = f"Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = f"FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = f"BackLeg")
        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        for n in range(0,self.weights.shape[0]):
            for m in range(0,self.weights.shape[1]):
                pyrosim.Send_Synapse( sourceNeuronName = n , targetNeuronName = m + self.weights.shape[0], weight = self.weights[n][m])

        pyrosim.End()

    def Create_World(self):
        pyrosim.Start_SDF(f"world{self.myID}.sdf")
        length,width,height = 0.2,0.2,0.2
        x,y,z = 1,1,height/2

        pyrosim.Send_Cube(name=f"Box1", pos=[x,y,z] , size=[length,width,height])
        pyrosim.End()
