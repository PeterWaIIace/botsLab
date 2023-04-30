import pyrosim.pyrosim as pyrosim
import random

def generateTower(steps,x,y):
    length,width,height = 0.2,0.2,0.2
    x,y,z = x,y,height/2

    for n in range(steps):

        pyrosim.Send_Cube(name=f"Box{n}", pos=[x,y,z] , size=[length,width,height])

        # first you need to add half of previous length
        z += height/2
        length = length * 0.9
        height = height * 0.9
        width = width * 0.9
        # then you need to move obj half of its own length so it wont spawn inside prev obj
        z += height/2


def Generate_Body():
    pyrosim.Start_URDF("body.urdf")

    scale = 1
    length,width,height = 1*scale,1*scale,1*scale
    vertDist = height/2

    pyrosim.Send_Cube(name=f"Torso", pos=[0,0,height+vertDist] , size=[length,width,height])
    pyrosim.Send_Cube(name=f"FrontLeg", pos=[length/2,0,-vertDist] , size=[length,width,height])
    pyrosim.Send_Cube(name=f"BackLeg", pos=[-length/2,0,-vertDist] , size=[length,width,height])
    pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [length/2,0,height])
    pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-length/2,0,height])

    pyrosim.End()

def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = f"Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = f"FrontLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = f"BackLeg")
    pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

    for n in range(0,3):
        for m in range(3,5):
            pyrosim.Send_Synapse( sourceNeuronName = n , targetNeuronName = m , weight = random.uniform(-1.0, 1.0))

    pyrosim.End()

def Create_Robot():
    Generate_Body()
    Generate_Brain()


def Create_World():
    pyrosim.Start_SDF("world.sdf")
    length,width,height = 0.2,0.2,0.2
    x,y,z = 1,1,height/2

    pyrosim.Send_Cube(name=f"Box1", pos=[x,y,z] , size=[length,width,height])
    pyrosim.End()

if __name__=="__main__":
    Create_World()
    Create_Robot()