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


if __name__=="__main__":
    pyrosim.Start_SDF("box.sdf")

    for n in range(50):
        generateTower(10,random.random()*2 -1 ,random.random()*2-1)

    pyrosim.End()