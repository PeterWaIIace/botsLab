from pyrosim.neuralNetwork import NEURAL_NETWORK
import pyrosim.pyrosim as pyrosim
import pybullet as p
import pybullet_data
import numpy as np
import time
import sys
import os

from threading import Thread


class World:

    def __init__(self):

        self.running = False
        # pybullet settings
        self.directOrGUI = "GUI"
        if self.directOrGUI == "GUI":
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # set world parameters
        p.setGravity(0,0,-9.8*10)

        # load world plane
        p.loadURDF("plane.urdf")

    def run(self,steps):

        self.running = True
        for t in range(steps):

            p.stepSimulation()

            if self.directOrGUI == "GUI":
                time.sleep(0.01)

        # TODO: check what disconnect does
        p.disconnect()
        self.running = False

    def isRunning(self):
        return self.running

class Simulation:

    def __init__(self,bots,world):
        self.bots = bots
        self.world = world

    def start(self):
        steps = 500
        self.simulationWorker = Thread(target=self.world.run,args=(steps,))
        self.simulationWorker.start()

    def wait(self):

        while(self.world.isRunning()):
            pass

        self.simulationWorker.join()

