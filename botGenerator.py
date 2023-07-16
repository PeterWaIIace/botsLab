from robot.solution import Solution
import numpy as np
class botGenerator:

    def __init__(self):
        self.bots = dict()
        self.names = dict()

    def __generateUniqueNames(self,nNames):
        for n in range(nNames):
            self.names[n] = f"bot_{n}"


    def generateVectors(self,nVectors, length):
        self.__generateUniqueNames(nVectors)
        for n in range(nVectors):
            nVectors.append = np.random(length)
        return nVectors

    def generateBots(self,vectors):
        if(len(self.names) == 0):
            self.__generateUniqueNames(len(vectors))

        for n,v in enumerate(vectors):
            self.bots[n] = Solution(self.names[n],v)

        return self.bots

    def remove(self,name):
        pass