from robot.solution import Solution
import numpy as np
class botGenerator:

    def __init__(self):
        self.bots = dict()
        self.names = dict()
        self.vectors = []

    def __generateUniqueNames(self,nNames):
        for n in range(nNames):
            self.names[n] = f"bot_{n}"


    def generateVectors(self,nVectors, length):
        self.__generateUniqueNames(nVectors)
        for n in range(nVectors):
            self.vectors.append(np.random.rand(length))
        return self.vectors

    def generateBots(self,vectors):
        if(len(self.names) == 0):
            self.__generateUniqueNames(len(vectors))

        for n,v in enumerate(vectors):
            self.bots[n] = Solution(self.names[n],v)

        return self.bots.values()

    def remove(self,name):
        pass