import os
import copy
import random
import constants as c
from solution import Solution

class Parallel_Hill_Climber:

    def __init__(self):
        self.parent = {} #Solution()
        for ID in range(0,c.populationSize):
            self.parent[ID] = Solution(ID)
        pass

    def evolve(self):
        # self.Show_Best()
        for i in range(0,c.populationSize):
            self.parent[i].start_simulation("GUI")
        print("started evolving\n===================================================")

        for i in range(0,c.populationSize):
            self.parent[i].wait_for_simulation_to_end()
        print("finished evolving\n===================================================")
        # for generation in range(c.numberOfGenerations):
        #     print(generation,c.numberOfGenerations)
        #     self.Evolve_For_One_Generation()
        # self.Show_Best()
        pass

    def Evolve_For_One_Generation(self):
        self.spawn()
        self.mutate()
        self.child.evaluate("DIRECT")
        print(self.child.fitness,self.parent.fitness)
        self.select()

    def spawn(self):
        self.child = copy.deepcopy(self.parent)
        pass

    def Show_Best(self):
        self.parent.evaluate("GUI")

    def mutate(self):
        maxCol = random.randint(0,self.child.weights.shape[0]-1)
        maxRow = random.randint(0,self.child.weights.shape[1]-1)
        self.child.weights[random.randint(0,maxCol),random.randint(0,maxRow)] = random.random() * 2 - 1.
        pass

    def select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child
        pass
    # def evaluate(self):
