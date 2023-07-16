import copy
import random
import constants
from solution import Solution
from multiprocessing import Pool

class Hill_Climber:

    def __init__(self):
        self.parent = Solution()
        pass

    def evolve(self):
        self.Show_Best()
        for generation in range(constants.numberOfGenerations):
            print(generation,constants.numberOfGenerations)
            self.Evolve_For_One_Generation()
        self.Show_Best()
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
