import os
import copy
import random
import constants as c
from solution import Solution

class Parallel_Hill_Climber:

    def __init__(self):
        self.parents = {} #Solution()
        self.children = {}
        self.next_unique_id = 0
        for _ in range(0,c.populationSize):
            self.parents[self.next_unique_id] = Solution(self.next_unique_id)
            self.next_unique_id+=1
        pass

    def clean(self):
        for key in self.parents.keys():
            self.parents[key].clean_simulation()

    def evolve(self):
        # cleaning simulation
        self.clean()
        self.Show_Best()
        # self.Show_Best()
        for generation in range(c.numberOfGenerations):
            print(generation,c.numberOfGenerations)
            self.evaluate(self.parents)

            self.Evolve_For_One_Generation()

        self.Show_Best()

    def evaluate(self,solutions):
        for key in solutions.keys():
            solutions[key].start_simulation("DIRECT")

        for key in solutions.keys():
            solutions[key].wait_for_simulation_to_end()



    def Evolve_For_One_Generation(self):
        self.spawn()
        self.mutate()
        self.evaluate(self.children)
        self.select()

    def spawn(self):
        for key in self.parents.keys():
            self.children[f"_parent_{key}"] = copy.deepcopy(self.parents[key])

        pass

    def Show_Best(self):
        lowest_fitness = 10000.0
        key_for_lowest_fitness = 0
        for key in self.parents.keys():
            if self.parents[key].fitness < lowest_fitness:
                key_for_lowest_fitness = key
                lowest_fitness = self.parents[key].fitness

        print(f"key_for_lowest_fitness {key_for_lowest_fitness}")
        self.parents[key_for_lowest_fitness].start_simulation("GUI")
        self.parents[key_for_lowest_fitness].wait_for_simulation_to_end()

        # self.parents.evaluate("GUI")

    def mutate(self):
        for key in self.parents.keys():
            maxCol = random.randint(0,self.children[f"_parent_{key}"].weights.shape[0]-1)
            maxRow = random.randint(0,self.children[f"_parent_{key}"].weights.shape[1]-1)
            self.children[f"_parent_{key}"].weights[random.randint(0,maxCol),random.randint(0,maxRow)] = random.random() * 2 - 1.
        pass

    def select(self):
        for key in self.parents.keys():
            if self.parents[key].fitness > self.children[f"_parent_{key}"].fitness:
                self.parents[key] = self.children[f"_parent_{key}"]
    # def evaluate(self):
