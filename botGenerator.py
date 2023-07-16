from robot.solution import Solution

class botGenerator:

    def __init__(self):
        self.bots = dict()

    def generate(self,numberOfBots):
        for n in range(numberOfBots):
            self.bots[n] = Solution()

        return self.bots

    def remove(self,name):
        pass