import numpy as np
import pyrosim.pyrosim as pyrosim

class Sensor:

    def __init__(self,linkName) -> None:
        self.values = np.zeros(100)
        self.linkName = linkName
        pass

    def getValue(self,t) -> float:
        if t >= len(self.values):
            tmp = self.values
            self.values = np.zeros(len(self.values) + 100)
            self.values[:len(tmp)] = tmp
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        # print(self.values)
