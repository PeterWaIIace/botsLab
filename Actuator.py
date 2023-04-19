import numpy as np
import pyrosim.pyrosim as pyrosim

class Actuator:

    def __init__(self,jointName,frequency,amplitude,offset) -> None:
        self.jointName = jointName
        self.frequency = frequency
        self.amplitude = amplitude
        self.offset = offset
    
        # self.motorValues = amplitude * sin(frequency * i + phaseOffset) # not sure if that is needed?
        

        pass

    def setValue(self,value,robotId,controlMode):
        pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,jointName = self.jointName ,controlMode = controlMode, targetPosition = value, maxForce = 500)
        pass
