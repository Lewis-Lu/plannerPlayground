
from matplotlib.colors import Normalize
import numpy as np
import math

class vehicle:
    def __init__(self) -> None:
        self.L = 2
        self.width = 1.5
        # center of the Mass referred to the rear bean
        self.CoMrefRear = 1.4
        # state vector:         [px,  py,  vx,  vy,  pxdot, pydot, vxdot, vydot, beta, theta]
        self.state = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0,   0.0,   0.0,   0.0,   0.0,  0.0]])
        # 4x4 transformation matrix
        self.homogenousTfMat = np.matrix([  [1.0, 0.0, 0.0, 0.0], 
                                            [0.0, 1.0, 0.0, 0.0], 
                                            [0.0, 0.0, 1.0, 0.0], 
                                            [0.0, 0.0, 0.0, 0.0] ])
        self.deltaTime = 0.01

    
    def update(self, action):
        '''
        @input: action = [a, phi]
        action.a : acceleration
        action.phi : steering angle
        '''
        try:
            vDot = action.a
        except:
            raise NameError("action has no member 'a'")
        try:
            phi = action.phi
        except:
            raise NameError("action has no member 'phi")
        vxDotInput = vDot[0]
        vyDotInput = vDot[1]
        phi = action.phi

        # beta
        self.state[8] = math.tanh((self.L - self.CoMrefRear) / self.L * math.tan(action.phi))
        
        # velocity
        self.state[2] += vxDotInput * self.deltaTime
        self.state[3] += vyDotInput * self.deltaTime
        # position
        self.state[0] += self.state[2] * self.deltaTime
        self.state[1] += self.state[3] * self.deltaTime
        # pdot
        self.state[4] = self.state[2]
        self.state[5] = self.state[3]
        # vdot
        self.state[6] = vxDotInput
        self.state[7] = vyDotInput
        
        dotTheta = np.sqrt(self.state[2]**2 + self.state[3]**2) * math.sin(self.state[8]) / self.CoMrefRear
        self.state[9] += dotTheta * self.deltaTime
        

