import numpy as np
from scipy.spatial import cKDTree

class StateObserver:
    def __init__(self, dynamics, mean = None, sd = None):
        """
        Init function for state observer

        Args:
            dynamics (Dynamics): Dynamics object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        self.dynamics = dynamics
        self.stateDimn = dynamics.stateDimn
        self.inputDimn = dynamics.inputDimn
        self.mean = mean
        self.sd = sd
        
    def get_state(self):
        """
        Returns a potentially noisy observation of the system state
        """
        if self.mean or self.sd:
            #return an observation of the vector with noise
            return self.dynamics.get_state() + np.random.normal(self.mean, self.sd, (self.stateDimn, 1))
        return self.dynamics.get_state()
    
class QuadObserver(StateObserver):
    def __init__(self, dynamics, mean, sd):
        """
        Init function for state observer for a planar quadrotor

        Args:
            dynamics (Dynamics): Dynamics object instance
            mean (float, optional): Mean for gaussian noise. Defaults to None.
            sd (float, optional): standard deviation for gaussian noise. Defaults to None.
        """
        super().__init__(dynamics, mean, sd)
    
    def get_pos(self):
        """
        Returns a potentially noisy measurement of JUST the position of the Qrotor mass center
        Returns:
            3x1 numpy array, observed position vector of system
        """
        return self.get_state()[0:3].reshape((3, 1))
    
    def get_vel(self):
        """
        Returns a potentially noisy measurement of JUST the spatial velocity of the Qrotor mass center
        Returns:
            3x1 numpy array, observed velocity vector of system
        """
        return self.get_state()[4:7].reshape((3, 1))

    def get_orient(self):
        """
        Returns a potentially noisy measurement of the 
        Assumes that the system is planar and just rotates about the X axis.
        Returns:
            theta (float): orientation angle of quadrotor with respect to world frame
        """
        return self.get_state()[3, 0]
    
    def get_omega(self):
        """
        Returns a potentially noisy measurement of the angular velocity theta dot
        Assumes that the system is planar and just rotates about the X axis.
        Returns:
            theta (float): orientation angle of quadrotor with respect to world frame
        """
        return self.get_state()[7, 0]