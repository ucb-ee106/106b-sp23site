import numpy as np

class LyapunovBarrier:
    """
    Skeleton class for Lyapunov/Barrier functions.
    Includes utilities to get Lyapunov/Barrier values and Lyapunov/Barrier derivatives
    """
    def __init__(self, stateDimn, inputDimn, dynamics):
        """
        Init function for a Lyapunov/Barrier object
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): dynamics object
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.dynamics = dynamics
        
        #parameters to store values
        self._vals = None #stored derivatives + value of function (initialize as None)
    
    def eval(self, u, t):
        """
        Returns a list of derivatives (going until zeroth derivative) of the lyapunov/Barrier function.
        Args:
            u (numpy array, (input_dimn x 1)): current input vector to system
            t (float): current time in simulation
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        self._vals = np.zeros((self.dynamics.relDegree + 1, 1))
        return self._vals
    
    def get(self):
        """
        Retreives stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals

"""
********************************
ADD YOUR LYAPUNOV FUNCTIONS HERE
********************************
"""

class DoubleIntLyapunov(LyapunovBarrier):
    def __init__(self, stateDimn, inputDimn, dynamics, observer, trajectory):
        """
        Double integrator system Lyapunov function.
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): Dynamics object
            observer (Observer): Observer object
            trajectory (Trajectory): traj object
        """
        super().__init__(stateDimn, inputDimn, dynamics)
        self._goal_pt = None
        self.observer = observer
        self.trajectory = trajectory
        
        #tuning parameter
        self.EPSILON = 1
    
    def eval(self, u, t):
        #get goal point, velocity, acceleration
        xG, vG, aG = self.trajectory.get_state(t)
        
        #first, get the spatial and velocity vectors from the observer
        x = self.observer.get_pos()
        v = self.observer.get_vel()
        
        #evaluate lyapunov function
        v = (0.5*((x-xG).T@(x-xG)) + 0.5*((v-vG).T@(v-vG))+self.EPSILON*((x-xG).T@(v-vG)))[0, 0]
        vDot = 0 #TODO: Implement these derivatives
        vDDot = 0
        self._vals = np.array([[vDDot, vDot, v]])
        return self._vals
    
"""
********************************
ADD YOUR BARRIER FUNCTIONS HERE
********************************
"""

class PointBarrier(LyapunovBarrier):
    def __init__(self, stateDimn, inputDimn, dynamics, observer, buffer):
        """
        Double integrator system Lyapunov function.
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            dynamics (Dynamics): dynamics object
            observer (DoubleIntObserver): observer object
        """
        super().__init__(stateDimn, inputDimn, dynamics)
        self._barrierPt = None
        self._buffer = buffer #barrier buffer
        self.observer = observer #store the system observer
        
    def set_barrier_pt(self, pt):
        """
        Function to update the point used in the barrier function (in the world frame)
        Args:
            pt (3 x 1 numpy array): new point to be used for a barrier function, (x, y, z) position
        """
        self._barrierPt = pt
    
    def get_barrier_pt(self):
        """
        Retreive the barrier point from the class attribute
        """
        return self._barrierPt
    
    def eval(self, u, t):
        """
        Evaluate the Euclidean distance to the barrier point.
        Args:
            u (input_dimn x 1 numpy array): current input vector
        Returns:
            (List): cbf time derivatives
        """
        #first, get the spatial and velocity vectors from the observer
        x = self.observer.get_pos()
        v = self.observer.get_vel()
        
        #evaluate the barrier function value
        h = ((x - self._barrierPt).T@(x - self._barrierPt))[0, 0] - self._buffer**2
        
        #evaluate its first derivative - assume a still obstacle
        hDot = (2*v.T@(x - self._barrierPt))[0, 0]
        
        #evaluate its second derivative - assume double integrator point mass dynamics
        xddot = u #pull directly from the force vector, double integrator system
        hDDot = 2*(xddot.T@(x - self._barrierPt) + v.T@v)
        
        #return the two derivatives and the barrier function
        self._vals = [hDDot, hDot, h]
        return self._vals
