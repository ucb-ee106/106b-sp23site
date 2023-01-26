import numpy as np

class Lyapunov:
    """
    Skeleton class for Lyapunov functions.
    Includes utilities to get Lyapunov values and Lyapunov derivatives
    """
    def __init__(self, stateDimn, inputDimn, observer, relDegree):
        """
        Init function for a Lyapunov/Barrier object
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            observer (observer): observer object
            relDegree (int): relative degree of Lyapunov function
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.observer = observer
        self.relDegree = relDegree
        
        #parameters to store values
        self._vals = None #stored derivatives + value of function (initialize as None)

    def evalLyapunov(self, t):
        """
        Evaluates the Lyapunov function at time t.
        Args:
            t (float): current time in simulation
        Returns:
            V(x, t): value of Lyapunov function at time t, state x
        """
        return 0
    
    def evalLyapunovDerivs(self, u, t):
        """
        Returns the r derivatives of the Lyapunov function
        Args:
            Args:
            u (numpy array, (input_dimn x 1)): current input vector to system
            t (float): current time in simulation
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return np.zeros((self.observer.dynamics.relDegree + 1, 1))

    
    def eval(self, u, t):
        """
        Returns a list of derivatives (going until zeroth derivative) of the lyapunov/Barrier function.
        Args:
            u (numpy array, (input_dimn x 1)): current input vector to system
            t (float): current time in simulation
        Returns:
        [..., vDDot, vDot, v] ((self.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        self._vals = np.vstack((self.evalLyapunov(t), self.evalLyapunovDerivs(u, t)))
        return self._vals
    
    def get(self):
        """
        Retreives stored function and derivative values
        Returns:
        [..., vDDot, vDot, v] ((self.observer.dynamics.relDegree + 1, 1) numpy Array): dynamics.relDegree lyapunov/Barrier time derivs, Descending order
        """
        return self._vals

"""
********************************
ADD YOUR LYAPUNOV FUNCTIONS HERE
********************************
"""
class LyapunovQrotor(Lyapunov):
    """
    Lyapunov function for point mass trajectory tracking in the quadrotor
    """
    def __init__(self, stateDimn, inputDimn, observer, traj):
        """
        Init function for a Lyapunov/Barrier object
        Args:
            stateDimn (int): length of state vector
            inputDimn (int): length of input vector
            observer (QuadObserver): QuadObserver object
            traj (Trajectory): Trajectory object (required for tracking)
        """
        #initialize skeleton object. The relative degree of this system is 1.
        super().__init__(stateDimn, inputDimn, observer, 1)

        #store the trajectory in a class parameter
        self.traj = traj

        #get the mass of the system from the observer
        self.m = self.observer.dynamics._m

        #store the Lyapunov tuning constants (selected to keep Vx quadratic)
        a = 1 #a may be tuned
        b = (self.m/2)**0.5 #b must be a fixed constant
        self.alpha = 2*a**2
        self.epsilon = 2*(a+b)
    
    def evalLyapunov(self, t):
        """
        Evaluates the Lyapunov function at time t.
        Args:
            t (float): current time in simulation
        Returns:
            V(x, t) (float): value of Lyapunov function at time t, state x
        """
        #get the position and velocity from the observer
        x = self.observer.get_pos()
        v = self.observer.get_vel()

        #get desired position, velocity, and accel from the trajectory
        xD, vD, aD = self.traj.get_state(t)

        #define position and velocity errors
        eX = x - xD
        eV = v - vD

        #extract alpha, epsilon, and mass
        alpha = self.alpha
        epsilon = self.epsilon
        m = self.m

        """
        ***************************************
        YOUR CODE HERE: Here, you should calculate and return the value of the Lyapunov function, V(x)
        Above, several useful parameters have been defined for you! Your code should return a single float value.
        ***************************************
        """

        return 0 #TODO: Change this to return the value of the Lyapunov function.

    def evalLyapunovDerivs(self, f, t):
        """
        Evaluates the first derivative of the Lyapunov function at time t.
        Args:
            f ((inputDimn x 1) numPy array): input to the system
            t (float): current time in simulation
        Returns:
            Vdot(x, t): first time derivative of Lyapunov function at time t, state x
        """
        #get the position and velocity from the observer
        x = self.observer.get_pos()
        v = self.observer.get_vel()

        #get desired position, velocity, and accel from the trajectory
        xD, vD, aD = self.traj.get_state(t)

        #define position and velocity errors
        eX = x - xD
        eV = v - vD

        #extract alpha, epsilon, and mass
        alpha = self.alpha
        epsilon = self.epsilon
        m = self.m

        """
        ***************************************
        YOUR CODE HERE: Here, you should calculate and return the value of the first time derivative of the Lyapunov function
        Above, several useful parameters have been defined for you!
        NOTE: In this function, do not use np.matmul for multiplication, use the @ symbol. To find the square of the norm of a vector, use x.T @ x.
        ***************************************
        """

        #calculate derivative of Lyapunov function
        return 0 #TODO: update this line