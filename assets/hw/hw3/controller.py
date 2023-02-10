import numpy as np
import casadi as ca
from collections import deque

"""
File containing controllers 
"""
class Controller:
    def __init__(self, observer, lyapunov = None, trajectory = None, obstacleQueue = None, uBounds = None):
        """
        Skeleton class for feedback controllers
        Args:
            dynamics (Dynamics): system Dynamics object
            observer (Observer): state observer object
            lyapunov (LyapunovBarrier): lyapunov functions, LyapunovBarrier object
            trajectory (Trajectory): trajectory for the controller to track (could just be a constant point!)
            obstacleQueue (ObstacleQueue): ObstacleQueue object, stores all barriers for the system to avoid
            uBounds ((Dynamics.inputDimn x 2) numpy array): minimum and maximum input values to the system
        """
        #store input parameters
        self.observer = observer
        self.lyapunov = lyapunov
        self.trajectory = trajectory
        self.obstacleQueue = obstacleQueue
        self.uBounds = uBounds
        
        #store input
        self._u = None
    
    def eval_input(self, t):
        """
        Solve for and return control input
        Inputs:
            t (float): time in simulation
        Returns:
            u ((Dynamics.inputDimn x 1)): input vector, as determined by controller
        """
        self._u = np.zeros((self.observer.inputDimn, 1))
        return self._u
    
    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        return self._u

class TurtlebotMPC:
    def __init__(self, observer, obstacles, x_d, N = 10):
        """
        Class for a turtlebot model predictive controller
        Args:
            observer (Observer): state observer object
            obstacles (List of Circle Objects): Python list of circular obstacle objects
            x_d ((3x1) NumPy Array): Desired state of the system
            N (Int): Optimization horizon
        """
        #store input parameters
        self.observer = observer
        self.obstacles = obstacles
        self.x_d = x_d

        #store CASADI PARAMETERS
        self.opti = None #instance of ca.Opti()
        self.X = None #optimization variables - initialize as none
        self.U = None
        self.cost = None #optimization cost - initialize as an none
        
        #store solution
        self.opti_sol = None

        #store optimization horizon
        self.N = N

        #store discretization time step
        self.dt = 1/50

    def get_input(self):
        """
        Retrieves input stored in class parameter
        Returns:
            self._u: most recent input stored in class paramter
        """
        return self._u

    def initialize_variables(self):
        """
        Initialize the variables for the optimization
        X Should be a matrix of dimension (3 x N+1). Each column represents one state in the state sequence.
        U Should be a matrix of dimension (2 x N). Each column represents one input in the input sequence.
        """
        """
        ****************************************
        YOUR CODE HERE:
        Declare the optimization variables for X and U
        You can declare an optimization variable with self.opti.variable(shape1, shape2).
        Here, we store opti in a class variable, which is why we need "self"!
        ****************************************
        """
        N = self.N #MPC horizon

        #TODO: Declare these variables
        self.X = ...
        self.U = ...

    def add_input_constraint(self):
        """
        Define input constraints on the velocity, v, and angular velocity, omega, inputs to the system!
        Recall that each entry of the input, u_i = [v, omega].
        The v and omega inputs should be between [-10 and 10].
        If you add a constraint on two elements in Casadi, it will apply the constraint element-wise!
        """
        """
        ****************************************
        YOUR CODE HERE:
        Constrain the v and omega inputs to be within -10 and 10 at all steps in the input sequence!
        Enfore the optimization constraint using self.opti.subject_to(...)
        Hint: Loop over the columns of self.U and apply the constraint to each column by slicing the self.U matrix.
        Tips:
        To get the ith column out of the matrix X, you may use self.X[:, i].
        To ge the ith column out of the matrix U, you may use self.U[:, i].
        When using matrix multiplication in Casadi, remember to use the @ symbol!
        ****************************************
        """
        N = self.N #MPC horizon


    def add_obs_constraint(self):
        """
        Add constraints on the path of the system so that it does not collide with the obstacles.
        self.obstacles contains a list of obstacles! The radius and center of each obstacle may be accessed with:
        obs.get_radius() -> returns the scalar radius of the obstacle
        obs.get_center() -> returns 2x1 NumPy array of center (x, y) position of the obstacle.
        """
        """
        ****************************************
        YOUR CODE HERE:
        Constrain X such that the path never crosses over the obstacles!
        Enfore the optimization constraint using self.opti.subject_to(...)
        Hint: Loop over the columns of self.X and apply the constraint to each column by slicing the self.X matrix.
        Tips:
        To get the ith column out of the matrix X, you may use self.X[:, i].
        To ge the ith column out of the matrix U, you may use self.U[:, i].
        When using matrix multiplication in Casadi, remember to use the @ symbol!
        ****************************************
        """
        N = self.N #MPC horizon

        for obs in self.obstacles:
            #center and radius of the obstacle
            center = obs.get_center()
            radius = obs.get_radius()
            #Apply the obstacle constraint to ALL N+1 columns of X
            #TODO: YOUR CODE HERE
                
    def discrete_dyn(self, q, u):
        """
        Discretized dynamics of the turtlebot.
        Inputs: q(k) = [x, y, phi] (Casadi vector)
                u(k) = [v, omega] (Casadi vector)
        Returns:
                q(k+1) = q(k) + dt*f(q(k), u(k)) - state vector at the next discretized time step (casadi vector)
        """
        #define q dot
        q_dot = ca.vertcat(u[0]*ca.cos(q[2]), u[0]*ca.sin(q[2]), u[1])
        dt = self.dt

        #TODO: YOUR CODE HERE: return q(k+1) using Euler discretization, given q_dot, q(k), and dt.
        return 0

    def add_dyn_constraint(self):
        """
        Enforce the dynamics constraint, xk+1 = f(xk, uk), on each element of the system.
        Enforce the initial condition of the optimization as well.
        """
        N = self.N #MPC horizon

        #get initial condition (x, y, phi) vector
        x0 = self.observer.get_state()

        """
        ****************************************
        YOUR CODE HERE:
        Add the initial condition constraint and the dynamics constraint to all of the entries in X.
        Hint: to apply the dynamics constraint, loop over all of the elements X, and use the function discrete_dyn 
        to get the next step for each x_k, u_k pair.
        Tips:
        To get the ith column out of the matrix X, you may use self.X[:, i].
        To ge the ith column out of the matrix U, you may use self.U[:, i].
        When using matrix multiplication in Casadi, remember to use the @ symbol!
        ****************************************
        """

        #TODO: add initial condition constraint
        

        #TODO: add dynamics constraint
        

    def add_cost(self):
        """
        Compute the cost for the MPC problem.
        """
        N = self.N #MPC horizon

        #get desired state
        xd = self.x_d

        """
        YOUR CODE HERE: 
        You should fill in an expression for self.cost using the cost function for MPC.
        Hint: To compute the sum term, use a for loop from 0 to N-1. Note that the horizon and desired state
        have been extracted above!
        Tips:
        To get the ith column out of the matrix X, you may use self.X[:, i].
        To ge the ith column out of the matrix U, you may use self.U[:, i].
        When using matrix multiplication in Casadi, remember to use the @ symbol!
        """

        #Define P, Q, R matrices - you may use NumPy matrices for these - start with the Identity and go from there!
        P = ...
        Q = ...
        R = ...

        self.cost = ... #TODO: fill in this parameter!


    def add_warm_start(self):
        """
        Warm starts the system with a guess of the Geometric PD controller
        """
        #get inital condition (x, y, phi vector)
        x0 = self.observer.get_state()

        #get desired state
        xd = self.x_d

        N = self.N #MPC horizon
        
        """
        YOUR CODE HERE: Use NumPy to get a guess, called xGuess, for X, a matrix of N+1 states, whose columns
        are guesses for the optimal sequence of states that solve the optimiation problem.
        Your guess can use a linear interpolation between the initial condition and desired state, 
        which have been extracted above for you. Your solution xGuess should be a numpy array:
        [x_0, x_1, ..., x_N], whose columns are the different terms in the optimal sequence.
        Hint: Look at the numpy function linspace.
        """
        #TODO: provide a guess of the matrix of optimal states.
        xGuess = ...

        #Convert to correct Casadi type (do not edit)
        xGuess = ca.DM(xGuess) 
        self.opti.set_initial(self.X, xGuess)
        
    def setup(self):
        """
        Setup optimization problem for the first time
        Inputs:
        quad_state: current state of quadrotor, [pos, vel, rot, ang_vel]
        des_state: desired state of quadrotor
        des_accel: desired acceleration of quadrotor (by default 0)
        """        
        #Define optimization problem
        self.opti = ca.Opti()
        
        #Define optimization variables
        self.initialize_variables()
        
        #Define constraint on/off switches
        input_constr = False #Turn on input constraints
        obs_constr = True #Turn on barrier constraints
        
        #Add constraints
        if input_constr:
            self.add_input_constraint()
        if obs_constr:
            self.add_obs_constraint()

        #Add obstacle constraint
        self.add_dyn_constraint()
            
        #Compute cost
        self.add_cost()
        
        #Add warm start linear interpolation guess
        self.add_warm_start()

    def solve_nlp(self):
        """
        Solve the optimization problem
        """
        #Solve optimization
        self.opti.minimize(self.cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        self.opti.solver("ipopt", option)
        self.opti_sol = self.opti.solve()
        #Extract the optimal u vector
        u_opti = self.opti_sol.value(self.U)
        x_opti = self.opti_sol.value(self.X)
        solCost = self.opti_sol.value(self.cost)
        print(solCost)
        return u_opti, x_opti

    def eval_input(self, t):
        """
        Solve for and return control input
        Inputs:
            t (float): time in simulation
        Returns:
            u ((Dynamics.inputDimn x 1)): input vector, as determined by controller
        """
        #set up the optimization at this step
        self.setup()
            
        #using the updated state, solve the problem
        u_opti, x_opti = self.solve_nlp()
                                
        #store the first input in the sequence in the self._u parameter
        self._u = (u_opti[:, 0]).reshape((2, 1))

        #return the full force vector
        return self._u