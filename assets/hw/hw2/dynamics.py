import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

class Dynamics:
    """
    Skeleton class for system dynamics
    Includes methods for returning state derivatives, plots, and animations
    """
    def __init__(self, x0, stateDimn, inputDimn, relDegree = 1):
        """
        Initialize a dynamics object
        Args:
            x0 (stateDimn x 1 numpy array): initial condition state vector
            stateDimn (int): dimension of state vector
            inputDimn (int): dimension of input vector
            relDegree (int, optional): relative degree of system. Defaults to 1.
        """
        self.stateDimn = stateDimn
        self.inputDimn = inputDimn
        self.relDegree = relDegree
        
        #store the state and input
        self._x = x0
        self._u = None
    
    def get_state(self):
        """
        Retrieve the state vector
        """
        return self._x
        
    def deriv(self, x, u, t):
        """
        Returns the derivative of the state vector
        Args:
            x (stateDimn x 1 numpy array): current state vector at time t
            u (inputDimn x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        return np.zeros((self.state_dimn, 1))
    
    def integrate(self, u, t, dt):
        """
        Integrates system dynamics using Euler integration
        Args:
            u (inputDimn x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
            dt (float): time step for integration
        Returns:
            x (stateDimn x 1 numpy array): state vector after integrating
        """
        #integrate starting at x
        self._x = self.get_state() + self.deriv(self.get_state(), u, t)*dt
        return self._x
    
    def get_plots(self, x, u, t):
        """
        Function to show plots specific to this dynamic system.
        Args:
            x ((stateDimn x N) numpy array): history of N states to plot
            u ((inputDimn x N) numpy array): history of N inputs to plot
            t ((1 x N) numpy array): history of N times associated with x and u
        """
        pass
    
    def show_animation(self, x, u, t):
        """
        Function to play animations specific to this dynamic system.
        Args:
            x ((stateDimn x N) numpy array): history of N states to plot
            u ((inputDimn x N) numpy array): history of N inputs to plot
            t ((1 x N) numpy array): history of N times associated with x and u
        """
        pass
    
"""
**********************************
PLACE YOUR DYNAMICS FUNCTIONS HERE
**********************************
"""
class QuadDyn(Dynamics):
    def __init__(self, x0 = np.zeros((8, 1)), stateDimn = 8, inputDimn = 2, relDegree = 2, m = 0.92, Ixx = 0.0023, l = 0.15):
        """
        Init function for a Planar quadrotor system.
        State Vector: X = [x, y, z, theta, x_dot, y_dot, z_dot, theta_dot]
        Input Vector: U = [F, M]
        
        Args:
            x0 ((8 x 1) NumPy Array): initial state (x, y, z, theta, x_dot, y_dot, z_dot, theta_dot)
            stateDimn (int): dimension of state vector
            inputDimn (int): dimension of input vector
            relDegree (int, optional): relative degree of system
            m (float): mass of quadrotor in kg
            Ixx (float): moment of inertia about x axis of quadrotor
            l (float): length of one arm of quadrotor
        """
        super().__init__(x0, stateDimn, inputDimn, relDegree)
        
        #store physical parameters
        self._m = m
        self._Ixx = Ixx
        self._g = 9.81
        self._l = l
    
    def deriv(self, X, U, t):
        """
        Returns the derivative of the state vector
        Args:
            X (8 x 1 numpy array): current state vector at time t
            U (2 x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        #unpack the input vector
        F, M = U[0, 0], U[1, 0]
        F = max(0, F) #CUT OFF THE FORCE AT ZERO!
        
        #unpack the state vector
        x_dot, y_dot, z_dot = X[4, 0], X[5, 0], X[6, 0] #velocities
        theta, theta_dot = X[3, 0], X[7, 0] #orientations
        
        #calculate the second time derivatives of each
        x_ddot = 0
        y_ddot = (-np.sin(theta)*F)/self._m
        z_ddot = (np.cos(theta)*F - self._m*self._g)/self._m
        theta_ddot = M/self._Ixx
        
        #construct and return state vector        
        return np.array([[x_dot, y_dot, z_dot, theta_dot, x_ddot, y_ddot, z_ddot, theta_ddot]]).T
        
    def show_animation(self, xData, uData, tData, animate = True):
        """
        Shows the animation and visualization of data for this system.
        Args:
            xData (stateDimn x N Numpy array): state vector history array
            u (inputDimn x N numpy array): input vector history array
            t (1 x N numpy array): time history
            animate (bool, optional): Whether to generate animation or not. Defaults to True.
        """
        #Set constant animtion parameters
        GOAL_POS = [1, 2]
        FREQ = 50 #control frequency, same as data update frequency
        L = 0.14 #quadrotor arm length
        
        if animate:
            fig, ax = plt.subplots()
            # set the axes limits
            ax.axis([-1, 2.5, 0, 2.5])
            # set equal aspect such that the circle is not shown as ellipse
            ax.set_aspect("equal")
            # create a point in the axes
            point, = ax.plot(0,1, marker="o")
            num_frames = xData.shape[1]-1

            #define the line for the quadrotor
            line, = ax.plot([], [], 'o-', lw=2)
            
            #plot the goal position
            ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y')
                
            def animate(i):
                y = xData[1, i]
                z = xData[2, i]
                point.set_data(y, z)
                
                #draw the quadrotor line body
                theta = xData[3, i]
                x1 = y + L*np.cos(theta)
                x2 = y - L*np.cos(theta)
                y1 = z + L*np.sin(theta)
                y2 = z - L*np.sin(theta)
                thisx = [x1, x2]
                thisy = [y1, y2]
                line.set_data(thisx, thisy)
                
                return line, point
            
            anim = animation.FuncAnimation(fig, animate, frames=num_frames, interval=1/FREQ*1000, blit=True)
            plt.xlabel("Y Position (m)")
            plt.ylabel("Z Position (m)")
            plt.title("Position of Drone in Space")
            plt.show()
            
        #Plot each state variable in time
        fig, axs = plt.subplots(6)
        fig.suptitle('Evolution of States in Time')
        xlabel = 'Time (s)'
        ylabels = ['Y Pos (m)', 'Z Pos (m)', 'Theta (rad)', 'Y Vel (m/s)', 'Z Vel (m/s)', 'Angular Vel (rad/s)']
        indices = [1, 2, 3, 5, 6, 7] #skip the x coordinates - indices in the state vector to plot
        goalStates = [0, 1, 2, 0, 0, 0, 0, 0]
        n = 0 #index in the subplot
        #plot the states
        for i in indices:
            axs[n].plot(tData.reshape((tData.shape[1], )).tolist(), xData[i, :].tolist())
            #plot the goal state for each
            axs[n].plot(tData.reshape((tData.shape[1], )).tolist(), [goalStates[i]]*tData.shape[1], 'r:')
            axs[n].set(ylabel=ylabels[n]) #pull labels from the list above
            axs[n].grid()
            n += 1
        axs[5].set(xlabel = xlabel)
        plt.show()
        #plot the inputs in a new plot
        fig, axs = plt.subplots(2)
        fig.suptitle('Evolution of Inputs in Time')
        xlabel = 'Time (s)'
        ylabels = ['Force (N)', 'Moment (N*m)']
        for i in range(2):
            axs[i].plot(tData.reshape((tData.shape[1], )).tolist(), uData[i, :].tolist())
            axs[i].set(ylabel=ylabels[i])
            axs[i].grid()
        axs[1].set(xlabel = xlabel)
        plt.show()