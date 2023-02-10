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
class DoubleIntDyn(Dynamics):
    def __init__(self, x0 = np.zeros((6, 1)), stateDimn = 6, inputDimn = 3, relDegree = 2):
        """
        Init function for a Planar double integrator system.
        Inputs are Fx and Fy (no Fz)

        Args:
            x0 (_type_): _description_ (x, y, z, x_dot, y_dot, z_dot)
            stateDimn (_type_): _description_
            inputDimn (_type_): _description_
            relDegree (int, optional): _description_. Defaults to 2.
        """
        super().__init__(x0, stateDimn, inputDimn, relDegree)
        
        #store the linear system dynamics matrices
        self._A = np.hstack((np.zeros((6, 3)), np.vstack((np.eye(3), np.zeros((3, 3))))))
        self._B = np.vstack((np.zeros((3, 3)), np.eye(3)))
    
    def deriv(self, x, u, t):
        """
        Returns the derivative of the state vector
        Args:
            x (6 x 1 numpy array): current state vector at time t
            u (2 x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        return self._A@x + self._B@u
        
    def get_state_spatial(self):
        """
        Returns the purely spatial (X, Y, Z) component of the state vector

        Returns:
            (2x1 numpy array): (x, y, z) position of the system
        """
        return self.get_state()[0:3].reshape((3, 1))
    
    def get_state_velocity(self):
        """
        Returns the velocity component (X_dot, Y_dot, Z_dot) of the state vector

        Returns:
            (2x1 numpy array): (x_dot, y_dot) velocity of the system
        """
        return self.get_state()[2:].reshape((2, 1))
    
    def get_rotation_matrix(self):
        """
        Returns the rotation matrix from the car frame to the spatial frame.
        """
        #use the velocity to get the first component.
        
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
        GOAL_POS = [9, 9]
        OBS_POS = [5, 6]
        OBS_R = 1.5
        FREQ = 50 #control frequency, same as data update frequency
        
        if animate:
            fig, ax = plt.subplots()
            # set the axes limits
            ax.axis([0, GOAL_POS[0]+2.5, 0, GOAL_POS[1]+2.5])
            # set equal aspect such that the circle is not shown as ellipse
            ax.set_aspect("equal")
            # create a point in the axes
            point, = ax.plot(0,1, marker="o")
            num_frames = xData.shape[1]-1

            #plot the obstacle
            circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
            plt.gca().add_patch(circle)
            ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
                
            def animate(i):
                x = xData[0, i]
                y = xData[1, i]
                point.set_data(x, y)
                return point,
            
            anim = animation.FuncAnimation(fig, animate, frames=num_frames, interval=1/FREQ*1000, blit=True)
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.title("Position of Car in Space")
            plt.show()

        #Plot the spatial trajectory of the car
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        xCoords = xData[0, :].tolist() #extract all of the velocity data to plot on the y axis
        yCoords = xData[1, :].tolist()
        ax.plot(xCoords, yCoords)
        ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
        circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
        plt.gca().add_patch(circle)
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Position of Car in Space")
        plt.show()
        
        #Plot each state variable in time
        fig, axs = plt.subplots(6)
        fig.suptitle('Evolution of States and Inputs in Time')
        xlabel = 'Time (s)'
        ylabels = ['X Pos (m)', 'Y Pos (m)', 'X Vel (m/s)', 'Y Vel (m/s)', 'uX (N)', 'uY (N)']
        indices = [0, 1, 3, 4] #skip the z coordinates - indices in the state vector to plot
        n = 0 #index in the subplot
        #plot the states
        for i in indices:
            axs[n].plot(tData.reshape((tData.shape[1], )).tolist(), xData[i, :].tolist())
            axs[n].set(ylabel=ylabels[n]) #pull labels from the list above
            axs[n].grid()
            n += 1
        #plot the inputs
        for i in range(2):
            axs[i+4].plot(tData.reshape((tData.shape[1], )).tolist(), uData[i, :].tolist())
            axs[i+4].set(ylabel=ylabels[i+4])
            axs[i+4].grid()
        axs[5].set(xlabel = xlabel)
        plt.show()


class TurtlebotDyn(Dynamics):
    def __init__(self, x0 = np.zeros((3, 1)), stateDimn = 3, inputDimn = 2, relDegree = 1):
        """
        Init function for a turtlebot system. Inputs are v, omega.

        Args:
            x0 (NumPy Array): (x, y, phi)
            stateDimn (Int): 
            inputDimn (Int): 
            relDegree (Int, optional): Defaults to 2.
        """
        super().__init__(x0, stateDimn, inputDimn, relDegree)

    def deriv(self, x, u, t):
        """
        Returns the derivative of the state vector
        Args:
            x (3 x 1 numpy array): current state vector at time t
            u (2 x 1 numpy array): current input vector at time t
            t (float): current time with respect to simulation start
        Returns:
            xDot: state_dimn x 1 derivative of the state vector
        """
        PHI = x[2, 0]
        return np.array([[np.cos(PHI), 0], [np.sin(PHI), 0], [0, 1]])@u

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
        GOAL_POS = [4, 4]
        OBS_POS = [2, 2.5]
        OBS_R = 0.75
        FREQ = 50 #control frequency, same as data update frequency
        
        if animate:
            fig, ax = plt.subplots()
            # set the axes limits
            ax.axis([0, GOAL_POS[0]+2.5, 0, GOAL_POS[1]+2.5])
            # set equal aspect such that the circle is not shown as ellipse
            ax.set_aspect("equal")
            # create a point in the axes
            point, = ax.plot(0,1, marker="o")
            num_frames = xData.shape[1]-1

            #plot the obstacle
            circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
            plt.gca().add_patch(circle)
            ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
                
            def animate(i):
                x = xData[0, i]
                y = xData[1, i]
                point.set_data(x, y)
                return point,
            
            anim = animation.FuncAnimation(fig, animate, frames=num_frames, interval=1/FREQ*1000, blit=True)
            plt.xlabel("X Position (m)")
            plt.ylabel("Y Position (m)")
            plt.title("Position of Car in Space")
            plt.show()

        #Plot the spatial trajectory of the car
        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        xCoords = xData[0, :].tolist() #extract all of the velocity data to plot on the y axis
        yCoords = xData[1, :].tolist() #remove the last point, get artefacting for some reason
        ax.plot(xCoords[0:-1], yCoords[0:-1])
        ax.scatter([GOAL_POS[0]], [GOAL_POS[1]], color = 'y') #goal position
        circle = plt.Circle((OBS_POS[0], OBS_POS[1]), radius = OBS_R, fc = 'c')
        plt.gca().add_patch(circle)
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Position of Turtlebot in Space")
        plt.show()
        
        #Plot each state variable in time
        fig, axs = plt.subplots(5)
        fig.suptitle('Evolution of States and Inputs in Time')
        xlabel = 'Time (s)'
        ylabels = ['X Pos (m)', 'Y Pos (m)', 'Phi (rad)', "V (m/s)", "Omega (rad/s)"]
        indices = [0, 1, 2]
        n = 0 #index in the subplot
        #plot the states
        for i in indices:
            axs[n].plot(tData.reshape((tData.shape[1], )).tolist()[0:-1], xData[i, :].tolist()[0:-1])
            axs[n].set(ylabel=ylabels[n]) #pull labels from the list above
            axs[n].grid()
            n += 1
        #plot the inputs
        for i in range(2):
            axs[i+3].plot(tData.reshape((tData.shape[1], )).tolist()[0:-1], uData[i, :].tolist()[0:-1])
            axs[i+3].set(ylabel=ylabels[i+3])
            axs[i+3].grid()
        axs[4].set(xlabel = xlabel)
        plt.show()
