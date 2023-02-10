#Our dependencies
from environment import *
from dynamics import *
from lyapunov_barrier import *
from controller import *
from trajectory import *
from state_estimation import *
from obstacle import *

#system initial condition
q0 = np.array([[0.1, 0, 0]]).T

#create a dynamics object for the double integrator
dynamics = TurtlebotDyn(q0)

#create an observer based on the dynamics object with noise parameters
mean = 0
sd = 0
observer = StateObserver(dynamics, mean, sd)

#create a circular obstacle
r = 0.75
c = np.array([[2, 2.5]]).T
circle = Circle(r, c)

#set a desired state
xD = np.array([[4, 4, 0]]).T

#define MPC Horizon
N = 30

#Create an MPC Controller
controller = TurtlebotMPC(observer, [circle], xD, N = N)

#create a simulation environment
env = Environment(dynamics, controller, observer)

#run the simulation
env.run()