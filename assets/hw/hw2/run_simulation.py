#Our dependencies
from environment import *
from dynamics import *
from controller import *
from trajectory import *
from state_estimation import *
from lyapunov import *

#system initial condition
x0 = np.array([[0, 0, 1, 0, 0, 0, 0, 0]]).T #start the quadrotor at 1 M in the air

#create a dynamics object for the double integrator
dynamics = QuadDyn(x0)

#create an observer based on the dynamics object with noise parameters
mean = 0
sd = 0.01
observer = QuadObserver(dynamics, mean, sd)

#create a trajectory
start = np.array([[0, 0, 1]]).T #Pass in simply spatial dimensions into the system
end = np.array([[0, 1, 2]]).T #goal state in space
T = 3 #Period of trajectory
trajectory = Trajectory(start, end, T)

#create a lyapunov function for a 3D particle with a force vector input
lyap = LyapunovQrotor(6, 3, observer, trajectory)

#define CLF gamma
gamma = 0 #TODO: YOU SHOULD TUNE THIS VALUE HERE TO ENSURE SMOOTH TRAJECTORY TRACKING

#create a planar quadrotor controller
controller = PlanarQrotorLyapunov(observer, lyapunov = lyap, trajectory = trajectory, clfGamma = gamma)

#create a simulation environment
env = Environment(dynamics, controller, observer)
env.reset()

#run the simulation
env.run()