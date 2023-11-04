import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import time
import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()
        self.weights = []

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map

            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)


            ## first quadrant
            # x = np.random.uniform(60, 120)
            # y = np.random.uniform(37.5, 75)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        self.control_len = 0
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:stored
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):

        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            print("x1 is empty")
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            #return 1/np.linalg.norm(tmp1-tmp2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))



    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """
       
        ## TODO #####
        weights = np.array([])

        for i in range(len(self.particles)):
            
          
            # if(i ==0):
            #     self.particles[i].x = self.bob.x
            #     self.particles[i].y = self.bob.y
            #     self.particles[i].heading = self.bob.heading
            #     #print("gaussian kernal SPECIAL", self.weight_gaussian_kernel(readings_robot, self.particles[i].read_sensor()), readings_robot, self.particles[i].read_sensor())

            #     weights = np.append(weights, self.weight_gaussian_kernel(readings_robot, self.particles[i].read_sensor()))

            # else:
            #     if(i <10):
            #         print("gaussian kernal", self.weight_gaussian_kernel(readings_robot, self.particles[i].read_sensor()), readings_robot, self.particles[i].read_sensor())

            weights = np.append(weights, self.weight_gaussian_kernel(readings_robot, self.particles[i].read_sensor(), std=2000))
        weights/= np.sum(weights)
        #print(weights)

        #print('weights', weights, np.sum(weights))
        
        
        return weights
        

    def resampleParticle(self, weights):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####


        # weights = []
        # for i in range(len(self.particles)):
        #     weights.append(self.particles[i].weight)


        # Resample using buckets
        # bucket = [weights[0]]
        # for i in range(1, len(weights)):
        #     bucket.append(bucket[i-1]+weights[i])
        # #print("bucket", bucket)

        # indices = []
        # for i in range(len(self.particles)):
        #     randn = np.random.random()
        #     #print(randn)
        #     for j in range(len(self.particles)):
        #         if bucket[j]>randn:
        #             #print("bucket:", bucket[j])
        #             indices.append(j)
        #             break
        # print("old particles")
        # print("weights")
        # for i in range(len(self.particles)):
        #     print(self.particles[i].xstored, self.particles[i].y, self.particles[i].heading,  weights[i])
        # print("has it shuffled?")
        # for i in range(490, 500):
        #     #self.particles[i].weight = weights[i]
        #     print(self.particles[i].x, self.particles[i].y, self.particles[i].heading)

        indices = np.random.choice(range(len(self.particles)), size=len(self.particles), p=weights)

        for idx in indices:
            #maxi = self.particles[idx]
            #print('index', idx, weights[idx])
            # print( maxi.x, maxi.y, maxi.heading, self.bob.x, self.bob.y, self.bob.heading, maxi.read_sensor(), self.bob.read_sensor() )

            # if idx % 1000 ==0: 
            #     new_part = Particle(np.random.uniform(0, 120), np.random.uniform(0, 75), self.world, np.random.uniform(0, 2*np.pi), weight = weights[idx], sensor_limit = self.sensor_limit, noisy=True)
            # else:
            new_part = Particle(self.particles[idx].x, self.particles[idx].y, self.world, self.particles[idx].heading, weight = weights[idx], sensor_limit = self.sensor_limit, noisy=True)
            new_part.x += np.random.uniform(low=-.2, high = .2)
            new_part.y += np.random.uniform(low=-.2, high = .2)
            new_part.heading += np.random.uniform(low=-.01, high = .01)

            particles_new.append(new_part)

        ###############
        #print("changing particles")
        self.particles = particles_new
        # print("new particles")
        # for i in self.particles:
        #     print(i.x, i.y, i.heading, )


    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####

        #print(len(self.control))
        # r = ode(vehicle_dynamics)
        # r.set_initial_value([self.bob.x, self.bob.y, self.bob.heading, 0, 0])
        # res = r.integrate(r.t + .01)
        # print(res)
        dt = 0.01
        #print("sampling motion model")
        # for i in self.particles:
        #     print(i.x, i.y, i.heading, self.bob.x, self.bob.y)
        # print('ctl length', len(self.control), self.control)
        # print("steps moved", len(self.control)-self.control_len)
        # if(len(self.control) >0):
        #     for i in range(len(self.particles)):
        #         arr = vehicle_dynamics(0, [self.particles[i].x, self.particles[i].y, self.particles[i].heading], self.control[-1][0],self.control[-1][1])
        #         self.particles[i].x += arr[0]*dt*(len(self.control)-self.control_len)
        #         self.particles[i].y += arr[1]*dt*(len(self.control)-self.control_len)
        #         self.particles[i].heading += arr[2]*dt*(len(self.control)-self.control_len)
        #     self.control_len = len(self.control)

        #print("steps moved", )
        for j in range(self.control_len, len(self.control)):
            #print(j)
            for i in range(len(self.particles)):
                arr = vehicle_dynamics(0, [self.particles[i].x, self.particles[i].y, self.particles[i].heading], self.control[j][0],self.control[j][1])
                self.particles[i].x += arr[0]*dt
                self.particles[i].y += arr[1]*dt
                self.particles[i].heading += arr[2]*dt
        self.control_len = len(self.control)
        

        '''
        Motion model with ode()
        '''
        # t0 = 0
        # for i in range(len(self.particles)):
        #     r = ode(vehicle_dynamics)
        #     r.set_initial_value(t0, [self.particles[i].x, self.particles[i].y, self.particles[i].heading])


        

        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        

        print("bob initial", self.bob.x, self.bob.y, self.bob.heading)
        time.sleep(.5)
        print(self.bob.read_sensor())
        # for i in range(5):
        #     self.particles[i].x = self.bob.x
        #     self.particles[i].y = self.bob.y
        #     self.particles[i].heading = self.bob.heading
        bob_x = np.array([])
        bob_y = np.array([])
        bob_ori = np.array([])
        estimate_x = np.array([])
        estimate_y = np.array([])
        estimate_ori = np.array([])
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
            self.world.clear_objects()
            # print('iter count', count)
            self.particleMotionModel()
            # if(count ==0):
            #     time.sleep(.5)
            #     print(self.bob.read_sensor())
            # print("resampling")
            #self.updateWeight(self.bob.read_sensor())
            # print("showing")
            # for i in self.particles:
                
            #     print(i.x, i.y, i.heading, "bob", self.bob.x, self.bob.y, self.bob.heading)   
            self.resampleParticle(self.updateWeight(self.bob.read_sensor()))
            count += 1
                    
            self.world.show_particles(self.particles, show_frequency=15)
            self.world.show_robot(self.bob)
            self.world.show_estimated_location(self.particles)
            
            bob_x = np.append(bob_x, self.bob.x)
            bob_y = np.append(bob_y, self.bob.y)
            bob_ori = np.append(bob_ori, self.bob.heading)
            estimate_x = np.append(estimate_x, self.world.show_estimated_location(self.particles)[0])
            estimate_y = np.append(estimate_y, self.world.show_estimated_location(self.particles)[1])
            estimate_ori = np.append(estimate_ori, self.world.show_estimated_location(self.particles)[2])
            # print('bobx', bob_x)
            # print('estimatex', estimate_x)
            

            # np.save('bob_x', bob_x)
            # np.save('bob_y', bob_y)
            # np.save('bob_ori', bob_ori)
            # np.save('estimate_x', estimate_x)
            # np.save('estimate_y', estimate_y)
            # np.save('estimate_ori', estimate_ori)
            if count == 200:
                np.save('Problem 4 1500 error2', np.sqrt((bob_x-estimate_x)**2+(bob_y-estimate_y)**2))
                np.save('Problem 4 1500 orientation2', estimate_ori )
                break


            ###############
