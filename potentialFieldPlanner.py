import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from copy import deepcopy
'''
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
'''
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation

    @staticmethod
    def attractive_force(self,target, current):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the
        target joint position and the current joint position

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint
        from the current position to the target position
        """

        ## STUDENT CODE STARTS HERE

        att_f = np.zeros((3, 1))

        d = 0.1
        zeta = 0.01

        x = current - target
        norm = np.linalg.norm(x)
        if norm <= d:    #conic well potential
            att_f = -zeta * x
        else:             # parabolic well potential
            att_f = (-d * zeta * x) / np.linalg.norm(x)

        ## END STUDENT CODE

        return att_f

    @staticmethod
    def repulsive_force(self,obstacle, current):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the
        obstacle and the current joint position

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position
        to the closest point on the obstacle box

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1))
        b = np.array([[0.2,0.2,0.2]]).T
        eta = 0.01  #repulsive field strength
        rho_0 = np.array([0.2,0.2,0.2])
        # radius of disc, region of repulsion    # need to ask
        obstacle = np.transpose(obstacle)
        if np.size(obstacle) == 0:
            rep_f = np.array([0,0,0])

        else:
        # current = current.reshape(3,1)
        # current = np.array([1,3,4]).reshape(3,1)
        # print(current)
            current = current.T
            rho, unitvec = self.dist_point2box(current, obstacle) # distance between robot and the obstacle
            #unit vector = 1 x 3
            # unitvec = unitvec.T
            # unitvec = np.array([unitvec]).T
            if rho == 0:
                rep_f =np.array([1000,1000,1000])
            else:
                prod1 = 1/rho
                prod2 = 1/rho_0

                rep_f = (eta * ((prod1 - prod2) * (1 / (rho ** 2))))

                rep_f = np.array([rep_f])

        ## END STUDENT CODE

        return rep_f

    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point

        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info

        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin * 0.5 + boxMax * 0.5
        p = p.reshape(1,3)


        # Get distance info from point to box boundary

        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], 0]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], 0]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], 0]).T, 1)

        # convert to distance
        distances = np.vstack([dx, dy, dz]).T  # 1 x3
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter - p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    # @staticmethod
    # def dist_point2box(p, box):
    #     """
    #     Helper function for the computation of repulsive forces. Computes the closest point
    #     on the box to a given point
    #
    #     INPUTS:
    #     p - nx3 numpy array of points [x,y,z]
    #     box - 1x6 numpy array of minimum and maximum points of box
    #
    #     OUTPUTS:
    #     dist - nx1 numpy array of distance between the points and the box
    #             dist > 0 point outside
    #             dist = 0 point is on or inside box
    #     unit - nx3 numpy array where each row is the corresponding unit vector
    #     from the point to the closest spot on the box
    #         norm(unit) = 1 point is outside the box
    #         norm(unit)= 0 point is on/inside the box
    #
    #      Method from MultiRRomero
    #      @ https://stackoverflow.com/questions/5254838/
    #      calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
    #     """
    #     # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU
    #
    #     # Get box info
    #     boxMin = np.array([box[0], box[1], box[2]])
    #     boxMax = np.array([box[3], box[4], box[5]])
    #     boxCenter = boxMin*0.5 + boxMax*0.5
    #     p = np.array(p)
    #
    #     # Get distance info from point to box boundary
    #     dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
    #     dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
    #     dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)
    #
    #     # convert to distance
    #     distances = np.vstack([dx, dy, dz]).T
    #     distances = distances.astype(float)
    #     dist = np.linalg.norm(distances, axis=1)
    #
    #     # Figure out the signs
    #     signs = np.sign(boxCenter-p)
    #
    #     # Calculate unit vector and replace with
    #     unit = distances.T / dist[:, np.newaxis] * signs
    #     unit = unit.astype(float)
    #     unit[np.isnan(unit)] = 0
    #     unit[np.isinf(unit)] = 0
    #     return dist, unit

    @staticmethod
    def compute_forces(self,target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum
        of forces (attactive, repulsive) on each joint.

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each
        joint/end effector
        """

        ## STUDENT CODE STARTS HERE

        joint_forces = np.zeros((3, 7))
        # print(np.shape(target))
        f_att = np.zeros((7,3))
        rep_f = np.zeros((7,3))
        target = target.T
        current = current.T
        # obstacle = obstacle.T

        for p in range(0,7):
            f_att[p] = self.attractive_force(self,target[:,p], current[:,p])

        for o in range(0,7):
            rep_f[o] = self.repulsive_force(self,obstacle, current[:,o])

        joint_forces = f_att.T + rep_f.T


        ## END STUDENT CODE

        return joint_forces

    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint
        """

        ## STUDENT CODE STARTS HERE
        J = calcJacobian(q)
        J1 = np.zeros((3,7))
        J2 = np.hstack((J[:3, 0:1], np.zeros((3, 6))))
        J3 = np.hstack((J[:3, 0:2], np.zeros((3, 5))))
        J4 = np.hstack((J[:3, 0:3], np.zeros((3, 4))))
        J5 = np.hstack((J[:3, 0:4], np.zeros((3, 3))))
        J6 = np.hstack((J[:3, 0:5], np.zeros((3, 2))))
        J7 = np.hstack((J[:3, 0:6], np.zeros((3, 1))))
        # J7 = J[:3, 0:7]

        # J1 = np.hstack((J[:3, 0:1], np.zeros((3, 6))))
        # J2 = np.hstack((J[:3, 0:2], np.zeros((3, 5))))
        # J3 = np.hstack((J[:3, 0:3], np.zeros((3, 4))))
        # J4 = np.hstack((J[:3, 0:4], np.zeros((3, 3))))
        # J5 = np.hstack((J[:3, 0:5], np.zeros((3, 2))))
        # J6 = np.hstack((J[:3, 0:6], np.zeros((3, 1))))
        # J7 = J[:3, 0:7]
        F = joint_forces
        F1 = F[:, 0:1]
        F2 = F[:, 1:2]
        F3 = F[:, 2:3]
        F4 = F[:, 3:4]
        F5 = F[:, 4:5]
        F6 = F[:, 5:6]
        F7 = F[:, 6:7]

        joint_torque_1 = J1.T @ F1
        joint_torque_2 = J2.T @ F2
        joint_torque_3 = J3.T @ F3
        joint_torque_4 = J4.T @ F4
        joint_torque_5 = J5.T @ F5
        joint_torque_6 = J6.T @ F6
        joint_torque_7 = J7.T @ F7
        joint_torques = joint_torque_1 + joint_torque_2 + joint_torque_3 + joint_torque_4 + joint_torque_5 + joint_torque_6 + joint_torque_7
        joint_torques = joint_torques.T
        # joint_torques = np.zeros((1, 7))
        # f = self.compute_forces(self,target, obstacle, current)   # NEED TO DO TOMORROW
        # J = J[0:3,:]
        # joint_torques = J.T @ f
        # joint_torques = np.zeros((1,7))

        ## END STUDENT CODE

        return joint_torques

    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets

        """

        ## STUDENT CODE STARTS HERE

        distance = 0
        d = current - target
        distance = np.linalg.norm(d,'fro')

        ## END STUDENT CODE

        return distance

    @staticmethod
    def compute_gradient(self,q, target,map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal
        configuration

        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        ## STUDENT CODE STARTS HERE   #from target and map, calculate the desired orce an calculate torque and then calculate dq

        dq = np.zeros((1, 7))
        alpha = 0.001
        q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
        q_f = target

        J = calcJacobian(q)
        joints_current,T0e = self.fk.forward(q)
        current = joints_current
        joints_target,T0e = self.fk.forward(target)
        forces = self.compute_forces(self,joints_target, map_struct, current)
        torques = self.compute_torques(forces,q)

        unit_vector_torque = torques / (np.linalg.norm(torques))
        dq = alpha*unit_vector_torque


        # threshold = 0.3
        # diff = q[i] - q_f
        # norm = np.linalg.norm(diff)
        # torque = compute_torques(joint_forces,q)
        # norm_torque = np.linalg.norm(torque)
        # unit_vector_torque = torque / norm_torque
        # while (q!= q_f):
        #     if norm > threshold:
        #         q = q[i] + alpha * unit_vector_torque
        #         i = i+1



        ## END STUDENT CODE

        return dq

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes
        start - 1x7 numpy array representing the starting joint angles for a configuration
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles.
        """

        # q_path = np.array([]).reshape(0,7)
        q_path = np.zeros((1000,7))
        # q_steps = np.zeros((1,7))
        q_steps = np.zeros((1000,7))
        q_steps[0] = np.array([start])
        i = 1
        q_path[0] = np.array([start])

        while np.linalg.norm(start - goal) > 0.2:


            ## STUDENT CODE STARTS HERE

            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code

            # Compute gradient
            # TODO: this is how to change your joint angles

            dq = self.compute_gradient(self,q_steps,goal, map_struct)
            # q_path = np.zeros((1,7))
            if i >=0:
                q_steps[i] = q_path[i-1] + dq  # changed to i prevuiusoly
                # q_steps[i] = q_steps[i-1] + dq
                q_path = np.vstack((q_steps))

                i = i+1

                # Termination Conditions
                if np.linalg.norm(start-goal) < 0.2: # TODO: check termination conditions
                    break # exit the while loop if conditions are met!

                # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
                # TODO: Figure out how to use the provided function
                if i>2:
                    joints, T0e = self.fk.forward(q_path[i])
                    # joints = tm1,tm2,tm3,tm4 .. tme
                    line_pt_1 = joints[:7,:]
                    line_pt_2 = joints[1:,:]
                    # map_struct = np.array([map_struct])

                    collision = detectCollision(line_pt_1, line_pt_2, map_struct)
                    collision_inv = ~collision

                    if np.all(collision_inv):
                        pass
                    else:
                        raise TypeError("colliding")

                # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
                # TODO: when detect a local minima, implement a random walk
                if i >5:
                    if np.linalg.norm(q_path[i-2] - q_path[i-1]) < 0.2 and np.linalg.norm(q_path[i-3] - q_path[i-1]) <0.2 and np.linalg.norm(q_path[i-4] - q_path[i-1]) < 0.2:
                        v = np.linspace(0.1,0.8,num = 7) #
                        q_random = np.zeros((7,1))
                        for j in range(0,7):
                            q_random[j] = q_path[i][j] + v[j]   # random step generated and added vi

                        q_random = q_random.T
                        q_path[i] = q_random  # current path is updated with random step.
                        i = i+1


                ## END STUDENT CODE

            return q_path
        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()

    # inputs
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])

    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    # show results
    for i in range(q_path.shape[0]):
        error = PotentialFieldPlanner.q_distance(q, goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error:3.4f}'.format(error=error))

    print("q path: ", q_path)
