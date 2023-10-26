#!/usr/bin/env python3
import numpy as np
from queue import Queue
#from server import Server
#from client import Client

class UVS:
    def __init__(self, tracker, server, client):
        self.tracker = tracker
        self.client = client
        self.server = server
        self.queue = Queue()
        self.jacobian = np.eye(2)  # Initialize Jacobian as 2x2 identity matrix
        self.goal = np.zeros((2,1))
        self.point = np.zeros((2,1))
        self.error = np.zeros((2,1))
        self.lastError = np.zeros((2,1))
        self.deltaError = np.zeros((2,1))
        self.lastDeltaError = np.zeros((2,1))
        self.deltaPoint = np.zeros((2,1))
        self.deltaGoal = np.zeros((2,1))
        self.deltaAngles = np.zeros((2,1))
        self.angles = np.zeros((2,1))
        # self.Kp: Proportional gain in control system. Determines response strength.
        self.Kp = 0.01
        # self.Kd: Derivative gain in control system. Predicts error change, counters overshoot.
        self.Kd = 0.01
    

    def initial_jacobian_estimate(self):
        # Small angle for estimation
        delta_angle = 0.01

        for i in range(2):
            # Get initial end effector position
            initial_position = self.point.copy()

            # Increment angle[i] by a small amount and get new position
            reply=self.server.sendAngles(delta_angle if i == 0 else 0, delta_angle if i == 1 else 0, self.queue)
            if len(reply) != 0:
                self.updateState()  # Update state to get new position
                new_position = self.point.copy()
            else:
                print("Error in initial Jacobian estimation")
                return

            # Calculate change in position
            delta_position = new_position - initial_position

            # Update jacobian column
            self.jacobian[:, i] = delta_position / delta_angle

            # Reset angle[i] back to original value by moving in opposite direction
            reply=self.server.sendAngles(-delta_angle if i == 0 else 0, -delta_angle if i == 1 else 0,self.queue)

    def updateJacobian(self, deltaAngles):
        # Compute change in error from the last iteration.
        deltaError = self.error - self.lastError

        # Compute change in Jacobian using Broyden's update
        deltaX = np.outer(deltaError - np.dot(self.jacobian, deltaAngles), deltaAngles) / np.dot(deltaAngles, deltaAngles)

        # Update Jacobian
        self.jacobian += deltaX

    def computeError(self):
        self.error = self.goal - self.point

    def computeDeltaAngles(self):
        # Compute the transpose of the Jacobian matrix and multiply it with the Jacobian matrix itself
        JTJ = np.matmul(self.jacobian.T,self.jacobian)
        # Compute the transpose of the Jacobian matrix and multiply it with the error vector
        JTerror = np.matmul(self.jacobian.T,self.error)
        
        try:
            # Try to compute the inverse of JTJ
            invJTJ = np.linalg.inv(JTJ)
            # Compute delta angles using the inverse of JTJ, error vector and a constant Kp  
            deltaAngles = -self.Kp*np.matmul(invJTJ,JTerror)
            return deltaAngles
        except np.linalg.LinAlgError:
            # If the inverse computation fails, use least squares method to solve for delta angles
            deltaAngles = -self.Kp*np.linalg.lstsq(JTJ,JTerror,rcond=None)[0]
            return deltaAngles

    def updateState(self):
        pointRaw = self.tracker.point[0:2]
        goalRaw = self.tracker.goal[0:2]
        print("ahahah")
        print(pointRaw)
        print(goalRaw)
        # Assign raw pixel values directly.
        self.point = np.array([[pointRaw[0]],[pointRaw[1]]])
        self.goal = np.array([[goalRaw[0]],[goalRaw[1]]])

        # Compute error.
        self.computeError()
        
    def iterate(self):
       print("Running UVS Iteration")
       # Update state variables.
       self.updateState()
       if np.linalg.norm(self.error) > 1:
           if np.linalg.norm(self.deltaAngles) == 0:
               print("Sending Zero Command")
               reply=self.server.sendAngles(0, 0)
               if reply == "DONE":
                   pass
               elif reply == "RESET":
                   print("Resetting")
                   return
               else:
                   print("Unknown Reply")
                   return
           else:
               print("Updating Jacobian")
               # Update jacobian based on last movement and observed change in error.
               self.updateJacobian(self.deltaAngles)
               
               print("Updated Jacobian: " + str(self.jacobian))
               
               # Compute change in angles needed to reduce error based on current jacobian and error.
               deltaAngles = self.computeDeltaAngles()
               
               print("Computed Delta Angles: " + str(deltaAngles))

               reply=self.server.sendAngles(deltaAngles[0], deltaAngles[1])
               if reply == "DONE":
                   print("Command Executed")
                   # Update angles based on command that was executed.
                   self.angles += deltaAngles
                   self.deltaAngles = deltaAngles
               elif reply == "RESET":
                   print("Resetting")
                   return
               else:
                   print("Unknown Reply")
                   return

    def run(self):
       self.initial_jacobian_estimate()  # Estimate initial Jacobian using orthogonal motions
       while True:
           if np.linalg.norm(self.error) < 1:  # Stopping condition
               break
           else:
               try:
                   self.iterate()
               except Exception as e:
                   print("Error occurred: ", e)
                   break
