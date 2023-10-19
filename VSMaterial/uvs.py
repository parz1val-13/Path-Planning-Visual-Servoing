import numpy as np
from queue import Queue

class UVS:
    def __init__(self, tracker, server):
        self.tracker = tracker
        self.server = server
        self.queue = Queue()
        self.jacobian = np.zeros((2,2))
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
        # Proportional gain
        self.Kp = 0.01
        # Derivative gain
        self.Kd = 0.01
        # Scale factor for converting raw pixel values to cm.
        # Replace 1 with the appropriate value for your application.
        self.scaleFactor = 1

    def updateJacobian(self, deltaAngles):
        # Compute the change in error from the last iteration.
        deltaError = self.error - self.lastError
        # Update each element of the jacobian matrix.
        for i in range(2):
            for j in range(2):
                if deltaAngles[j] != 0:
                    # The change in error divided by the change in angle gives an approximation of the derivative.
                    self.jacobian[i,j] = deltaError[i] / deltaAngles[j]
                else:
                    # If there was no change in angle, then we cannot compute a derivative and instead just keep the old value.
                    pass

    def computeError(self):
        self.error = self.goal - self.point

    def computeDeltaAngles(self):
        JTJ = np.matmul(self.jacobian.T,self.jacobian)
        JTerror = np.matmul(self.jacobian.T,self.error)
        
        try:
            invJTJ = np.linalg.inv(JTJ)
            deltaAngles = -self.Kp*np.matmul(invJTJ,JTerror)
            return deltaAngles
        except np.linalg.LinAlgError:
            deltaAngles = -self.Kp*np.linalg.lstsq(JTJ,JTerror,rcond=None)[0]
            return deltaAngles

    def updateState(self):
        pointRaw = self.tracker.point[0:2]
        goalRaw = self.tracker.goal[0:2]
        
        # Scale raw pixel values to cm.
        self.point = np.array([[pointRaw[0]*self.scaleFactor],[pointRaw[1]*self.scaleFactor]])
        self.goal = np.array([[goalRaw[0]*self.scaleFactor],[goalRaw[1]*self.scaleFactor]])
        
        # Compute error.
        self.computeError()
        
        # Update deltaPoint and deltaGoal.
        self.deltaPoint = self.point - self.lastPoint
        self.deltaGoal = self.goal - self.lastGoal
        
        # Update lastError.
        self.lastError = np.copy(self.error)

    def iterate(self):
        print("Running UVS Iteration")
        
        # Update state variables.
        self.updateState()
        
        if np.linalg.norm(self.error) > 1:
            if np.linalg.norm(self.deltaAngles) == 0:
                print("Sending Zero Command")
                self.server.sendAngles(0, 0, self.queue)
                reply = self.queue.get()
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
                
                # Send computed change in angles to robot arm.
                print("Sending Command")
                self.server.sendAngles(deltaAngles[0], deltaAngles[1], self.queue)
                
                reply = self.queue.get()
                
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
       while True:
           if np.linalg.norm(self.error) < 1:
               break
           else:
               try:
                   self.iterate()
               except Exception as e:
                   print("Error occurred: ", e)
                   break
