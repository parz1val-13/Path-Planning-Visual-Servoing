#!/usr/bin/env python3

import numpy as np
from server import Server
# from client import Client
from color_tracking import Tracker
from queue import Queue


# color_tracker = Tracker('b', 'r')

# server_host = "169.254.199.204"
# server_port = 9999

# server = Server(server_host, server_port)


class UVS:
    def __init__(self, tracker, server):
        self.tracker = tracker
        self.server = server
        self.queue = Queue()
        self.jacobian = np.eye(2)  # Initialize Jacobian as 2x2 identity matrix
        self.goal = np.zeros((2, 1))
        self.point = np.zeros((2, 1))
        self.error = np.zeros((2, 1))
        self.lastError = np.zeros((2, 1))
        self.deltaError = np.zeros((2, 1))
        self.lastDeltaError = np.zeros((2, 1))
        self.deltaPoint = np.zeros((2, 1))
        self.deltaGoal = np.zeros((2, 1))
        self.deltaAngles = np.zeros((2, 1))
        self.angles = np.zeros((2, 1))
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
            reply = Server.sendAngles(
                delta_angle if i == 0 else 0, delta_angle if i == 1 else 0, self.queue)
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
            reply = Server.sendAngles(-delta_angle if i ==
                                      0 else 0, -delta_angle if i == 1 else 0, self.queue)

    def updateJacobian(self, deltaAngles):
        # Compute change in error from the last iteration.
        deltaError = self.error - self.lastError

        # Compute change in Jacobian using Broyden's update
        deltaX = np.outer(deltaError - np.dot(self.jacobian, deltaAngles),
                          deltaAngles) / np.dot(deltaAngles, deltaAngles)

        # Update Jacobian
        self.jacobian += deltaX

    def computeError(self):
        self.error = self.goal - self.point

    def computeDeltaAngles(self):
        # Compute the transpose of the Jacobian matrix and multiply it with the Jacobian matrix itself
        JTJ = np.matmul(self.jacobian.T, self.jacobian)
        # Compute the transpose of the Jacobian matrix and multiply it with the error vector
        JTerror = np.matmul(self.jacobian.T, self.error)

        try:
            # Try to compute the inverse of JTJ
            invJTJ = np.linalg.inv(JTJ)
            # Compute delta angles using the inverse of JTJ, error vector and a constant Kp
            deltaAngles = -self.Kp*np.matmul(invJTJ, JTerror)
            return deltaAngles
        except np.linalg.LinAlgError:
            # If the inverse computation fails, use least squares method to solve for delta angles
            deltaAngles = -self.Kp*np.linalg.lstsq(JTJ, JTerror, rcond=None)[0]
            return deltaAngles

    def updateState(self):
        pointRaw = self.tracker.point[0:2]
        goalRaw = self.tracker.goal[0:2]

        # Assign raw pixel values directly.
        self.point = np.array([[pointRaw[0]], [pointRaw[1]]])
        self.goal = np.array([[goalRaw[0]], [goalRaw[1]]])

        # Compute error.
        self.computeError()

    def iterate(self):
        print("Running UVS Iteration")
        # Update state variables.
        self.updateState()
        if np.linalg.norm(self.error) > 1:
            if np.linalg.norm(self.deltaAngles) == 0:
                print("Sending Zero Command")
                reply = Server.sendAngles(0, 0)
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

                reply = Server.sendAngles(deltaAngles[0], deltaAngles[1])
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
        # Estimate initial Jacobian using orthogonal motions
        self.initial_jacobian_estimate()
        while True:
            if np.linalg.norm(self.error) < 1:
                break
            else:
                try:
                    self.iterate()
                except Exception as e:
                    print("Error occurred: ", e)
                    break

    def main(self):
        color_tracker = Tracker('b', 'r')

        server_host = '169.254.138.119'
        server_port = 9999

        server = Server(server_host, server_port)
        uvs = UVS(color_tracker, server)
        uvs.run()

    if __name__ == "__main__":
        main()

# # Global Variables
# matrix = np.eye(2)
# end_point = np.zeros((2, 1))
# current_pt = np.zeros((2, 1))
# last_error = np.zeros((2, 1))
# current_error = np.zeros((2, 1))
# angle_changes = np.zeros((2, 1))
# gain_value = 0.01

# empty_queue = Queue()


# def initialize_matrix():
#     global current_pt, matrix
#     angle_delta = 0.09
#     for i in range(2):
#         initial_position = current_pt.copy()
#         response = server.sendAngles(
#             angle_delta if i == 0 else 0, angle_delta if i == 1 else 0, empty_queue)

#         if len(response) != 0:
#             update_position()
#             new_position = current_pt.copy()
#         else:
#             print("Error")
#             return

#         position_diff = new_position - initial_position
#         matrix[:, i] = position_diff / angle_delta

#         server.sendAngles(-angle_delta if i == 0 else 0, -
#                           angle_delta if i == 1 else 0)

#         print(server.sendAngles(-angle_delta if i == 0 else 0, -
#                                 angle_delta if i == 1 else 0))
#     return True

# def jacobian_update(angle_changes):
#     error_change = current_error -


# def broyden_update(delta_err):
#     global matrix, angle_changes
#     delta_x = np.outer(delta_err - np.dot(matrix, angle_changes),
#                        angle_changes) / np.dot(angle_changes, angle_changes)
#     matrix += delta_x


# def calculate_deltas(err):
#     matrix_T = matrix.T
#     matrix_product = np.matmul(matrix_T, matrix)
#     matrix_err_product = np.matmul(matrix_T, err)

#     try:
#         inv_matrix = np.linalg.inv(matrix_product)
#         return -gain_value * np.matmul(inv_matrix, matrix_err_product)
#     except np.linalg.LinAlgError:
#         return -gain_value * np.linalg.lstsq(matrix_product, matrix_err_product, rcond=None)[0]


# def update_position():
#     raw_point = Tracker.point[0:2]
#     goal_raw = Tracker.point[0:2]

#     point = np.array[[raw_point[0], raw_point[1]]]
#     goal = np.array[[goal_raw[0], goal_raw[1]]]

#     # return (goal - point)


# def iterate_servoing(server):
#     global current_pt, last_error, angle_changes
#     print("Running UVS Iteration")

#     current_pt = update_position()
#     err = end_point - current_pt
#     delta_err = err - last_error

#     # Define a stop criteria
#     if np.linalg.norm(err) < 0.01:  # Change this threshold as per your need
#         print("Target reached or close enough.")
#         return False

#     if np.linalg.norm(angle_changes) == 0:
#         response = server.sendAngles(0, 0)
#     else:
#         broyden_update(delta_err)
#         angle_changes = calculate_deltas(err)
#         response = server.sendAngles(angle_changes[0], angle_changes[1])

#     last_error = err
#     return True


# def run_uvs():
#     server = Server(server_host, server_port)
#     # print("hello")
#     if not initialize_matrix(server):
#         print("Matrix initialization failed.")
#         return

#     while iterate_servoing(server):
#         pass


# if __name__ == "__main__":
#     run_uvs()