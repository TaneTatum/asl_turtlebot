import numpy as np
from numpy import sin, cos
import scipy.linalg    # you may find scipy.linalg.block_diag useful
from ExtractLines import ExtractLines, normalize_line_parameters, angle_difference
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams

class EKF(object):

    def __init__(self, x0, P0, Q):
        self.x = x0    # Gaussian belief mean
        self.P = P0    # Gaussian belief covariance
        self.Q = Q     # Gaussian control noise covariance (corresponding to dt = 1 second)

    # Updates belief state given a discrete control step (Gaussianity preserved by linearizing dynamics)
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def transition_update(self, u, dt):
        g, Gx, Gu = self.transition_model(u, dt)

        #### TODO ####
        # update self.x, self.P
        ##############

        self.x = g
        self.P = np.matmul(Gx,np.matmul(self.P,Gx.T)) + dt*np.matmul(Gu,np.matmul(self.Q,Gu.T))

    # Propagates exact (nonlinear) state dynamics; also returns associated Jacobians for EKF linearization
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: (g, Gx, Gu)
    #      g  - result of belief mean self.x propagated according to the system dynamics with control u for dt seconds
    #      Gx - Jacobian of g with respect to the belief mean self.x
    #      Gu - Jacobian of g with respect to the control u
    def transition_model(self, u, dt):
        raise NotImplementedError("transition_model must be overriden by a subclass of EKF")

    # Updates belief state according to a given measurement (with associated uncertainty)
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def measurement_update(self, rawZ, rawR):
        z, R, H = self.measurement_model(rawZ, rawR)
        if z is None:    # don't update if measurement is invalid (e.g., no line matches for line-based EKF localization)
            return

        #### TODO ####
        # update self.x, self.P
        ##############

        sig = np.matmul(H, np.matmul(self.P, H.T)) + R
        K = np.matmul(self.P, np.matmul(H.T,np.linalg.inv(sig)))
        x = np.array([self.x]).T + np.matmul(K,z)
        self.x = x[:,0]
        self.P = self.P - np.matmul(K, np.matmul(sig, K.T))


    # Converts raw measurement into the relevant Gaussian form (e.g., a dimensionality reduction);
    # also returns associated Jacobian for EKF linearization
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: (z, R, H)
    #       z - measurement mean (for simple measurement models this may = rawZ)
    #       R - measurement covariance (for simple measurement models this may = rawR)
    #       H - Jacobian of z with respect to the belief mean self.x
    def measurement_model(self, rawZ, rawR):
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


class Localization_EKF(EKF):

    def __init__(self, x0, P0, Q, map_lines, tf_base_to_camera, g):
        self.map_lines = map_lines                    # 2xJ matrix containing (alpha, r) for each of J map lines
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Unicycle dynamics (Turtlebot 2)
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x

        #### TODO ####
        # compute g, Gx, Gu
        ##############

        w = om #Rename variable
        if np.absolute(w) > 0.001:
            g = np.array([x + v/w*(sin(w*dt + th) - sin(th)),
                y - v/w*(cos(w*dt + th) - cos(th)),
                th + w*dt])
            Gx = np.array([[1, 0, v/w*(cos(w*dt+th)-cos(th))],
                [0.0, 1.0, v/w*(sin(w*dt+th)-sin(th))],
                [0.0, 0.0, 1.0]])
            Gu = np.array([[1/w*(sin(w*dt + th)-sin(th)), v/(w**2)*(w*dt*cos(w*dt + th)-sin(w*dt + th)+sin(th))],
                [-1/w*(cos(w*dt + th)-cos(th)), v/(w**2)*(w*dt*sin(w*dt + th)+cos(w*dt + th)-cos(th))],
                [0.0, dt]])
        # If w is small
        else:
            g = np.array([x + dt*v*cos(th),
                y + dt*v*sin(th),
                th])
            Gx = np.array([[1, 0, -dt*v*sin(th)],
                [0.0, 1.0, dt*v*cos(th)],
                [0.0, 0.0, 1.0]])
            Gu = np.array([[dt*cos(th), v*(w*dt*cos(w*dt + th)-sin(w*dt + th)+sin(th))],
                [dt*sin(th),v*(w*dt*sin(w*dt + th)+cos(w*dt + th)-cos(th))],
                [0.0, dt]])

        return g, Gx, Gu

    # Given a single map line m in the world frame, outputs the line parameters in the scanner frame so it can
    # be associated with the lines extracted from the scanner measurements
    # INPUT:  m = (alpha, r)
    #       m - line parameters in the world frame
    # OUTPUT: (h, Hx)
    #       h - line parameters in the scanner (camera) frame
    #      Hx - Jacobian of h with respect to the the belief mean self.x
    def map_line_to_predicted_measurement(self, m):
        alpha, r = m

        #### TODO ####
        # compute h, Hx
        ##############

        x, y, th = self.x   # Coordinates of base frame in world frame
        xc, yc, thc = self.tf_base_to_camera # Corrdinates of camera in base frame
        # Coordinates of camera in world frame
        x_ = x + xc*cos(th) - yc*sin(th)
        y_ = y + xc*sin(th) + yc*cos(th)
        th_ = th+thc

        # Eq. 5.94
        h = np.array([alpha - (th_), r-(x_*cos(alpha) + y_*sin(alpha))])
        Hx = np.array([[0, 0, -1],
            [-cos(alpha), -sin(alpha), yc*cos(alpha-th_+thc)-xc*sin(alpha-th_+thc)]])

        ##############
        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Given lines extracted from the scanner data, tries to associate to each one the closest map entry
    # measured by Mahalanobis distance
    # INPUT:  (rawZ, rawR)
    #    rawZ - 2xI matrix containing (alpha, r) for each of I lines extracted from the scanner data (in scanner frame)
    #    rawR - list of I 2x2 covariance matrices corresponding to each (alpha, r) column of rawZ
    # OUTPUT: (v_list, R_list, H_list)
    #  v_list - list of at most I innovation vectors (predicted map measurement - scanner measurement)
    #  R_list - list of len(v_list) covariance matrices of the innovation vectors (from scanner uncertainty)
    #  H_list - list of len(v_list) Jacobians of the innovation vectors with respect to the belief mean self.x
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############

        v_list = []
        R_list = []
        H_list = []

        # Convert world map to expected measurements
        h = np.empty([2,0])
        Hx = []
        for j in range(self.map_lines.shape[1]):
            h_, Hx_ = self.map_line_to_predicted_measurement(self.map_lines[:,j])
            h = np.hstack([h,np.array([h_]).T])
            Hx.append(Hx_)

        # Associate measurement with expected measurement
        I = rawZ.shape[1] # Total number of measuremenets
        for i in range(I):
            v = np.array([rawZ[:,i]]).T - h  # innovation
            d = np.empty([1,0])
            # Claculate Mahalanobis distance for each map feature
            for j in range(v.shape[1]):
                S = np.matmul(Hx[j], np.matmul(self.P, Hx[j].T)) + rawR[i]
                vj = np.array([v[:,j]]).T
                d_ = np.matmul(vj.T, np.matmul(np.linalg.inv(S), vj))
                d = np.hstack([d, d_])
            #Check if distance is within threshold
            if np.amin(d) < self.g**2:
                idx = np.argmin(d)      # Index of smallest d
                v_list.append(v[:,idx]) # Add new innovation to v_list
                R_list.append(rawR[i])
                H_list.append(Hx[idx])

        return v_list, R_list, H_list

    # Assemble one joint measurement, covariance, and Jacobian from the individual values corresponding to each
    # matched line feature
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H
        ##############

        z = np.array([np.hstack(v_list)]).T
        R = scipy.linalg.block_diag(*R_list)
        H = np.vstack(H_list)

        return z, R, H


class SLAM_EKF(EKF):

    def __init__(self, x0, P0, Q, tf_base_to_camera, g):
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Combined Turtlebot + map dynamics
    # Adapt this method from Localization_EKF.transition_model.
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x[:3]

        #### TODO ####
        # compute g, Gx, Gu (some shape hints below)
        # g = np.copy(self.x)
        # Gx = np.eye(self.x.size)
        # Gu = np.zeros((self.x.size, 2))
        ##############

        return g, Gx, Gu

    # Combined Turtlebot + map measurement model
    # Adapt this method from Localization_EKF.measurement_model.
    #
    # The ingredients for this model should look very similar to those for Localization_EKF.
    # In particular, essentially the only thing that needs to change is the computation
    # of Hx in map_line_to_predicted_measurement and how that method is called in
    # associate_measurements (i.e., instead of getting world-frame line parameters from
    # self.map_lines, you must extract them from the state self.x)
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H (should be identical to Localization_EKF.measurement_model above)
        ##############

        return z, R, H

    # Adapt this method from Localization_EKF.map_line_to_predicted_measurement.
    #
    # Note that instead of the actual parameters m = (alpha, r) we pass in the map line index j
    # so that we know which components of the Jacobian to fill in.
    def map_line_to_predicted_measurement(self, j):
        alpha, r = self.x[(3+2*j):(3+2*j+2)]    # j is zero-indexed! (yeah yeah I know this doesn't match the pset writeup)

        #### TODO ####
        # compute h, Hx (you may find the skeleton for computing Hx below useful)

        Hx = np.zeros((2,self.x.size))
        Hx[:,:3] = FILLMEIN
        # First two map lines are assumed fixed so we don't want to propagate any measurement correction to them
        if j > 1:
            Hx[0, 3+2*j] = FILLMEIN
            Hx[1, 3+2*j] = FILLMEIN
            Hx[0, 3+2*j+1] = FILLMEIN
            Hx[1, 3+2*j+1] = FILLMEIN

        ##############

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Adapt this method from Localization_EKF.associate_measurements.
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############

        return v_list, R_list, H_list
