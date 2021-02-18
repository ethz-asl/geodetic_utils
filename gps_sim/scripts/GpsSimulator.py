import numpy as np
from GpsNoiser import GpsNoiser
from Magnetometer import Magnetometer


class GpsSimulator:

    def __init__(self):
        self._rtk_noiser = GpsNoiser(white=np.array([0.0005, 0.0005, 0.002]),
                                     pink=np.array([0.0035, 0.0035, 0.007]),
                                     brown=np.array([0.001, 0.001, 0.002]),
                                     epsilons=np.array([0, 1, 2]))

        self._float_noiser = GpsNoiser(white=np.array([0.0075, 0.0075, 0.015]),
                                       pink=np.array([0.001, 0.001, 0.002]),
                                       brown=np.array([0.03, 0.03, 0.06]),
                                       epsilons=np.array([0, 1, 3.0]))

        self._spp_noiser = GpsNoiser(white=np.array([0.001, 0.001, 0.002]),
                                     pink=np.array([0.0, 0.0, 0.0]),
                                     brown=np.array([0.50, 0.5, 1.0]),
                                     epsilons=np.array([0, 1, 3.0]))

        self._spp_output_cov = np.eye(3)
        self._float_output_cov = np.eye(3)
        self._rtk_output_cov = np.eye(3)

        self._mag_noiser = Magnetometer()

        # can be: auto, none, spp, float, rtk
        self._current_mode = "auto"

        self._rtk_polygon = []
        self._spp_polygon = []
        self._float_polygon = []
        self._none_polygon = []

    def set_mode(self, mode):
        self._current_mode = mode

    def get_mag(self, input_rot):
        return self._mag_noiser.get_field(input_rot)

    def get_gps(self, input_enu, fixtype):
        if fixtype is "none":
            # *screaming" lost fix!!
            return None, None

        output_enu = np.zeros([0, 0, 0])
        output_cov = np.eye(3)

        if fixtype is "rtk":
            output_enu = self._rtk_noiser.perturb(input_enu)
            output_cov = self._rtk_output_cov

        elif fixtype is "float":
            output_enu = self._float_noiser.perturb(input_enu)
            output_cov = self._float_output_cov

        elif fixtype is "spp":
            output_enu = self._spp_noiser.perturb(input_enu)
            output_cov = self._spp_output_cov

        return output_enu, output_cov

    def get_fixtype(self, input_enu):
        return "rtk"  # for now

    def simulate(self, input_pose):
        input_enu = input_pose[0:3, 3]

        # output GPS
        if self._current_mode is "auto":
            fixtype = self.get_fixtype(input_enu)
            output_enu, output_cov = self.get_gps(input_enu, fixtype)
        else:
            output_enu, output_cov = self.get_gps(input_enu, self._current_mode)

        output_mag = self.get_mag(input_pose[0:3, 0:3])

        # weird pythonic flex that these variables are found
        return output_enu, output_cov, output_mag
