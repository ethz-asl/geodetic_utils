import numpy as np
from GpsNoiser import GpsNoiser
from Magnetometer import Magnetometer


class GpsSimulator:

    def __init__(self):
        self._rtk_noiser = GpsNoiser(white=np.array([0.0005, 0.0005, 0.002]),
                                     pink=np.array([0.0035, 0.0035, 0.007]),
                                     brown=np.array([0.001, 0.001, 0.002]),
                                     epsilons=np.array([0, 1, 2]))
        self._rtk_noiser.sample_noise()

        self._float_noiser = GpsNoiser(white=np.array([0.0075, 0.0075, 0.015]),
                                       pink=np.array([0.001, 0.001, 0.002]),
                                       brown=np.array([0.03, 0.03, 0.06]),
                                       epsilons=np.array([0, 1, 3.0]))
        self._float_noiser.sample_noise()

        self._spp_noiser = GpsNoiser(white=np.array([0.001, 0.001, 0.002]),
                                     pink=np.array([0.0, 0.0, 0.0]),
                                     brown=np.array([0.50, 0.5, 1.0]),
                                     epsilons=np.array([0, 1, 3.0]))
        self._spp_noiser.sample_noise()

        self._spp_output_cov = np.array([[1.15363866, 0.00769917, -0.02256928],
                                         [0.00769917, 1.16177632, -0.03191735],
                                         [-0.02256928, -0.03191735, 5.23445582]])

        # empirical covariances from imagined model not based on data
        # -> to be based on data at a later point.
        self._float_output_cov = np.array([[8.17408501e-04, 7.13988338e-05, 1.47721410e-04],
                                           [7.13988338e-05, 1.01484400e-03, -1.63778765e-04],
                                           [1.47721410e-04, -1.63778765e-04, 2.07055086e-03]])

        self._rtk_output_cov = np.array([[9.47897207e-05, 1.92104152e-05, -6.46848160e-05],
                                         [1.92104152e-05, 9.39087989e-05, -7.25870764e-05],
                                         [-6.46848160e-05, -7.25870764e-05, 4.24079132e-04]])

        self._mag_noiser = Magnetometer()

        # can be: auto, none, spp, float, rtk
        self._current_mode = "spp"

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
