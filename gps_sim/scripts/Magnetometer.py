import geomag as gm
import numpy as np

import matplotlib
import matplotlib.pyplot as plt


class Magnetometer:
    def __init__(self, noise_cov=500):
        # set initial field strength
        magnetic_field = gm.geomag.GeoMag()
        magnetic_data = magnetic_field.GeoMag(47.37, 8.53)
        self._field_vector = np.array([magnetic_data.bx, magnetic_data.by, magnetic_data.bz])
        # d =
        self._noise_cov = noise_cov
        self._declination = magnetic_data.dec

    # takes current orientation and rotates magnetic field according to this
    # currently does not include non-linearities in calibration
    def get_field(self, orientation=np.eye(3)):
        noised_vector = self._field_vector + np.random.normal([0, 0, 0],
                                                              [self._noise_cov, self._noise_cov, self._noise_cov])
        return orientation.dot(noised_vector)

    def get_declination(self):
        return self._declination
