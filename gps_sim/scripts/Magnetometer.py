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
    def getMagneticField(self, orientation=np.eye(3)):
        noised_vector = self._field_vector + np.random.normal([0, 0, 0],
                                                              [self._noise_cov, self._noise_cov, self._noise_cov])
        return orientation.dot(noised_vector)

    def getDeclination(self):
        return self._declination


if __name__ == "__main__":
    a = Magnetometer()
    magnetic_field = np.zeros([1000, 3])
    resulting_heading = np.zeros([1000, 1])
    for i in range(0, 1000):
        magnetic_field[i, :] = a.getMagneticField()
        resulting_heading[i] = np.arctan2(magnetic_field[i, 1], magnetic_field[i, 0]) * (180 / 3.1415)

    fig, ax = plt.subplots()
    # ax.plot(magnetic_field[0:1000,0],'r')
    # ax.plot(magnetic_field[0:1000, 1], 'g')
    # ax.plot(magnetic_field[0:1000, 2], 'b')
    ax.plot(resulting_heading, 'r')

    ax.grid()
    plt.show()
