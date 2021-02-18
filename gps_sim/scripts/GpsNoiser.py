import colorednoise as cn
import numpy as np

import matplotlib
import matplotlib.pyplot as plt


class GpsNoiser:

    # multiplier for each of the noise types, each of the axes
    def __init__(self, white=np.array([0.1, 0.1, 0.1]),
                        pink=np.array([0.05, 0.05, 0.6]),
                        brown=np.array([0.5, 0.5, 0.1])):
        self._white = white
        self._pink = pink
        self._brown = brown
        self._sample_buffer_length = 7200  # [s]
        self._sampling_freq = 10  # [hz]
        # order: white, pink, brown
        self._sample_buffer = np.zeros([self._sample_buffer_length * self._sampling_freq, 9])
        self._sample_buffer_pos = 0

    def sample_noise(self):
        # make some noise!
        n = self._sample_buffer_length * self._sampling_freq
        for i in range(0, 3):
            self._sample_buffer[:, i] = cn.powerlaw_psd_gaussian(0, n)  # white
            self._sample_buffer[:, i + 3] = cn.powerlaw_psd_gaussian(1, n)  # pink
            self._sample_buffer[:, i + 6] = cn.powerlaw_psd_gaussian(2, n)  # brown
        self._sample_buffer_pos = 0

    def perturb(self, pos_enu):
        if self._sample_buffer_pos >= self._sample_buffer_length * self._sampling_freq:
            print("Run out of sampling buffer")
            # redo -> might cause discontinuity in noise, should be avoided
            self.sample_noise()

        i = self._sample_buffer_pos
        self._sample_buffer_pos += 1

        return pos_enu + np.multiply(self._white, self._sample_buffer[i, 0:3]) + \
               np.multiply(self._pink, self._sample_buffer[i, 3:6]) + \
               np.multiply(self._brown, self._sample_buffer[i, 6:9])


if __name__ == "__main__":
    a = GpsNoiser()
    true_pos = np.array([0, 0, 0])
    a.sample_noise()
    disturbed_pos = np.zeros([1000,3])

    for i in range(0,1000):
        disturbed_pos[i,:] = a.perturb(true_pos)


    fig, ax = plt.subplots()
    ax.scatter(disturbed_pos[:, 0], disturbed_pos[:, 1],c = np.arange(1000), marker='x')
    #ax.plot(disturbed_pos[0:60,0],'r')
    #ax.plot(disturbed_pos[0:60, 1], 'g')
    #ax.plot(disturbed_pos[0:60, 2], 'b')
    ax.set_xlabel("ENU_X noise")
    ax.set_ylabel("ENU_Y noise")
    ax.set_title("combined example")
    ax.grid()
    plt.show()
