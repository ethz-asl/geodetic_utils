import colorednoise as cn
import numpy as np

class GpsNoiser:
    # multiplier for each of the noise types, each of the axes
    def __init__(self, white=np.array([0.1, 0.1, 0.1]),
                 pink=np.array([0.05, 0.05, 0.6]),
                 brown=np.array([0.5, 0.5, 0.1]), epsilons=np.array([0, 1, 2.5])):
        self._white = white
        self._pink = pink
        self._brown = brown
        self._sample_buffer_length = 7200  # [s]
        self._sampling_freq = 10  # [hz]
        # order: white, pink, brown
        self._sample_buffer = np.zeros([self._sample_buffer_length * self._sampling_freq, 9])
        self._sample_buffer_pos = 0
        self._epsilons = epsilons

        self.sample_noise()

    def sample_noise(self):
        # make some noise!
        n = self._sample_buffer_length * self._sampling_freq
        for i in range(0, 3):
            self._sample_buffer[:, i] = cn.powerlaw_psd_gaussian(self._epsilons[0], n)  # white
            self._sample_buffer[:, i + 3] = cn.powerlaw_psd_gaussian(self._epsilons[1], n)  # pink

            # for larger epsilons a low cut off frequency (fmin) has improved ease of tuning:
            self._sample_buffer[:, i + 6] = cn.powerlaw_psd_gaussian(self._epsilons[2], n, fmin=0.001)  # brown

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
