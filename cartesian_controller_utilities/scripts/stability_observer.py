#!/usr/bin/env python3
import sys
import numpy as np
import threading
import rospy
import time
from geometry_msgs.msg import WrenchStamped




class StabilityObserver(object):
    """ Automatically decrease the controllers' oscillations in contact

    TODO: Detailed description and link to the paper.
    [1]: "Online stability in human-robot cooperation with admittance control", Dimeas et al., 2016
    [2]: "Frequency domain stability observer and active damping control for stable haptic interaction, Ryu et al., 2008
    """
    def __init__(self):
        rospy.init_node('stability_observer')

        # force data samples.
        # Influences the frequency resolution (df) with the sensor's sampling time (T):
        # df = 1 / (T * N)
        self.N = 128

        # Crossover frequency omega (below = stable, above = unstable).
        # Fastest rate that humans could intentionally generate in experiments
        # with haptic interaction. Everything above is considered unstable,
        # unwanted control behavior.
        self.wc = 20 # Hz

        dim = 6 # Cartesian force dimension
        self.data = np.zeros((self.N, dim)).tolist()
        self.sensor_sub = rospy.Subscriber('/wrench', WrenchStamped, self.sensor_cb)

        # We measure our computation rate for performance
        self.calls = 0
        self.rate = 500 # nominal sensor rate
        self.thread = threading.Thread(target=self.measure, daemon=True)
        self.thread.start()

    def measure(self):
        """ Compute our computation rate in Hz """
        while True:
            t = 0.1 # sec
            time.sleep(t)
            new_rate = self.calls / t
            self.rate = (self.rate + new_rate) / 2.0 # mean filter
            self.calls = 0

    def sensor_cb(self, data):
        """ Apply instability indices in the frequency domain

        This applies Fast Fourier Transformation (FFT) and computes the terms from the paper.
        """
        self.calls += 1 # performance measure

        point = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z,
                 data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z
                 ]
        self.data.append(point)
        self.data.pop(0)  # drop oldest value

        # Amplitude spectrum.
        # We are interested only in the first half (= positive frequencies).
        amplitude_spec = np.abs(np.fft.fftn(self.data))
        amplitude_spec = amplitude_spec[0:int(self.N / 2)]

        # Compute the index where the frequencies are greater than the cross over frequency.
        # This is an analytical computation and does not depend on the actual data.
        timestep = 1.0 / self.rate # sensor
        freq = np.fft.fftfreq(self.N, d=timestep)
        index = next(x[0] for x in enumerate(freq) if x[1] > self.wc)

        # Sum of magnitudes of the high frequency components
        axis = 2  # of sensor measurements
        high_mag = np.sum(amplitude_spec[index:, axis])

        # Sum of magnitudes of all frequency components
        all_mag = np.sum(amplitude_spec[:, axis])
        sys.stdout.write(f"\t{self.rate:.1f} Hz\t\rstability index: {high_mag / all_mag:.3f}")
        sys.stdout.flush()



if __name__ == '__main__':
    observer = StabilityObserver()
    rospy.spin()
