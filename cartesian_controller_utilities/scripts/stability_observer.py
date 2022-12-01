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

        # Online recursive stability index according to [1].
        self.I_s = 0

        # We measure our computation rate for performance
        self.calls = 0
        self.rate = 500 # nominal sensor rate
        self.thread = threading.Thread(target=self.measure, daemon=True)
        self.thread.start()

    def measure(self):
        """ Compute our computation rate in Hz

        We need a non-zero lower rate for algorithmic stability reasons.
        """
        while True:
            t = 0.1 # sec
            min_rate = 10e-5 # Hz
            time.sleep(t)
            new_rate = self.calls / t
            self.rate = max(min_rate, (self.rate + new_rate) / 2.0) # mean filter
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
        # Also drop the 0th element that holds the sum over all signals.
        amplitude_spec = np.abs(np.fft.fftn(self.data))
        amplitude_spec = amplitude_spec[1:int(self.N / 2)]

        # Compute the index where the frequencies are greater than the cross over frequency.
        # The frequency spectrum is defined by the sensor rate and the sample count.
        # Like with the amplitudes, we select only the relevant frequencies.
        timestep = 1.0 / self.rate # sensor
        freq = np.fft.fftfreq(self.N, d=timestep)
        freq = freq[1:int(self.N / 2)]
        if max(freq) > self.wc:
            index = next(x[0] for x in enumerate(freq) if x[1] > self.wc)
        else:
            # Frequency spectrum still insufficient.
            # We seem to be resuming after a pause and need to build up the sensor rate.
            index = -1

        # Sum of magnitudes of the high frequency components
        axis = 2  # of sensor measurements
        high_mag = np.sum(amplitude_spec[index:, axis])

        # Sum of magnitudes of all frequency components
        all_mag = np.sum(amplitude_spec[:, axis])

        # Stability indices and parameters from [1]. See p. 7.
        I_w = high_mag / all_mag if all_mag else 0.0
        f_max = 100 # Newton
        h = 0.99 # lambda
        f_window = 25
        p = min(f_window, len(self.data))
        f_h = np.array(self.data)[-p:, axis]  # The last p values, one Cartesian axis for testing
        I_frms = np.sqrt(1/p * np.sum(np.square(f_h))) / f_max

        # Final stability index
        self.I_s = I_w * I_frms + h * self.I_s


        # Debug output
        print(f" {self.rate:.1f} Hz  stability index: {self.I_s:.3f}", end='\r', flush=True)
        #print(f" {self.rate:.1f} Hz  I_w: {I_w:.3f}", end='\r', flush=True)



if __name__ == '__main__':
    observer = StabilityObserver()
    rospy.spin()
