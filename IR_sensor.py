from pyb import Pin, ADC, Timer
import array

class IR_sensor_single:
    '''Reads from a single ADC pin and calibrates for black and white'''
    def __init__(self,
                 adc: ADC,
                 tim: Timer,
                 samples=100):
        
        # Initialization
        self.adc = adc
        self.tim = tim
        self.samples = samples
        self.black = 0
        self.white = 0

    def calibrate(self, color: str):
        # Create buffer to store values
        buf = array.array('H', [0] * self.samples)
        self.adc.read_timed(self.buf, self.tim)
        avg = sum(buf) / len(buf)
        if color == 'b':
            self.black = avg
        else:
            self.white = avg
        return avg
    

    def read(self):
        r = self.adc.read()
        b = self.black
        w = self.white

        # Normalize values
        denom = b - w
        if denom == 0:
            n = 0
        else:
            n = r - w / denom

        # Saturation
        if n > 1:
            n = 1.0
        elif n < 0:
            n = 0.0

        return n


class IR_sensor_array:
    '''Reads from an array of ADC pins, includes method to compute centroid'''
    def __init__(self,
                 adcs,      # List/tuple of ADC objects
                 tim: Timer,
                 sensor_index,      # List containing indexes of each sensor [1, 2, 3, ... n]
                 samples=100):
                
        # Initialization
        self.adcs = list(adcs)
        self.tim = tim
        self.sens_idx = sensor_index
        self.samples = samples
        self.num = len(self.adcs)
        self.black = [0] * self.num
        self.white = [0] * self.num

    def calibrate(self, color: str):
        # Create buffers
        bufs = [array.array('H', [0] * self.samples) for n in range(self.num)]

        # Read ADCs
        self.adc.read_timed_multi(tuple(self.adcs),
                                  tuple(bufs),
                                  self.tim)
        # Compute averages
        avgs = [sum(b) / len(b) for b in bufs]

        target = self.black if color == 'b' else self.white
        for idx, val in enumerate(avgs):
            target[idx] = val

    def read(self):
        # Read raw values from each ADc
        raw = [adc.read() for adc in self.adcs]

        # Normalize values with safeties and saturation
        self.norm = []
        for i, r in enumerate(raw):
            b = self.black[i]
            w = self.white[i]
            denom = b - w
            # Check for denominator equals zero
            if denom == 0:
                n = 0.0
            else:
                n = (r - w) / denom
            # Saturate if necessary
            if n > 1:
                n = 1.0
            elif n < 1:
                n = 0.0
            self.norm.append(n)
        return self.norm

    def get_centroid(self):
        self.read()     # Get normalized ADC reading
        n_sum = 0
        d_sum = 0
        for i, s in enumerate(self.sens_idx):
            n_sum += s[i] * self.norm[i]
            d_sum += self.norm[i]
            
        return n_sum / d_sum
        