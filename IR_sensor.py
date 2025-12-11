from os import listdir
from pyb import Pin, ADC, Timer
import array

class IRArray:
    """
    IR sensor array driver with per-channel calibration and centroid calculation.
    - Pins are specified by the caller (in main.py); driver creates ADCs.
    - Supports flexible board index mapping (physical sensor indices printed on the board).
    - Uses read_timed_multi() for calibration; read() uses single-shot for low overhead.
    """
# ----------------------------------------------------------------------
    def __init__(self,
                 tim_num: int,
                 samples: int,
                 IR_pins,
                 sensor_indices=None):
        """
        Args:
            tim_num: Timer number for creating a timer for ADC sampling
            samples: Number of samples to average in calibration
            IR_pins: List of Pin.cpu.<X> constants (order: left -> right)
            sensor_indices: Optional list of physical board indices corresponding to the IR_pins list
                            (example: [1,3,5,7,9,11] if only odd sensors are populated). If None, indices default to [1..N] in the given order.
        """
        self.tim_obj = Timer(tim_num, freq = 20000) # 20 kHz timer for sampling from IR sensors
        self.samples = int(samples)
        # Create ADCs internally in the driver from IR_pins specified in main.py
        self.adcs = [ADC(Pin(p)) for p in IR_pins]
        self.num = len(self.adcs)
        self.white_cal = False
        self.black_cal = False

        # Indices used for centroid math (center = (min(idx)+max(idx))/2)
        if sensor_indices is None:
            self.sensor_index = list(range(1, self.num + 1))
        else:
            if len(sensor_indices) != self.num:
                raise ValueError("sensor_indices length must match IR_pins length")
            self.sensor_index = list(sensor_indices)

        # Calibration data
        self.black = [0.0] * self.num
        self.white = [0.0] * self.num

        # Last normalized read (0..1 where 1 ~ black line)
        self.norm = [0.0] * self.num

    # ----------------------------------------------------------------------
    def calibrate(self, color: str):
        """
        Calibrate on 'w' (white background) or 'b' (black line).
        Uses read_timed_multi() to fill per-channel buffers and averages them.
        Prints a compact table of per-sensor averages for visual verification.
        Returns the list of averages (float) in the same order as sensor_index.
        """
        bufs = [array.array('H', [0] * self.samples) for _ in range(self.num)]
        ADC.read_timed_multi(tuple(self.adcs), tuple(bufs), self.tim_obj)
        avgs = [sum(b) / len(b) for b in bufs]

        if color == 'b':
            for i, v in enumerate(avgs):
                self.black[i] = float(v)
            label = "BLACK"
            self.black_cal = True
        else:
            for i, v in enumerate(avgs):
                self.white[i] = float(v)
            label = "WHITE"
            self.white_cal = True
        
        print("\r\n[IR CALIBRATION] {} averages:".format(label))
        print(" Index | Average (counts)")
        print("-------+------------------")
        for idx, val in zip(self.sensor_index, avgs):
            print(f" {idx:5d} | {val:7.1f}")
        print(f"  min  | {min(avgs):7.1f}")
        print(f"  max  | {max(avgs):7.1f}")
        print("")

        # Save to file if both calibrations done
        if self.black_cal and self.white_cal:
            with open("IR_cal.txt", "w") as f:
                black_line = ",".join(f"{v:.1f}" for v in self.black)
                white_line = ",".join(f"{v:.1f}" for v in self.white)
                f.write(black_line + "\n")
                f.write(white_line + "\n")
            print("[IR CALIBRATION] Calibration data saved to IR_cal.txt")

        return avgs
    
    # --------------------------------------------------------
    def set_calibration(self):
        with open("IR_cal.txt", "r") as f:
            lines = f.readlines()
            if len(lines) >= 2:
                black_vals = [float(v) for v in lines[0].strip().split(",")]
                white_vals = [float(v) for v in lines[1].strip().split(",")]
                if len(black_vals) == self.num and len(white_vals) == self.num:
                    self.black = black_vals
                    self.white = white_vals
                    print("Black and White calibration data loaded.")
                else:
                    print("[IR CALIBRATION] Calibration file sensor count mismatch; using defaults")
            else:
                print("[IR CALIBRATION] Calibration file format error; using defaults")
    
    # ----------------------------------------------------------------------
    def read(self):
        """
        Single-shot read on all channels, normalized to [0,1].
        0 ~ white (background), 1 ~ black (line).
        Returns list of floats, same order as IR_pins / sensor_index.
        """
        out = []
        for i, adc in enumerate(self.adcs):
            r = adc.read()
            b = self.black[i]
            w = self.white[i]
            denom = (b - w)
            n = (r - w) / denom if denom != 0 else 0.0
            if n < 0.0: n = 0.0
            if n > 1.0: n = 1.0
            out.append(n)
        self.norm = out
        return self.norm

    # ----------------------------------------------------------------------
    def get_centroid(self):
        """
        Compute centroid using the last normalized vector (or perform a read).
        Returns:
            (centroid, seen)
            centroid: float in the index space (not 0..1; uses sensor_index values)
            seen: bool indicating if any signal was seen (sum(norm) > small eps)
        """
        self.read()
        total = sum(self.norm)
        if total <= 1e-6:
            return 0.0, False

        weighted = 0.0
        for idx, val in zip(self.sensor_index, self.norm):
            weighted += idx * val

        return (weighted / total), True

    # ----------------------------------------------------------------------
    def center_index(self):
        """
        Returns the ideal center location in index space.
        For arbitrary index sets, use average of min and max (center of span).
        """
        return 0.5 * (min(self.sensor_index) + max(self.sensor_index))