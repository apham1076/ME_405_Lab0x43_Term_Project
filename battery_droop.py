# battery_droop.py
# ==============================================================================
# Measures the battery voltage and compensates for droop by correcting the
# nominal effort based on the measured battery voltage.
# ==============================================================================

from pyb import Pin, ADC

# Constants for voltage divider
# -----------------------------------------------------------------------------
R_1_OHM = 9.98e3   # 10k ohm (measured 9.98k ohm)
R_2_OHM = 4.712e3  # 4.7k ohm (measured 4.712k ohm)
V_REF = 3.3        # ADC reference voltage
V_NOM_SINGLE = 1.6         # Nominal battery voltage for a single cell
V_NOM_TOTAL = 6*V_NOM_SINGLE  # Total nominal battery voltage
# -----------------------------------------------------------------------------

class Battery:
    """Reads raw battery voltage through a voltage divider and computes a
       corrected motor effort based on the measured voltage to compensate for
       battery droop."""
    
    def __init__(self, adc_pin, R_1_OHM, R_2_OHM, V_REF, V_NOM_TOTAL):
        """Initializes the Battery object with ADC pin and voltage divider values."""
        self.adc = ADC(Pin(adc_pin))
        self.R_1_OHM = R_1_OHM
        self.R_2_OHM = R_2_OHM
        self.V_REF = V_REF
        self.V_NOM_TOTAL = V_NOM_TOTAL
        self.scale = (R_1_OHM + R_2_OHM) / R_2_OHM

    def read_voltage(self):
        """Reads the raw ADC value and computes the actual battery voltage."""
        raw_adc = self.adc.read()  # Read raw ADC value (0-4095)
        V_adc = (raw_adc / 4095.0) * self.V_REF  # Measured voltage at ADC pin
        V_batt = V_adc * self.scale  # Compute actual battery voltage
        return V_batt
    
    def droop_gain(self):
        """Computes the droop compensation gain (V_NOM / V_batt) based on measured voltage."""
        V_batt = self.read_voltage()

        # Avoid division by zero or very low voltage (ADC may be disconnected)
        if V_batt <= 0.5:
            print("Warning: Measured battery voltage too low or invalid.")
            return 1.0  # Avoid division by zero, no compensation
        
        # Actually compute the droop gain
        droop_gain = self.V_NOM_TOTAL / V_batt
        return droop_gain