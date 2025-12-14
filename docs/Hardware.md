@page hardware Hardware & Electrical Design

## System Overview
The Romi robot is built on a Pololu Romi chassis and controlled using an ST Nucleo development board running MicroPython. The hardware and electrical design emphasizes predictable startup behavior, clean signal routing, and modular interfaces that support incremental development, debugging, and future extension.

Major subsystems integrated into the robot include:
- Dual DC motors with quadrature encoders
- Reflectance-based line sensor array
- Inertial Measurement Unit (IMU)
- Bluetooth communication module
- Battery-powered system with onboard voltage monitoring

---

## Microcontroller and Power System
The system is powered by **six AA batteries**, routed through a power distribution board (PDB). Battery voltage is supplied to the Nucleo board via the VIN pin. A resistive voltage divider feeds an ADC input, allowing real-time measurement of battery voltage during operation.

This measurement is used to compensate for battery voltage droop in the motor control pipeline, improving repeatability over the duration of a run.

Startup behavior was treated as a primary design constraint. Motor driver enable (nSLP) pins are explicitly controlled in software, and pin selections were chosen to prevent unintended motor motion during power-up or reset. Pull-down configurations and initialization order were carefully managed to ensure safe and predictable behavior.

---

## Motor Driver Interfaces
Each motor is driven using three signals:
- PWM signal for effort command
- Direction signal
- Sleep/enable (nSLP) signal

Hardware timers were selected to generate PWM outputs independently for each motor without conflicting with encoder timers or communication peripherals. Motor enable pins are controlled as standard digital outputs to enforce explicit enable/disable behavior in firmware.

This structure ensures motors remain disabled until the system has fully initialized and the scheduler has started.

---

## Quadrature Encoder Interfaces
Wheel encoders are interfaced using hardware timer encoder modes. This approach offloads quadrature decoding to dedicated timer hardware, providing robust position and velocity measurements with minimal CPU overhead.

Separate timers are used for the left and right encoders to allow fully independent velocity estimation and closed-loop control. Encoder wiring and pin selection were chosen to avoid overlap with PWM, ADC, and communication peripherals.

---

## Line Sensor Array
Line detection is performed using a multi-channel reflectance sensor array mounted at the front of the chassis. Individual sensors are read using ADC inputs distributed across multiple GPIO ports to balance pin availability and minimize conflicts with timers and communication interfaces.

Although only a subset of sensors is used for control during line following, all channels are wired and supported in firmware. This provided flexibility during development and allowed sensor selection and weighting strategies to evolve without hardware changes.

---

## IMU Interface
An IMU is connected via the I2C2 peripheral. Dedicated SDA and SCL pins were selected to avoid interference with timers, ADC inputs, and UART communication.

The IMU is supported in firmware and was used extensively during Lab 0x05 for state estimation experiments. Although it was not relied upon for the final line-following implementation, its integration provided valuable insight into system dynamics and informed later design decisions.

---

## Bluetooth Communication Interface
A Bluetooth module provides wireless communication between the Romi and a PC. The module is connected via a UART interface and is used for real-time data streaming, diagnostics, and parameter adjustment during testing.

Streaming is enabled by default on startup and can be toggled through the PC-side interface. This approach reduced onboard memory usage and provided immediate visibility into system behavior during controller tuning and debugging.

---

## Pin Assignment Summary
The following table summarizes the primary electrical connections used in the final system configuration. Wire colors are shown explicitly to aid debugging and reproducibility.

<table>
  <tr>
    <th>Cable</th>
    <th>Contact</th>
    <th>Color</th>
    <th>Signal / Module Pin</th>
    <th>Nucleo Pin</th>
    <th>PDB Pin</th>
    <th>Port</th>
    <th>AF</th>
    <th>Function</th>
  </tr>

  <!-- ================= ENCODERS ================= -->
  <tr>
    <td rowspan="2">Encoder (Left)</td>
    <td>1</td>
    <td style="background:#1f77b4;color:white;">Blue</td>
    <td>Encoder Ch. A</td>
    <td>Timer Pin, Ch.1</td>
    <td>ELA or ERA</td>
    <td>PA9</td>
    <td>AF1</td>
    <td>TIM1_CH2</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:#f1c40f;">Yellow</td>
    <td>Encoder Ch. B</td>
    <td>Timer Pin, Ch.2</td>
    <td>ELB or ERB</td>
    <td>PA8</td>
    <td>AF2</td>
    <td>TIM1_CH1</td>
  </tr>

  <tr>
    <td rowspan="2">Encoder (Right)</td>
    <td>1</td>
    <td style="background:#1f77b4;color:white;">Blue</td>
    <td>Encoder Ch. A</td>
    <td>Timer Pin, Ch.1</td>
    <td>ELA or ERA</td>
    <td>PC7</td>
    <td>AF2</td>
    <td>TIM8_CH2</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:#f1c40f;">Yellow</td>
    <td>Encoder Ch. B</td>
    <td>Timer Pin, Ch.2</td>
    <td>ELB or ERB</td>
    <td>PC6</td>
    <td>AF2</td>
    <td>TIM8_CH1</td>
  </tr>

  <!-- ================= MOTORS ================= -->
  <tr>
    <td rowspan="3">Motor (Left)</td>
    <td>1</td>
    <td style="background:#f1c40f;">Yellow</td>
    <td>Motor Enable</td>
    <td>Digital Output</td>
    <td>nSLP</td>
    <td>PB4</td>
    <td>-</td>
    <td>Enable / Sleep</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:#1f77b4;color:white;">Blue</td>
    <td>Motor Direction</td>
    <td>Digital Output</td>
    <td>DIR</td>
    <td>PB3</td>
    <td>-</td>
    <td>Direction</td>
  </tr>
  <tr>
    <td>3</td>
    <td style="background:#2ecc71;">Green</td>
    <td>Motor Effort</td>
    <td>Timer Pin</td>
    <td>PWM</td>
    <td>PB5</td>
    <td>AF2</td>
    <td>TIM3_CH2</td>
  </tr>

  <tr>
    <td rowspan="3">Motor (Right)</td>
    <td>1</td>
    <td style="background:#f1c40f;">Yellow</td>
    <td>Motor Enable</td>
    <td>Digital Output</td>
    <td>nSLP</td>
    <td>PC9</td>
    <td>-</td>
    <td>Enable / Sleep</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:#1f77b4;color:white;">Blue</td>
    <td>Motor Direction</td>
    <td>Digital Output</td>
    <td>DIR</td>
    <td>PB9</td>
    <td>-</td>
    <td>Direction</td>
  </tr>
  <tr>
    <td>3</td>
    <td style="background:#2ecc71;">Green</td>
    <td>Motor Effort</td>
    <td>Timer Pin</td>
    <td>PWM</td>
    <td>PB8</td>
    <td>AF2</td>
    <td>TIM4_CH3</td>
  </tr>

  <!-- ================= IMU ================= -->
  <tr>
    <td rowspan="6">IMU</td>
    <td>-</td>
    <td style="background:#e74c3c;color:white;">Red</td>
    <td>IMU VIN</td>
    <td>-</td>
    <td>5V (CN7)</td>
    <td>-</td>
    <td>-</td>
    <td>Power</td>
  </tr>
  <tr>
    <td>-</td>
    <td>—</td>
    <td>IMU 3vo</td>
    <td>-</td>
    <td>Optional*</td>
    <td>-</td>
    <td>-</td>
    <td>Regulated Output</td>
  </tr>
  <tr>
    <td>-</td>
    <td style="background:black;color:white;">Black</td>
    <td>IMU GND</td>
    <td>-</td>
    <td>GND (CN7)</td>
    <td>-</td>
    <td>-</td>
    <td>Ground</td>
  </tr>
  <tr>
    <td>-</td>
    <td style="background:#800020;color:white;">Burgundy</td>
    <td>IMU SDA</td>
    <td>-</td>
    <td>-</td>
    <td>PB14</td>
    <td>AF4</td>
    <td>I2C2_SDA</td>
  </tr>
  <tr>
    <td>-</td>
    <td style="background:gray;color:white;">Gray</td>
    <td>IMU SCL</td>
    <td>-</td>
    <td>-</td>
    <td>PB13</td>
    <td>AF4</td>
    <td>I2C2_SCL</td>
  </tr>
  <tr>
    <td>-</td>
    <td style="background:white;">White</td>
    <td>IMU RST</td>
    <td>-</td>
    <td>-</td>
    <td>-</td>
    <td>-</td>
    <td>Reset</td>
  </tr>

  <!-- ================= BLUETOOTH ================= -->
  <tr>
    <td rowspan="4">Bluetooth Module</td>
    <td>1</td>
    <td style="background:white;">White</td>
    <td>Module RX</td>
    <td>Romi TX</td>
    <td>-</td>
    <td>PB6</td>
    <td>AF7</td>
    <td>UART1_TX</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:orange;">Orange</td>
    <td>Module TX</td>
    <td>Romi RX</td>
    <td>-</td>
    <td>PB7</td>
    <td>AF7</td>
    <td>UART1_RX</td>
  </tr>
  <tr>
    <td>3</td>
    <td style="background:black;color:white;">Black</td>
    <td>Module GND</td>
    <td>Romi GND</td>
    <td>-</td>
    <td>GND (CN7)</td>
    <td>-</td>
    <td>Ground</td>
  </tr>
  <tr>
    <td>4</td>
    <td style="background:#e74c3c;color:white;">Red</td>
    <td>Module VCC</td>
    <td>Romi 5V</td>
    <td>-</td>
    <td>E5V (CN7)</td>
    <td>-</td>
    <td>Power</td>
  </tr>

  <!-- ================= POWER ================= -->
  <tr>
    <td rowspan="5">Battery / Power</td>
    <td>1</td>
    <td style="background:black;color:white;">Black</td>
    <td>Divider Ground</td>
    <td>GND</td>
    <td>-</td>
    <td>GND (CN7)</td>
    <td>-</td>
    <td>Ground</td>
  </tr>
  <tr>
    <td>2</td>
    <td style="background:#1f77b4;color:white;">Blue</td>
    <td>Divider Node</td>
    <td>ADC Input</td>
    <td>-</td>
    <td>PC2</td>
    <td>-</td>
    <td>Battery Sense</td>
  </tr>
  <tr>
    <td>3</td>
    <td style="background:#e74c3c;color:white;">Red</td>
    <td>Power to Nucleo</td>
    <td>VIN</td>
    <td>-</td>
    <td>VIN (CN6)</td>
    <td>-</td>
    <td>Board Power</td>
  </tr>
  <tr>
    <td>4</td>
    <td style="background:black;color:white;">Black</td>
    <td>PDB Ground</td>
    <td>-</td>
    <td>GND</td>
    <td>-</td>
    <td>-</td>
    <td>Ground</td>
  </tr>
  <tr>
    <td>5</td>
    <td style="background:#e74c3c;color:white;">Red</td>
    <td>PDB Battery Power</td>
    <td>-</td>
    <td>VSW</td>
    <td>-</td>
    <td>-</td>
    <td>Battery</td>
  </tr>

  <!-- ================= LINE SENSORS ================= -->
  <tr>
    <td rowspan="12">Line Sensors</td>
    <td>-</td>
    <td style="background:#e74c3c;color:white;">Red</td>
    <td>Power</td>
    <td>3.3V</td>
    <td>-</td>
    <td>3.3V (CN7)</td>
    <td>-</td>
    <td>Power</td>
  </tr>
  <tr><td>-</td><td style="background:black;color:white;">Black</td><td>Ground</td><td>GND</td><td>-</td><td>GND (CN7)</td><td>-</td><td>Ground</td></tr>
  <tr><td>-</td><td style="background:#800020;color:white;">Burgundy</td><td>Sensor 1</td><td>ADC</td><td>-</td><td>PC4</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:#800020;color:white;">Burgundy</td><td>Sensor 3</td><td>ADC</td><td>-</td><td>PA1</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:orange;">Orange</td><td>Sensor 5</td><td>ADC</td><td>-</td><td>PA4</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:#2ecc71;">Green</td><td>Sensor 7</td><td>ADC</td><td>-</td><td>PB0</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:#1f77b4;color:white;">Blue</td><td>Sensor 9</td><td>ADC</td><td>-</td><td>PC1</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:#f1c40f;">Yellow</td><td>Sensor 11</td><td>ADC</td><td>-</td><td>PC0</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:orange;">Orange</td><td>Sensor 2</td><td>ADC</td><td>-</td><td>PA6</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:#2ecc71;">Green</td><td>Sensor 4</td><td>ADC</td><td>-</td><td>PA7</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:gray;color:white;">Gray</td><td>Sensor 6</td><td>ADC</td><td>-</td><td>PA0</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:white;">White</td><td>Sensor 8</td><td>ADC</td><td>-</td><td>PB1</td><td>-</td><td>ADC</td></tr>
  <tr><td>-</td><td style="background:brown;color:white;">Brown</td><td>Sensor 10</td><td>ADC</td><td>-</td><td>PC3</td><td>-</td><td>ADC</td></tr>

  <!-- ================= BUMP SENSORS ================= -->
  <tr>
    <td rowspan="8">Bump Sensors</td>
    <td>-</td>
    <td style="background:black;color:white;">Black</td>
    <td>Ground</td>
    <td>GND</td>
    <td>-</td>
    <td>GND</td>
    <td>-</td>
    <td>Ground</td>
  </tr>
  <tr><td>-</td><td style="background:#e74c3c;color:white;">Red</td><td>BMP0</td><td>Digital Input</td><td>-</td><td>PH0</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:#2ecc71;">Green</td><td>BMP1</td><td>Digital Input</td><td>-</td><td>PH1</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:orange;">Orange</td><td>BMP2</td><td>Digital Input</td><td>-</td><td>PC10</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:#800020;color:white;">Burgundy</td><td>BMP3</td><td>Digital Input</td><td>-</td><td>PC11</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:gray;color:white;">Gray</td><td>BMP4</td><td>Digital Input</td><td>-</td><td>PC12</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:brown;color:white;">Brown</td><td>BMP5</td><td>Digital Input</td><td>-</td><td>PD2</td><td>-</td><td>Input</td></tr>
  <tr><td>-</td><td style="background:white;">White</td><td>Ground</td><td>GND</td><td>-</td><td>GND</td><td>-</td><td>Ground</td></tr>

</table>

---

## Pinout Reference and Wiring Diagram
The diagram below summarizes the final pin assignments used on the Nucleo board. This diagram was used throughout development to track peripheral usage and avoid conflicts between timers, ADC inputs, and communication interfaces.

@image html nucleo_pin_assignment_diagram.png "Pin assignment diagram showing connections between the Nucleo board, motors, encoders, sensors, and peripherals." width=60%

---

## Discussion
The hardware and electrical design evolved alongside the firmware throughout the quarter. Careful pin selection, explicit motor enable control, and deliberate startup behavior were critical to achieving reliable and repeatable operation.

By prioritizing modularity and clarity in both wiring and software interfaces, the system remained flexible enough to support experimentation while maintaining a stable configuration for final demonstrations.
