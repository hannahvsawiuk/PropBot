# GPS and INS Research

## Definitions

- **GPS**: Global Positioning System (GPS) is one component of the Global Navigation Satellite System. Specifically, it refers to the NAVSTAR Global Positioning System, a constellation of satellites developed by the United States Department of Defence (DoD).
- **GNSS**: GNSS stands for Global Navigation Satellite System, and is an umbrella term that encompasses all global satellite positioning systems. This includes constellations of satellites orbiting over the earth’s surface and continuously transmitting signals that enable users to determine their position.
- **Inertial Navigation System (INS):** Navigation device that uses a computer, accelerometer, gyroscope to continually calculate by dead reckoning the position, orientation and velocity of a moving object without the need for external references.
  - Often include barometric altimeter (for altitude) and magnetometer (for heading)
  - It is important to note that an INS is subject to drifts, so navigation based on stand-alone INS suffer a rapid degradation of position over time -**(GPS or GNSS)/INS:** The benefits of using GPS with an INS are that the INS may be calibrated by the GPS signals and that the INS can provide position and angle updates at a quicker rate than GPS. For high dynamic vehicles, such as missiles and aircraft, INS fills in the gaps between GPS positions. Additionally, GPS may lose its signal and the INS can continue to compute the position and angle during the period of lost GPS signal. The two systems are complementary and are often employed together.
- **RTK Positioning:** Satellite navigation technique used to enhance the precision of position data derived form satellite-based position data. It uses measurements of the phase of the signal's carrier wave in addition to the information content of the signal and relies on a single reference station or interpolated virtual station to provide real-time corrections, providing up to centimetre-level accuracy. (`Due to the nature of the robot's application, high-accuracy location will be required, so we should try to source an RTK GPS`)
- **AHRS:** Measurement system for determining roll-pitch-yaw angle changes, accelerations, and heading (for aerospace in particular) (`We just need heading, so this is not as important for us.`)
- **Untethered Dead-Reckoning:**UDR (Untethered Dead Reckoning) refers to the fusion of GNSS data with inertial sensor data. It is an easy-to-use and competitive solution that enables high positioning performance in locations where GNSS signals are poor or unavailable.

## Ground Robot Navigation

- **Dynamic Alignment**: Occurs when there is sufficient motion present to allow an INS sensor to observe heading based on the correlation between the measurement of acceleration from the IMU and the measured change in velocity measured by the GPS receiver
- Slow moving platforms typically do not experience sufficient acceleration during normal operating conditions to perform dynamic alignment. (`This applies to our robot, so we will need to look for an GPS/INS that can handle slow moving vehicles`)
- Deriving accurate heading using magnetic sensors is notoriously difficult for ground vehicles, due to to the presence of ferrous material in both the vehicle and the local environment such as rebar in concrete, the vehicle itself, and other nearby objects.
- Achieving accurate heading measurements using magnetic sensors is only possible in applications where the magnetic environment can be carefully controlled by the user (`This is not our case, because we will be hosting different types of lab equipment, and the robot will need to drive through different types of environments` )

## Navigation Solutions

<table>
<thead>
  <tr>
    <th>Name</th>
    <th>Company</th>
    <th>Cost</th>
    <th>Sensors</th>
    <th>GPS</th>
    <th>Position Accuracy (m)</th>
    <th>Heading Accuracy (deg)</th>
    <th>Roll/Pitch Accuracy (deg)</th>
    <th>Notes</th>
    <th>IMU Frequency (Hz)</th>
    <th>Navigation Frequency (Hz)</th>
    <th>Position Frequency (Hz)</th>
    <th>Power Consumption</th>
    <th>Communication</th>
    <th>PPS</th>
  </tr>
</thead>
<tbody>
  <tr>
    <th>ZED-F9K</th>
    <td>ublox</td>
    <td>Need Quote</td>
    <td>GNSS + IMU</td>
    <td>Multi-band dead reckoning, RTK</td>
    <td>~0.1</td>
    <td>-</td>
    <td>-</td>
    <td>Would need to package somehow, or ask about packaging</td>
    <td>IMU Frequency (Hz)</td>
    <td>-</td>
    <td>30</td>
    <td>2.76V - 3.6V</td>
    <td>2 UART, 1 SPI, 1 DDC</td>
    <td>-</td>
  </tr>
  <tr>
    <th>NEO-M8U</th>
    <td>ublox</td>
    <td>20 USD</td>
    <td>Sensors</td>
    <td>72-channel u-blox M8 engine </td>
    <td>3</td>
    <td>1</td>
    <td>Roll/Pitch Accuracy</td>
    <td>Notes</td>
    <td>100</td>
    <td>30</td>
    <td>2</td>
    <td>Power Consumption</td>
    <td>Communication</td>
    <td>Yes, 30 ns accuracy</td>
  </tr>
  <tr>
    <th>VN-200</th>
    <td>VectorNav</td>
    <td>2900USD</td>
    <td>GPS + IMU</td>
    <td>50-channel u-blox GPS receiver</td>
    <td>-</td>
    <td>0.3</td>
    <td>0.1</td>
    <td>agricultural robots, already in rugged casing, advanced Kalman filtering algorithms for position, velocity, and attitude</td>
    <td>800</td>
    <td>500</td>
    <td>-</td>
    <td>80 mA @ 5 V</td>
    <td>1 UART, 1 SPI</td>
    <td>Yes, need to double check</td>
  </tr>
  <tr>
    <th>VN-300</th>
    <td>VectorNav</td>
    <td>5000USD</td>
    <td>Dual Antenna GNSS + IMU</td>
    <td>Two onboard high-sensitivity 72-channel, L1, GNSS receivers, RTK</td>
    <td>-</td>
    <td>0.3 (GPS-Compass)</td>
    <td>0.1</td>
    <td>advanced Kalman filtering algorithms for position, velocity, and attitude</td>
    <td>800</td>
    <td>400</td>
    <td>-</td>
    <td>250 mA @ 5 V</td>
    <td>1 UART, 1 SPI</td>
    <td>Yes, need to check</td>
  </tr>
  <tr>
    <th>SPAN CPT7</th>
    <td>Novatel</td>
    <td>Need Quote</td>
    <td>GNSS + IMU</td>
    <td>555 channel GNSS</td>
    <td>0.01 (with RTK), otherwise 0.4-2.5</td>
    <td>0.05- 0.08</td>
    <td>0.007 - 0.035</td>
    <td>Dead-reckoning, in ruggedized casing</td>
    <td>300</td>
    <td>20</td>
    <td>20</td>
    <td>7W, 9-32VDC</td>
    <td>1 RS-232, 1 RS-422, 1 Ethernet, 1 USB, 1 CAN</td>
    <td>Yes</td>
  </tr>
  <tr>
    <th>Duro Inertial Ruggedized Receiver</th>
    <td>SwiftNav</td>
    <td>3995 USD</td>
    <td>Sensors</td>
    <td>GPS</td>
    <td>0.75</td>
    <td>0.8</td>
    <td>0.2</td>
    <td>Smooth dead-reckoning, in ruggedized casing, velocity accuracy is 0.06m/s, used by clearpath</td>
    <td>IMU Frequency (Hz)</td>
    <td>10</td>
    <td>10</td>
    <td>5W</td>
    <td>2 RS232, Ethernet, 1 CAN with selectable termination resistor, power outputs</td>
    <td>Yes</td>
  </tr>
  <tr>
    <th>BD992-INS Receiver</th>
    <td>Trimble</td>
    <td>Need quote</td>
    <td>GNSS + IMU</td>
    <td>336 Channel Maxwell 7 chip</td>
    <td>0.05-1</td>
    <td>0.09</td>
    <td>0.10</td>
    <td>Baud rates up to 460,800, need to package</td>
    <td>100</td>
    <td>-</td>
    <td>100</td>
    <td>Power Consumption</td>
    <td>3 RS-232, 1 CAN, 1 USB, 1 Ethernet</td>
    <td>PPS</td>
  </tr>
</tbody>
</table>

# Sources

- https://en.wikipedia.org/wiki/Inertial_navigation_system
- https://en.wikipedia.org/wiki/GPS/INS
- https://en.wikipedia.org/wiki/Real-time_kinematic
- http://www.terrisgps.com/gnss-gps-differences-explained/
- https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5621051/
- https://en.wikipedia.org/wiki/Attitude_Heading_Reference_System
- https://www.u-blox.com/de/udr-untethered-dead-reckoning