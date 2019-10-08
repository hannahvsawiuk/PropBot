# GPS and INS Research

## Definitions

- **GPS**: Global Positioning System (GPS) is one component of the Global Navigation Satellite System. Specifically, it refers to the NAVSTAR Global Positioning System, a constellation of satellites developed by the United States Department of Defence (DoD).
- **GNSS**: GNSS stands for Global Navigation Satellite System, and is an umbrella term that encompasses all global satellite positioning systems. This includes constellations of satellites orbiting over the earth’s surface and continuously transmitting signals that enable users to determine their position.
- **Inertial Navigation System (INS):** Navigation device that uses a computer, accelerometer, gyroscope to continually calculate by dead reckoning the position, orientation and velocity of a moving object without the need for external references.
  - Often include barometric altimeter (for altitude) and magnetometer (for heading)
  - It is important to note that an INS is subject to drifts, so navigation based on stand-alone INS suffer a rapid degradation of position over time -**(GPS or GNSS)/INS:** The benefits of using GPS with an INS are that the INS may be calibrated by the GPS signals and that the INS can provide position and angle updates at a quicker rate than GPS. For high dynamic vehicles, such as missiles and aircraft, INS fills in the gaps between GPS positions. Additionally, GPS may lose its signal and the INS can continue to compute the position and angle during the period of lost GPS signal. The two systems are complementary and are often employed together.
- **RTK Positioning:** Satellite navigation technique used to enhance the precision of position data derived form satellite-based position data. It uses measurements of the phase of the signal's carrier wave in addition to the information content of the signal and relies on a single reference station or interpolated virtual station to provide real-time corrections, providing up to centimetre-level accuracy. (`Due to the nature of the robot's application, high-accuracy location will be required, so we should try to source an RTK GPS`)
- **AHRS:** Measurement system for determining roll-pitch-yaw angle changes, accelerations, and heading (for aerospace in particular) (`We just need heading, so this is not as important for us.`)

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
    <th>Position Accuracy</th>
    <th>Heading Accuracy</th>
    <th>Roll/Pitch Accuracy</th>
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
    <th>Vector</th>
    <td>Colu1</td>
    <td>Column 2</td>
    <td>Column 3</td>
  </tr>
  <tr>
    <td>Custom Table Content</td>
    <td>Column 4</td>
    <td>Column 5</td>
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