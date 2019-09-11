![Betaflight](docs/assets/images/betaflightplan.png)

**This fork of Betaflight provides a robust flight plan mode suitable for multi-rotor craft with GPS and no compass.**

**Features:**
- Unified algorithm for GPS rescue and Flight plan modes.
- Simplified GPS rescue state engine.
- Continuous GPS orientation calibration.
- Home position calibration to easily verify GPS accuracy.
- Flight plan can be paused / continued.
- Flight plan execution speed can be directly controlled by throttle input.
- Flight plan can override failsafe for full autonomous flight.

**FOR SAFETY PLEASE OBSERVE THE FOLLOWING:**

- Each way-point should have a 20 meter radius of clear air space.  The flight plan algorithm is NOT intended for high precision autonomous flight.
- On first use set conservative values for gps_rescue_throttle_max  (less than 100 from the hover point) and set gps_rescue_angle to no more than 30 degrees.
- Make sure gps_rescue_throttle_hover is set accurately. 

**Configuration settings (CLI):**

**gps_rescue_angle**

This is the maximum flight angle for GPS recovery and flight plan manoeuvres.  A higher value will result in faster horizontal movement and is independent of  gps_rescue_throttle_max.

**gps_rescue_throttle_hover**

This is the throttle hover position and should be set as accurately as possible.

**gps_rescue_throttle_max**

This is the maximum throttle used  for altitude positioning and determines how aggressively the quad will climb to the recovery and flight plan altitudes. Must be greater than gps_rescue_throttle_hover.

**gps_rescue_throttle_min**

This value will determine how quickly the quad will fall to the required altitude. Must be less than gps_rescue_throttle_hover.

**gps_rescue_initial_alt**

GPS rescue only.  The return to home altitude in meters.

**gps_rescue_landing_alt**

GPS rescue only.  The at home altitude in meters.

**gps_rescue_approach_dist**

The distance from the home or way-point position at which maximum throttle is gradually reduced towards the hover setting.

**gps_rescue_landing_dist**

The distance from the home or way-point position at which the maximum flight angle is reduced towards zero.

**gps_rescue_cal_lat**

**gps_rescue_cal_lon**

Home point calibration position.  When not armed, the OSD GPS distance is set against the calibration point. This allows you to easily check that the GPS position is within your required tolerance.

**fplan_waypoints**

Total number of way-points in the flight plan.  Zero disables the flight plan. A single way-point will make the quad fly to and hold the corresponding position. More than one way-point will be flown in a continuous cycle.

**fplan_lat_01 .. 10**

**fplan_lon_01 .. 10**

**fplan_alt_01 .. 10**

Flight plan way-point latitude, longitude in degrees and altitude from launch in meters. Up to 10 way-points can be set.

**IMPORTANT NOTES:**

- Flight plan execution requires both GPS RESCUE and TELEMETRY modes to be active.
- The flight plan will continue to execute if both modes are active during failsafe.
- GPS orientation is only updated when the ground speed exceeds 3m/s (11 km/h).
- When flying a flight plan the throttle controls the speed of flight plan execution.
- The flight plan is reset to the first way-point when only GPS RESCUE mode is enabled.
- You can pause / continue a flight plan by exiting / entering GPS RESCUE and TELEMETRY modes simultaneously.
- All altitude settings are relative to the altitude at launch.
