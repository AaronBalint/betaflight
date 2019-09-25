![Betaflight](docs/assets/images/betaflightplan.png)

**This fork of Betaflight provides a robust mission flight plan mode suitable for multi-rotor craft with GPS and no compass.**

**Version 4.2.0 release**
- Improved gps rescue algorithm to prevent high throttle settings at home position.
- Improved waypoint handling using derivative positioning.
- Introduced proportional roll for high speed flight plan execution.

**Features:**
- Unified algorithm for GPS rescue and flight plan modes.
- Simplified GPS rescue state engine.
- Continuous GPS orientation calibration.
- Home position calibration to easily verify GPS accuracy.
- Flight plan mission can be paused / continued.
- Flight plan execution speed can be directly controlled by throttle input.

**FOR SAFETY PLEASE OBSERVE THE FOLLOWING:**

- Each way-point should have a 20 meter radius of clear air space.  The mission flight plan algorithm is NOT intended for high precision autonomous flight.
- On first use set conservative values for gps_rescue_throttle_max  (less than 100 from the hover point) and set gps_rescue_angle to no more than 30 degrees.
- Make sure gps_rescue_throttle_hover is set accurately.
- Align the front of the craft towards North before connecting the battery.
- Keep the throttle towards the low position when launching a flight plan near the home location.

**Configuration settings (CLI):**

**gps_rescue_max_angle**

This is the maximum flight angle for GPS recovery and mission flight plan manoeuvres.  A higher value will result in faster horizontal movement and is independent of  gps_rescue_throttle_max.

**gps_rescue_throttle_hover**

This is the throttle hover position and should be set as accurately as possible.

**gps_rescue_throttle_max**

This is the maximum throttle used  for altitude positioning and determines how aggressively the quad will climb to the recovery and flight plan altitudes. Must be greater than gps_rescue_throttle_hover.

**gps_rescue_throttle_min**

This value will determine how quickly the quad will fall to the required altitude. Must be less than gps_rescue_throttle_hover.

**gps_rescue_return_alt**

GPS rescue only.  The return to home altitude in meters.

**gps_rescue_home_alt**

GPS rescue only.  The at home altitude in meters.

**gps_rescue_approach_dist**

The distance from the home position at which maximum flight speed is gradually reduced towards the hover setting.

**gps_rescue_cal_lat**

**gps_rescue_cal_lon**

Home point calibration position.  When not armed, the OSD GPS distance is set against the calibration point. This allows you to easily check that the GPS position is within your required mission tolerance.

**fplan_waypoints**

Total number of way-points in the flight plan.  Zero disables the flight plan. A single way-point will make the quad fly to and hold the corresponding position. Two or more way-points will be flown in a continuous cycle.

**fplan_lat_01 .. 10**

**fplan_lon_01 .. 10**

**fplan_alt_01 .. 10**

Flight plan way-point latitude, longitude in degrees and altitude from launch in meters. Up to 10 way-points can be set.

**IMPORTANT NOTES:**

- Flight plan execution requires both GPS RESCUE and TELEMETRY modes to be active.
- The flight plan mission will continue to execute if both modes are active during failsafe.
- GPS orientation is only updated when the ground speed exceeds 2m/s (7 km/h).
- The throttle controls the speed of flight plan execution during a mission.
- The flight plan is reset to the first way-point when only GPS RESCUE mode is enabled.
- You can pause / continue a flight plan mission by exiting / entering GPS RESCUE and TELEMETRY modes simultaneously.
- All altitude settings are relative to the altitude at launch.

**OSD**

When unarmed the Home Distance will display the distance to the home calibration point.

When unarmed the Altitude will display the current GPS altitude above sea level. When armed the Altitude will be relative to the launch altitude.

When a flight plan mission is activated, The Home Direction and Home Distance will be relative to the next way-point.  The Home Distance will also display the current active way-point number (W1-10).

**Betaflight_plan Releases**

https://github.com/AaronBalint/betaflight_plan/releases



**What is 'robust' GPS autonomous flight?**

Put simply, this is where the orientation of the quadcopter is continuously recalibrated using the GPS course over ground data. This technique elegantly resolves two issues:
1) Recovering from inaccurate GPS positional data (particularly at low ground speed) and
2) Correcting the flight path attitude due to cross-wind.

Any lateral movement due to cross-wind will introduce an error to the calculated orientation, however this error actually turns the quadcopter into the cross-wind.  Natural feedback from continuous calibration results in accurate directional flight to the next way-point.

