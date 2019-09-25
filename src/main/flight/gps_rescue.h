/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"

#include "pg/pg.h"

// flight plan behaviour on failsafe can be controlled by BOXTELEMETRY failsafe state
#define IS_FLIGHT_PLAN_MODE (IS_RC_MODE_ACTIVE(BOXTELEMETRY) && gpsRescueConfig()->total_waypoints > 0)

#define FLIGHTPLAN_MAX_WAYPOINT_COUNT 10

#define FLIGHTPLAN_DATA_LENGTH 16

typedef struct gpsRescue_s {
    uint16_t angle; //degrees
    uint16_t initialAltitudeM; //meters
    uint16_t descentDistanceM; //meters
    uint16_t rescueGroundspeed; //centimeters per second
    uint16_t throttleP, throttleI, throttleD;
    uint16_t yawP;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint16_t velP, velI, velD;
    uint8_t minSats;
    uint16_t minRescueDth; //meters
    uint8_t sanityChecks;
    uint8_t allowArmingWithoutFix;
    uint8_t useMag;
    uint16_t targetLandingAltitudeM; //meters
    //uint16_t targetLandingDistanceM; //meters
    uint8_t total_waypoints;
    char gps_callibration_latitude [FLIGHTPLAN_DATA_LENGTH + 1];
    char gps_callibration_longitude[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_01[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_02[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_03[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_04[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_05[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_06[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_07[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_08[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_09[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lat_10[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_01[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_02[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_03[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_04[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_05[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_06[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_07[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_08[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_09[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_lon_10[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_01[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_02[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_03[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_04[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_05[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_06[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_07[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_08[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_09[FLIGHTPLAN_DATA_LENGTH + 1];
    char fp_alt_10[FLIGHTPLAN_DATA_LENGTH + 1];
    
} gpsRescueConfig_t;

typedef enum {
    RESCUE_HEALTHY,
    RESCUE_GPSLOST,
    RESCUE_CRASH_FLIP_DETECTED,
} rescueFailureState_e;

PG_DECLARE(gpsRescueConfig_t, gpsRescueConfig);

extern int32_t gpsRescueAngle[ANGLE_INDEX_COUNT]; //NOTE: ANGLES ARE IN CENTIDEGREES

void updateGPSRescueState(void);
void rescueNewGpsData(void);

float gpsRescueGetYawRate(void);
float gpsRescueGetThrottle(void);
bool gpsRescueIsConfigured(void);
bool gpsRescueIsAvailable(void);
rescueFailureState_e gpsRescueGetRescueState (void);
bool gpsRescueIsDisabled(void);
bool gpsRescueDisableMag(void);

int32_t getGPSAltitudeCm(void);
int getFlightplanTargetWaypoint(void);
uint16_t getGPSDistanceToHome(void);
int16_t getGPSDirectionToHome(void);
