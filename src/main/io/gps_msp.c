/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "build/build_config.h"


#if defined(USE_GPS_PROTO_MSP)

#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/gps_private.h"
#include "io/serial.h"

#include "scheduler/protothreads.h"

typedef struct __attribute__((packed)) {
    uint32_t msTOW;
    uint8_t  fixType;
    uint8_t  satellitesInView;
    uint16_t horizontalPosAccuracy;     // [cm]
    uint16_t verticalPosAccuracy;       // [cm]
    uint16_t horizontalVelAccuracy;     // [cm/s]
    uint16_t hdop;
    int32_t  longitude;
    int32_t  latitude;
    int32_t  mslAltitude;       // cm
    int32_t  nedVelNorth;       // cm/s
    int32_t  nedVelNorthEast;
    int32_t  nedVelNorthDown;
    int32_t  groundSpeed;
    int16_t  groundCourse;      // deg * 100
    int16_t  trueYaw;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} mspGpsDataMessage_t;

static bool newDataReady;

void gpsRestartMSP(void)
{
    // NOP
}

void gpsHandleMSP(void)
{
    if (newDataReady) {
        gpsProcessNewSolutionData();
    }
}

void mspGPSReceiveNewData(uint8_t * bufferPtr)
{
    mspGpsDataMessage_t * pkt = (mspGpsDataMessage_t *)bufferPtr;

    gpsSol.fixType   = (pkt->fixType >= 3);
    gpsSol.numSat    = pkt->satellitesInView;
    gpsSol.llh.lon   = pkt->longitude;
    gpsSol.llh.lat   = pkt->latitude;
    gpsSol.llh.alt   = pkt->mslAltitude;
    gpsSol.velNED[X] = pkt->nedVelNorth;
    gpsSol.velNED[Y] = pkt->nedVelNorthEast;
    gpsSol.velNED[Z] = pkt->nedVelNorthDown;
    gpsSol.groundSpeed = pkt->groundSpeed;
    gpsSol.groundCourse = pkt->groundCourse / 10;   // in deg * 10
    gpsSol.eph = gpsConstrainEPE(pkt->horizontalPosAccuracy / 10);
    gpsSol.epv = gpsConstrainEPE(pkt->verticalPosAccuracy / 10);
    gpsSol.hdop = gpsConstrainHDOP(pkt->hdop);
    gpsSol.flags.validVelNE = 1;
    gpsSol.flags.validVelD = 1;
    gpsSol.flags.validEPE = 1;

    gpsSol.time.year   = pkt->year;
    gpsSol.time.month  = pkt->month;
    gpsSol.time.day    = pkt->day;
    gpsSol.time.hours  = pkt->hour;
    gpsSol.time.minutes = pkt->min;
    gpsSol.time.seconds = pkt->sec;
    gpsSol.time.millis  = 0;

    gpsSol.flags.validTime = (pkt->fixType >= 3);

    newDataReady = true;
}

#endif
