/*
 *  ATRVJR Driver, modified by Mikhail Medvedev from
 *  B21 Driver - By David Lu!! 2/2010
 *  Modified from Player code
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <rflex/atrvjr_driver.h>
#include <rflex/atrvjr_config.h>
#include <math.h>
#include <stdio.h>

ATRVJR::ATRVJR() {
    found_distance = false;
}

ATRVJR::~ATRVJR() {
}

float ATRVJR::getDistance() {
    if (!found_distance && isOdomReady()) {
        first_distance = distance;
        found_distance = true;
    }

    return (distance-first_distance) / (float) ODO_DISTANCE_CONVERSION;
}

float ATRVJR::getBearing() {
    return (bearing-HOME_BEARING) / (float) ODO_ANGLE_CONVERSION;
}

float ATRVJR::getTranslationalVelocity() const {
    return transVelocity / (float) ODO_DISTANCE_CONVERSION;
}

float ATRVJR::getRotationalVelocity() const {
    return rotVelocity / (float) ODO_ANGLE_CONVERSION;
}

float ATRVJR::getVoltage() const {
    if (voltage==0.0)
        return 0.0;
    else
        return voltage/100.0 + POWER_OFFSET;
}

bool ATRVJR::isPluggedIn() const {
    float v = getVoltage();
    if (v>PLUGGED_THRESHOLD)
        return true;
    else
        return false;
}

int ATRVJR::getNumBodySonars() const {
    return SONARS_PER_RING[BODY_INDEX];
}

void ATRVJR::getBodySonarReadings(float* readings) const {
    getSonarReadings(BODY_INDEX, readings);
}

void ATRVJR::getBodySonarPoints(sensor_msgs::PointCloud* cloud) const {
    getSonarPoints(BODY_INDEX, cloud);
}

void ATRVJR::setSonarPower(bool on) {
    unsigned long echo, ping, set, val;
    if (on) {
        echo = SONAR_ECHO_DELAY;
        ping = SONAR_PING_DELAY;
        set = SONAR_SET_DELAY;
        val = 2;
    } else {
        echo = ping = set = val = 0;
    }
    configureSonar(echo, ping, set, val);
}


void ATRVJR::setMovement( float tvel, float rvel,
                       float acceleration ) {
    setVelocity(tvel * ODO_DISTANCE_CONVERSION,
                rvel * ODO_ANGLE_CONVERSION,
                acceleration * ODO_DISTANCE_CONVERSION);
}


void ATRVJR::getSonarReadings(const int ringi, float* adjusted_ranges) const {
    int i = 0;
    for (int x = SONAR_RING_BANK_BOUND[ringi];x<SONAR_RING_BANK_BOUND[ringi+1];x++) {
        for (int y=0;y<SONARS_PER_BANK[x];y++) {
            int range = sonar_ranges[x*SONAR_MAX_PER_BANK+y];		//??Find how ranges are calculated ---> sonar_ranges[x*SONAR_MAX_PER_BANK+y]
            if (range > SONAR_MAX_RANGE)
                range = SONAR_MAX_RANGE;
            float fRange = range / (float) RANGE_CONVERSION;		//convert to meters
            adjusted_ranges[i] = fRange;
            i++;
        }
    }
}

void ATRVJR::getSonarPoints(const int ringi, sensor_msgs::PointCloud* cloud) const {
    int numSonar = SONARS_PER_RING[ringi];
    float* readings = new float[numSonar];
    getSonarReadings(ringi, readings);
    cloud->points.resize(numSonar);
    int c = 0;
    double sonar_ring_distance[numSonar];
    double x,y,angle;
    double distance_from_center_of_rover;

    for (int i = 0; i < numSonar; i++)
    {
        if (readings[i] < SONAR_MAX_RANGE/ (float) RANGE_CONVERSION)
        {
            angle = SONAR_RING_ANGLES[i];
	    if (angle < 0)
	    {
    	        angle += 2*M_PI;//convert negative radians to 2pi
	    }
        	
            x = SONAR_RING_X_DISTANCE[i];
	    y = SONAR_RING_Y_DISTANCE[i];
	    x /= 1000;
	    y /= 1000;
	    sonar_ring_distance[i] = sqrt((x*x)+(y*y));
	        
            distance_from_center_of_rover = sonar_ring_distance[i] + readings[i];
	    cloud->points[c].x = cos(angle) * distance_from_center_of_rover;
            cloud->points[c].y = sin(angle) * distance_from_center_of_rover;
            cloud->points[c].z = SONAR_RING_HEIGHT[ringi];
            c++;
        }
    }
}

void ATRVJR::processDioEvent(unsigned char address, unsigned short data) {

    if (address == HEADING_HOME_ADDRESS) {
        home_bearing = bearing;
        printf("ATRVJR Home %f \n", home_bearing / (float) ODO_ANGLE_CONVERSION);
    }
    else {
        printf("ATRVJR DIO: address 0x%02x (%d) value 0x%02x (%d)\n", address, address, data, data);
    }
}


