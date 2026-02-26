#include "Localizer.hpp"
#include <cmath>

// Constructor with distance sensors
Localizer::Localizer(lemlib::Chassis& chassis, 
                     pros::Distance* frontDist,
                     pros::Distance* backDist,
                     pros::Distance* leftDist,
                     pros::Distance* rightDist)
    : m_chassis(chassis),
      m_frontDist(frontDist),
      m_backDist(backDist),
      m_leftDist(leftDist),
      m_rightDist(rightDist),
      m_frontOpt(nullptr),
      m_backOpt(nullptr),
      m_leftOpt(nullptr),
      m_rightOpt(nullptr),
      m_useDistanceSensors(true) {}

// Constructor with optical sensors
Localizer::Localizer(lemlib::Chassis& chassis,
                     pros::Optical* frontOpt,
                     pros::Optical* backOpt,
                     pros::Optical* leftOpt,
                     pros::Optical* rightOpt)
    : m_chassis(chassis),
      m_frontDist(nullptr),
      m_backDist(nullptr),
      m_leftDist(nullptr),
      m_rightDist(nullptr),
      m_frontOpt(frontOpt),
      m_backOpt(backOpt),
      m_leftOpt(leftOpt),
      m_rightOpt(rightOpt),
      m_useDistanceSensors(false) {}

void Localizer::setSensorOffsets(float front, float back, float left, float right) {
    frontSensorOffset = front;
    backSensorOffset = back;
    leftSensorOffset = left;
    rightSensorOffset = right;
}

float Localizer::normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

float Localizer::proximityToInches(int proximity) {
    // Optical sensor proximity is 0-255, with 255 being closest
    // Rough estimate: max detection range is about 4 inches (100mm)
    if (proximity <= 0) return -1; // No detection
    // Inverse relationship: higher proximity = closer
    // This is a rough linear approximation
    return 4.0f * (255.0f - proximity) / 255.0f;
}

float Localizer::getFrontDistance() {
    if (m_useDistanceSensors && m_frontDist) {
        int dist = m_frontDist->get();
        if (dist > 0 && dist < 9999) {
            return mmToInches(dist);
        }
    } else if (m_frontOpt) {
        int prox = m_frontOpt->get_proximity();
        if (prox > 10) { // Minimum threshold for valid reading
            return proximityToInches(prox);
        }
    }
    return -1;
}

float Localizer::getBackDistance() {
    if (m_useDistanceSensors && m_backDist) {
        int dist = m_backDist->get();
        if (dist > 0 && dist < 9999) {
            return mmToInches(dist);
        }
    } else if (m_backOpt) {
        int prox = m_backOpt->get_proximity();
        if (prox > 10) {
            return proximityToInches(prox);
        }
    }
    return -1;
}

float Localizer::getLeftDistance() {
    if (m_useDistanceSensors && m_leftDist) {
        int dist = m_leftDist->get();
        if (dist > 0 && dist < 9999) {
            return mmToInches(dist);
        }
    } else if (m_leftOpt) {
        int prox = m_leftOpt->get_proximity();
        if (prox > 10) {
            return proximityToInches(prox);
        }
    }
    return -1;
}

float Localizer::getRightDistance() {
    if (m_useDistanceSensors && m_rightDist) {
        int dist = m_rightDist->get();
        if (dist > 0 && dist < 9999) {
            return mmToInches(dist);
        }
    } else if (m_rightOpt) {
        int prox = m_rightOpt->get_proximity();
        if (prox > 10) {
            return proximityToInches(prox);
        }
    }
    return -1;
}

bool Localizer::resetX(bool useLeftWall) {
    float distance;
    float sensorOffset;
    float wallX;
    
    if (useLeftWall) {
        distance = getLeftDistance();
        sensorOffset = leftSensorOffset;
        wallX = -FIELD_HALF_SIZE; // Left wall at X = -72
    } else {
        distance = getRightDistance();
        sensorOffset = rightSensorOffset;
        wallX = FIELD_HALF_SIZE; // Right wall at X = +72
    }
    
    if (distance < 0) return false; // Invalid reading
    
    // Calculate robot center X position
    // Robot center = wall position + distance from wall + sensor offset (towards center)
    float newX;
    if (useLeftWall) {
        newX = wallX + distance + sensorOffset;
    } else {
        newX = wallX - distance - sensorOffset;
    }
    
    // Get current pose and update X
    lemlib::Pose currentPose = m_chassis.getPose();
    m_chassis.setPose(newX, currentPose.y, currentPose.theta);
    
    return true;
}

bool Localizer::resetY(bool useFrontWall) {
    float distance;
    float sensorOffset;
    float wallY;
    
    if (useFrontWall) {
        distance = getFrontDistance();
        sensorOffset = frontSensorOffset;
        wallY = FIELD_HALF_SIZE; // Front wall at Y = +72
    } else {
        distance = getBackDistance();
        sensorOffset = backSensorOffset;
        wallY = -FIELD_HALF_SIZE; // Back wall at Y = -72
    }
    
    if (distance < 0) return false; // Invalid reading
    
    // Calculate robot center Y position
    float newY;
    if (useFrontWall) {
        newY = wallY - distance - sensorOffset;
    } else {
        newY = wallY + distance + sensorOffset;
    }
    
    // Get current pose and update Y
    lemlib::Pose currentPose = m_chassis.getPose();
    m_chassis.setPose(currentPose.x, newY, currentPose.theta);
    
    return true;
}

bool Localizer::resetXWithHeading(float wallX) {
    lemlib::Pose pose = m_chassis.getPose();
    float heading = normalizeAngle(pose.theta);
    
    float distance = -1;
    float sensorOffset = 0;
    bool sensorPointsPositiveX = false;
    
    // Determine which sensor is pointing towards the wall based on heading
    // Heading 0 = facing positive Y, 90 = facing positive X, 180 = facing negative Y, 270 = facing negative X
    
    if (wallX > 0) { // Right wall (X = +72)
        // Which sensor points to the right (+X)?
        if (heading >= 45 && heading < 135) {
            // Robot facing right-ish, use front sensor
            distance = getFrontDistance();
            sensorOffset = frontSensorOffset;
        } else if (heading >= 225 && heading < 315) {
            // Robot facing left-ish, use back sensor
            distance = getBackDistance();
            sensorOffset = backSensorOffset;
        } else if (heading >= 315 || heading < 45) {
            // Robot facing forward-ish, use right sensor
            distance = getRightDistance();
            sensorOffset = rightSensorOffset;
        } else {
            // Robot facing backward-ish, use left sensor
            distance = getLeftDistance();
            sensorOffset = leftSensorOffset;
        }
    } else { // Left wall (X = -72)
        if (heading >= 225 && heading < 315) {
            // Robot facing left-ish, use front sensor
            distance = getFrontDistance();
            sensorOffset = frontSensorOffset;
        } else if (heading >= 45 && heading < 135) {
            // Robot facing right-ish, use back sensor
            distance = getBackDistance();
            sensorOffset = backSensorOffset;
        } else if (heading >= 315 || heading < 45) {
            // Robot facing forward-ish, use left sensor
            distance = getLeftDistance();
            sensorOffset = leftSensorOffset;
        } else {
            // Robot facing backward-ish, use right sensor
            distance = getRightDistance();
            sensorOffset = rightSensorOffset;
        }
    }
    
    if (distance < 0) return false;
    
    // Calculate new X position
    float newX;
    if (wallX > 0) {
        newX = wallX - distance - sensorOffset;
    } else {
        newX = wallX + distance + sensorOffset;
    }
    
    m_chassis.setPose(newX, pose.y, pose.theta);
    return true;
}

bool Localizer::resetYWithHeading(float wallY) {
    lemlib::Pose pose = m_chassis.getPose();
    float heading = normalizeAngle(pose.theta);
    
    float distance = -1;
    float sensorOffset = 0;
    
    if (wallY > 0) { // Front wall (Y = +72)
        if (heading >= 315 || heading < 45) {
            // Robot facing forward, use front sensor
            distance = getFrontDistance();
            sensorOffset = frontSensorOffset;
        } else if (heading >= 135 && heading < 225) {
            // Robot facing backward, use back sensor
            distance = getBackDistance();
            sensorOffset = backSensorOffset;
        } else if (heading >= 45 && heading < 135) {
            // Robot facing right, use left sensor (points to +Y)
            distance = getLeftDistance();
            sensorOffset = leftSensorOffset;
        } else {
            // Robot facing left, use right sensor (points to +Y)
            distance = getRightDistance();
            sensorOffset = rightSensorOffset;
        }
    } else { // Back wall (Y = -72)
        if (heading >= 135 && heading < 225) {
            // Robot facing backward, use front sensor
            distance = getFrontDistance();
            sensorOffset = frontSensorOffset;
        } else if (heading >= 315 || heading < 45) {
            // Robot facing forward, use back sensor
            distance = getBackDistance();
            sensorOffset = backSensorOffset;
        } else if (heading >= 225 && heading < 315) {
            // Robot facing left, use left sensor
            distance = getLeftDistance();
            sensorOffset = leftSensorOffset;
        } else {
            // Robot facing right, use right sensor
            distance = getRightDistance();
            sensorOffset = rightSensorOffset;
        }
    }
    
    if (distance < 0) return false;
    
    // Calculate new Y position
    float newY;
    if (wallY > 0) {
        newY = wallY - distance - sensorOffset;
    } else {
        newY = wallY + distance + sensorOffset;
    }
    
    m_chassis.setPose(pose.x, newY, pose.theta);
    return true;
}

bool Localizer::autoReset(int proximityThreshold, int distanceThreshold) {
    bool anyReset = false;
    lemlib::Pose pose = m_chassis.getPose();
    float heading = normalizeAngle(pose.theta);
    
    // Check each sensor and reset if close enough to a wall
    float frontDist = getFrontDistance();
    float backDist = getBackDistance();
    float leftDist = getLeftDistance();
    float rightDist = getRightDistance();
    
    float thresholdInches = m_useDistanceSensors ? mmToInches(distanceThreshold) : 3.0f;
    
    // Determine which walls the sensors are pointing at based on heading
    // and reset if the reading is valid and within threshold
    
    // Front sensor
    if (frontDist > 0 && frontDist < thresholdInches) {
        if (heading >= 315 || heading < 45) {
            // Front sensor points to +Y wall
            float newY = FIELD_HALF_SIZE - frontDist - frontSensorOffset;
            m_chassis.setPose(pose.x, newY, pose.theta);
            anyReset = true;
        } else if (heading >= 45 && heading < 135) {
            // Front sensor points to +X wall
            float newX = FIELD_HALF_SIZE - frontDist - frontSensorOffset;
            m_chassis.setPose(newX, pose.y, pose.theta);
            anyReset = true;
        } else if (heading >= 135 && heading < 225) {
            // Front sensor points to -Y wall
            float newY = -FIELD_HALF_SIZE + frontDist + frontSensorOffset;
            m_chassis.setPose(pose.x, newY, pose.theta);
            anyReset = true;
        } else {
            // Front sensor points to -X wall
            float newX = -FIELD_HALF_SIZE + frontDist + frontSensorOffset;
            m_chassis.setPose(newX, pose.y, pose.theta);
            anyReset = true;
        }
    }
    
    // Similar logic can be added for back, left, right sensors as needed
    
    return anyReset;
}