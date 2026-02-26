#pragma once

#include "pros/optical.hpp"
#include "pros/distance.hpp"
#include "lemlib/chassis/chassis.hpp"
#include <cmath>

/**
 * @brief Localizer class for resetting robot coordinates using wall detection
 * 
 * Uses optical or distance sensors pointed at walls to detect proximity
 * and reset odometry coordinates to reduce drift during autonomous.
 * 
 * Field is 12x12 feet (144x144 inches), with (0,0) at center.
 * Walls are at X = ±72 inches and Y = ±72 inches.
 */
class Localizer {
public:
    // Field dimensions (half-width since 0,0 is center)
    static constexpr float FIELD_HALF_SIZE = 72.0f; // inches (12ft / 2)
    
    // Sensor mounting offsets from robot center (configure these based on your robot)
    float frontSensorOffset = 7.0f;   // inches from center to front sensor
    float backSensorOffset = 7.0f;    // inches from center to back sensor
    float leftSensorOffset = 5.0f;    // inches from center to left sensor
    float rightSensorOffset = 5.0f;   // inches from center to right sensor

    /**
     * @brief Constructor with distance sensors (recommended for wall detection)
     * @param chassis Reference to LemLib chassis for getting/setting pose
     * @param frontDist Distance sensor facing forward (can be nullptr)
     * @param backDist Distance sensor facing backward (can be nullptr)
     * @param leftDist Distance sensor facing left (can be nullptr)
     * @param rightDist Distance sensor facing right (can be nullptr)
     */
    Localizer(lemlib::Chassis& chassis, 
              pros::Distance* frontDist = nullptr,
              pros::Distance* backDist = nullptr,
              pros::Distance* leftDist = nullptr,
              pros::Distance* rightDist = nullptr);

    /**
     * @brief Constructor with optical sensors (limited range ~100mm)
     * @param chassis Reference to LemLib chassis for getting/setting pose
     * @param frontOpt Optical sensor facing forward (can be nullptr)
     * @param backOpt Optical sensor facing backward (can be nullptr)
     * @param leftOpt Optical sensor facing left (can be nullptr)
     * @param rightOpt Optical sensor facing right (can be nullptr)
     */
    Localizer(lemlib::Chassis& chassis,
              pros::Optical* frontOpt,
              pros::Optical* backOpt,
              pros::Optical* leftOpt,
              pros::Optical* rightOpt);

    /**
     * @brief Set sensor mounting offsets from robot center
     * @param front Distance from center to front sensor (inches)
     * @param back Distance from center to back sensor (inches)
     * @param left Distance from center to left sensor (inches)
     * @param right Distance from center to right sensor (inches)
     */
    void setSensorOffsets(float front, float back, float left, float right);

    /**
     * @brief Reset X coordinate using left/right wall detection
     * Call this when robot is parallel to Y-axis walls (heading ~0 or ~180)
     * @param useLeftWall If true, uses left sensor and assumes near left wall (X = -72)
     *                    If false, uses right sensor and assumes near right wall (X = +72)
     * @return true if reset was successful, false if sensor reading invalid
     */
    bool resetX(bool useLeftWall);

    /**
     * @brief Reset Y coordinate using front/back wall detection
     * Call this when robot is parallel to X-axis walls (heading ~90 or ~270)
     * @param useFrontWall If true, uses front sensor and assumes near front wall (Y = +72)
     *                     If false, uses back sensor and assumes near back wall (Y = -72)
     * @return true if reset was successful, false if sensor reading invalid
     */
    bool resetY(bool useFrontWall);

    /**
     * @brief Automatically detect and reset coordinates based on current heading and wall proximity
     * Uses heading to determine which walls the sensors are pointing at
     * @param proximityThreshold For optical sensors: 0-255 (higher = closer), default 200
     * @param distanceThreshold For distance sensors: max mm to consider "near wall", default 300
     * @return true if any coordinate was reset
     */
    bool autoReset(int proximityThreshold = 200, int distanceThreshold = 300);

    /**
     * @brief Reset X coordinate when aligned with a specific heading
     * Takes into account the robot's heading to calculate correct position
     * @param wallX The X coordinate of the wall (-72 or +72)
     * @return true if reset was successful
     */
    bool resetXWithHeading(float wallX);

    /**
     * @brief Reset Y coordinate when aligned with a specific heading
     * Takes into account the robot's heading to calculate correct position
     * @param wallY The Y coordinate of the wall (-72 or +72)
     * @return true if reset was successful
     */
    bool resetYWithHeading(float wallY);

    /**
     * @brief Get distance reading from front sensor in inches
     * @return Distance in inches, or -1 if sensor not available/invalid
     */
    float getFrontDistance();

    /**
     * @brief Get distance reading from back sensor in inches
     * @return Distance in inches, or -1 if sensor not available/invalid
     */
    float getBackDistance();

    /**
     * @brief Get distance reading from left sensor in inches
     * @return Distance in inches, or -1 if sensor not available/invalid
     */
    float getLeftDistance();

    /**
     * @brief Get distance reading from right sensor in inches
     * @return Distance in inches, or -1 if sensor not available/invalid
     */
    float getRightDistance();

private:
    lemlib::Chassis& m_chassis;
    
    // Distance sensors (preferred)
    pros::Distance* m_frontDist;
    pros::Distance* m_backDist;
    pros::Distance* m_leftDist;
    pros::Distance* m_rightDist;
    
    // Optical sensors (backup option)
    pros::Optical* m_frontOpt;
    pros::Optical* m_backOpt;
    pros::Optical* m_leftOpt;
    pros::Optical* m_rightOpt;
    
    bool m_useDistanceSensors;

    /**
     * @brief Convert mm to inches
     */
    static float mmToInches(float mm) { return mm / 25.4f; }

    /**
     * @brief Normalize angle to 0-360 range
     */
    static float normalizeAngle(float angle);

    /**
     * @brief Convert optical proximity (0-255) to approximate distance in inches
     * Note: This is a rough estimate, optical sensors aren't designed for distance measurement
     * @param proximity Proximity value 0-255 (255 = closest)
     * @return Approximate distance in inches (very rough estimate)
     */
    static float proximityToInches(int proximity);
};