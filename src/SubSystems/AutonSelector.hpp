#pragma once

#include "liblvgl/lvgl.h"

/**
 * @brief Available autonomous routes
 */
enum class AutonRoute {
    SKILLS,
    RIGHT_SIDE,
    LEFT_SIDE,
    SOLO_AWP,
    DO_NOTHING
};

/**
 * @brief Alliance color for route variations
 */
enum class Alliance {
    RED,
    BLUE
};

/**
 * @brief Visual autonomous route selector using LVGL
 * 
 * Creates a touchscreen UI on the V5 brain for selecting
 * which autonomous routine to run before a match.
 * 
 * Usage:
 *   - Call init() in initialize() or competition_initialize()
 *   - Call getSelectedRoute() in autonomous() to run selection
 *   - Call destroy() when entering opcontrol (optional)
 */
class AutonSelector {
public:
    /**
     * @brief Initialize the selector UI
     * Creates LVGL widgets and displays the selector screen
     */
    static void init();
    
    /**
     * @brief Destroy the selector UI and clean up
     */
    static void destroy();
    
    /**
     * @brief Get the currently selected autonomous route
     */
    static AutonRoute getSelectedRoute();
    
    /**
     * @brief Get the selected alliance color
     */
    static Alliance getAlliance();
    
    /**
     * @brief Get the name of a route as a string
     */
    static const char* getRouteName(AutonRoute route);

private:
    static AutonRoute selectedRoute;
    static Alliance selectedAlliance;
    
    // LVGL objects
    static lv_obj_t* screen;
    static lv_obj_t* titleLabel;
    static lv_obj_t* selectionLabel;
    static lv_obj_t* allianceSwitch;
    static lv_obj_t* routeButtons[5];
    
    // Button styles
    static lv_style_t styleDefault;
    static lv_style_t styleSelected;
    static lv_style_t styleRed;
    static lv_style_t styleBlue;
    
    static void createStyles();
    static void createUI();
    static void updateSelection();
    
    // Event callbacks
    static void onRouteButtonClick(lv_event_t* e);
    static void onAllianceToggle(lv_event_t* e);
};
