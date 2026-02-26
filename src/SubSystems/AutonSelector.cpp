#include "AutonSelector.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

// Global chassis access
extern lemlib::Chassis chassis;

// Static member definitions
AutonRoute AutonSelector::selectedRoute = AutonRoute::SKILLS;
Alliance AutonSelector::selectedAlliance = Alliance::RED;

lv_obj_t* AutonSelector::screen = nullptr;
lv_obj_t* AutonSelector::titleLabel = nullptr;
lv_obj_t* AutonSelector::selectionLabel = nullptr;
lv_obj_t* AutonSelector::allianceSwitch = nullptr;
lv_obj_t* AutonSelector::routeButtons[5] = {nullptr};

lv_style_t AutonSelector::styleDefault;
lv_style_t AutonSelector::styleSelected;
lv_style_t AutonSelector::styleRed;
lv_style_t AutonSelector::styleBlue;

const char* AutonSelector::getRouteName(AutonRoute route) {
    switch (route) {
        case AutonRoute::SKILLS:     return "SKILLS";
        case AutonRoute::RIGHT_SIDE: return "RIGHT SIDE";
        case AutonRoute::LEFT_SIDE:  return "LEFT SIDE";
        case AutonRoute::SOLO_AWP:   return "SOLO AWP";
        case AutonRoute::DO_NOTHING: return "DO NOTHING";
        default:                     return "UNKNOWN";
    }
}

void AutonSelector::createStyles() {
    // Default button style - dark gray
    lv_style_init(&styleDefault);
    lv_style_set_bg_color(&styleDefault, lv_color_hex(0x333333));
    lv_style_set_bg_opa(&styleDefault, LV_OPA_COVER);
    lv_style_set_border_width(&styleDefault, 2);
    lv_style_set_border_color(&styleDefault, lv_color_hex(0x555555));
    lv_style_set_radius(&styleDefault, 8);
    lv_style_set_text_color(&styleDefault, lv_color_white());
    lv_style_set_pad_all(&styleDefault, 10);
    
    // Selected button style - bright cyan
    lv_style_init(&styleSelected);
    lv_style_set_bg_color(&styleSelected, lv_color_hex(0x00AAFF));
    lv_style_set_bg_opa(&styleSelected, LV_OPA_COVER);
    lv_style_set_border_width(&styleSelected, 3);
    lv_style_set_border_color(&styleSelected, lv_color_white());
    lv_style_set_radius(&styleSelected, 8);
    lv_style_set_text_color(&styleSelected, lv_color_white());
    lv_style_set_shadow_width(&styleSelected, 15);
    lv_style_set_shadow_color(&styleSelected, lv_color_hex(0x00AAFF));
    
    // Red alliance style
    lv_style_init(&styleRed);
    lv_style_set_bg_color(&styleRed, lv_color_hex(0xCC2222));
    lv_style_set_bg_opa(&styleRed, LV_OPA_COVER);
    
    // Blue alliance style  
    lv_style_init(&styleBlue);
    lv_style_set_bg_color(&styleBlue, lv_color_hex(0x2244CC));
    lv_style_set_bg_opa(&styleBlue, LV_OPA_COVER);
}

void AutonSelector::createUI() {
    // Create main screen with dark background
    screen = lv_obj_create(lv_scr_act());
    lv_obj_set_size(screen, 480, 240);
    lv_obj_set_pos(screen, 0, 0);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x1a1a2e), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_set_style_pad_all(screen, 0, 0);
    
    // Title label
    titleLabel = lv_label_create(screen);
    lv_label_set_text(titleLabel, "AUTONOMOUS SELECTOR");
    lv_obj_set_style_text_color(titleLabel, lv_color_hex(0x00AAFF), 0);
    lv_obj_set_style_text_font(titleLabel, &lv_font_montserrat_20, 0);
    lv_obj_align(titleLabel, LV_ALIGN_TOP_MID, 0, 8);
    
    // Route button definitions
    struct ButtonDef {
        const char* label;
        AutonRoute route;
        int x, y;
    };
    
    ButtonDef buttons[] = {
        {"SKILLS",     AutonRoute::SKILLS,     20,  45},
        {"RIGHT",      AutonRoute::RIGHT_SIDE, 130, 45},
        {"LEFT",       AutonRoute::LEFT_SIDE,  240, 45},
        {"SOLO AWP",   AutonRoute::SOLO_AWP,   350, 45},
        {"DO NOTHING", AutonRoute::DO_NOTHING, 175, 110}
    };
    
    // Create route buttons
    for (int i = 0; i < 5; i++) {
        routeButtons[i] = lv_btn_create(screen);
        lv_obj_set_size(routeButtons[i], 100, 55);
        lv_obj_set_pos(routeButtons[i], buttons[i].x, buttons[i].y);
        lv_obj_add_style(routeButtons[i], &styleDefault, 0);
        
        // Store route enum in user data
        lv_obj_set_user_data(routeButtons[i], (void*)(intptr_t)buttons[i].route);
        lv_obj_add_event_cb(routeButtons[i], onRouteButtonClick, LV_EVENT_CLICKED, nullptr);
        
        // Button label
        lv_obj_t* label = lv_label_create(routeButtons[i]);
        lv_label_set_text(label, buttons[i].label);
        lv_obj_center(label);
    }
    
    // Make "DO NOTHING" button wider
    lv_obj_set_size(routeButtons[4], 130, 55);
    
    // Alliance toggle section
    lv_obj_t* allianceLabel = lv_label_create(screen);
    lv_label_set_text(allianceLabel, "Alliance:");
    lv_obj_set_style_text_color(allianceLabel, lv_color_white(), 0);
    lv_obj_set_pos(allianceLabel, 20, 185);
    
    // Alliance switch (RED off, BLUE on)
    allianceSwitch = lv_switch_create(screen);
    lv_obj_set_size(allianceSwitch, 60, 30);
    lv_obj_set_pos(allianceSwitch, 95, 180);
    lv_obj_set_style_bg_color(allianceSwitch, lv_color_hex(0xCC2222), 0); // Red when off
    lv_obj_set_style_bg_color(allianceSwitch, lv_color_hex(0x2244CC), LV_PART_INDICATOR | LV_STATE_CHECKED); // Blue when on
    lv_obj_add_event_cb(allianceSwitch, onAllianceToggle, LV_EVENT_VALUE_CHANGED, nullptr);
    
    // Red/Blue labels next to switch
    lv_obj_t* redLabel = lv_label_create(screen);
    lv_label_set_text(redLabel, "RED");
    lv_obj_set_style_text_color(redLabel, lv_color_hex(0xFF4444), 0);
    lv_obj_set_pos(redLabel, 165, 185);
    
    lv_obj_t* blueLabel = lv_label_create(screen);
    lv_label_set_text(blueLabel, "BLUE");
    lv_obj_set_style_text_color(blueLabel, lv_color_hex(0x4488FF), 0);
    lv_obj_set_pos(blueLabel, 200, 185);
    
    // Selection label (shows current choice)
    selectionLabel = lv_label_create(screen);
    lv_obj_set_style_text_color(selectionLabel, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(selectionLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_pos(selectionLabel, 280, 185);
    
    // Set initial selection
    updateSelection();
}

void AutonSelector::updateSelection() {
    // Update selection label
    char buf[64];
    snprintf(buf, sizeof(buf), "Selected: %s", getRouteName(selectedRoute));
    lv_label_set_text(selectionLabel, buf);
    
    // Update button styles
    for (int i = 0; i < 5; i++) {
        AutonRoute btnRoute = (AutonRoute)(intptr_t)lv_obj_get_user_data(routeButtons[i]);
        
        // Remove all custom styles first
        lv_obj_remove_style(routeButtons[i], &styleSelected, 0);
        lv_obj_remove_style(routeButtons[i], &styleDefault, 0);
        
        if (btnRoute == selectedRoute) {
            lv_obj_add_style(routeButtons[i], &styleSelected, 0);
        } else {
            lv_obj_add_style(routeButtons[i], &styleDefault, 0);
        }
    }
}

void AutonSelector::onRouteButtonClick(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    AutonRoute route = (AutonRoute)(intptr_t)lv_obj_get_user_data(btn);
    
    // Only calibrate if selection changes or user re-selects same route (confirmation)
    // We update selection first so UI shows the click
    selectedRoute = route;
    updateSelection();
    
    // Force redraw so user sees the button highlight before calibration freezes
    lv_refr_now(NULL);
    
    // Wait 1 second as requested
    pros::delay(1000);
    
    // Calibrate sensors
    chassis.calibrate();
}

void AutonSelector::onAllianceToggle(lv_event_t* e) {
    bool isBlue = lv_obj_has_state(allianceSwitch, LV_STATE_CHECKED);
    selectedAlliance = isBlue ? Alliance::BLUE : Alliance::RED;
}

void AutonSelector::init() {
    createStyles();
    createUI();
}

void AutonSelector::destroy() {
    if (screen != nullptr) {
        lv_obj_del(screen);
        screen = nullptr;
    }
}

AutonRoute AutonSelector::getSelectedRoute() {
    return selectedRoute;
}

Alliance AutonSelector::getAlliance() {
    return selectedAlliance;
}
