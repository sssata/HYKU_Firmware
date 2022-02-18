#pragma once


typedef enum
{
    KEY_TYPE_MOUSE = 0,
    KEY_TYPE_KEYBOARD = 1,
} KeyType_t;


typedef struct 
{
    int long_press_time_ms;
    KeyType_t key_type; 
    char key_keyboard;
    int key_mouse;
} ButtonSettings_t;

class ActivateButton
{
public:
    unsigned long last_pressed_time_ms;
    unsigned long last_released_time_ms;
    const unsigned long long_press_threshold_ms = 1000;
    const unsigned long debounce_threshold_ms = 10;
    unsigned long last_toggle_time_ms;
    bool curr_state_unfiltered;
    bool prev_state_unfiltered;
    bool curr_state;
    bool prev_state;
    uint32_t pin;
    bool mouse_move_activated;
    bool mouse_pressed;
    bool prev_mouse_pressed;
    bool long_press_flag;

    ActivateButton(uint32_t _pin);

    void setup();

    void mainLoopService();
};