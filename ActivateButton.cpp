
#include "Arduino.h"
#include "ActivateButton.h"

const unsigned long long_press_threshold_ms = 1000;
const unsigned long debounce_threshold_ms = 10;

ActivateButton::ActivateButton(uint32_t _pin)
{
    last_pressed_time_ms = 0UL;
    last_released_time_ms = 0UL;
    curr_state = false;
    prev_state = false;
    curr_state_unfiltered = false;
    prev_state_unfiltered = false;
    pin = _pin;
    mouse_move_activated = true;
    mouse_pressed = false;
    prev_mouse_pressed = false;
    long_press_flag = false;
}

void ActivateButton::setup()
{
    pinMode(pin, INPUT_PULLUP);
    curr_state_unfiltered = digitalRead(pin);
    prev_state_unfiltered = curr_state_unfiltered;
    curr_state = curr_state_unfiltered;
    prev_state = curr_state_unfiltered;
}

void ActivateButton::mainLoopService()
{
    prev_state_unfiltered = curr_state_unfiltered;
    curr_state_unfiltered = !digitalRead(pin);
    unsigned long current_time_ms = millis();

    if (prev_state_unfiltered != curr_state_unfiltered)
    {
        last_toggle_time_ms = current_time_ms;
    }

    if (current_time_ms - last_toggle_time_ms > debounce_threshold_ms)
    {
        // Debounce filter passed
        prev_state = curr_state;
        curr_state = curr_state_unfiltered;

        // Pressed transition
        if (curr_state && !prev_state)
        {
            long_press_flag = false;
            last_pressed_time_ms = current_time_ms;
        }

        // Released transition
        if (!curr_state && prev_state)
        {
            long_press_flag = false;
            last_released_time_ms = current_time_ms;
        }

        prev_mouse_pressed = mouse_pressed;
        if (!long_press_flag && mouse_move_activated)
        {
            mouse_pressed = curr_state;
        }
        else
        {
            mouse_pressed = false;
        }

        // Check for long press
        if (curr_state && !long_press_flag && current_time_ms - last_pressed_time_ms > long_press_threshold_ms)
        {
            long_press_flag = true;

            mouse_move_activated = !mouse_move_activated;
        }
    }
}