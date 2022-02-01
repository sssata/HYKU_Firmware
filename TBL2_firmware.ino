#pragma once
#include <AS5600.h>
#include <Wire.h>
#include <AbsMouse.h>
#include "Custom_AS5600.h"

#define FLASH_DEBUG 0
#include <FlashStorage_SAMD.h>

//#define DEBUG_LOGGING

#ifdef DEBUG_LOGGING
#define debugPrint(...) Serial.print(__VA_ARGS__)
#else
#define debugPrint(...)
#endif

#define VERSION 2



// FAST TRIG STUFF START


#define INT16_BITS  (8 * sizeof(int16_t))
#ifndef INT16_MAX
#define INT16_MAX   ((1<<(INT16_BITS-1))-1)
#endif

/*
 * "5 bit" large table = 32 values. The mask: all bit belonging to the table
 * are 1, the all above 0.
 */
#define TABLE_BITS  (5)
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)
 
/*
 * The lookup table is to 90DEG, the input can be -360 to 360 DEG, where negative
 * values are transformed to positive before further processing. We need two
 * additional bits (*4) to represent 360 DEG:
 */
#define LOOKUP_BITS (TABLE_BITS+2)
#define LOOKUP_MASK ((1<<LOOKUP_BITS)-1)
#define FLIP_BIT    (1<<TABLE_BITS)
#define NEGATE_BIT  (1<<(TABLE_BITS+1))
#define INTERP_BITS (INT16_BITS-1-LOOKUP_BITS)
#define INTERP_MASK ((1<<INTERP_BITS)-1)
 
/**
 * "5 bit" lookup table for the offsets. These are the sines for exactly
 * at 0deg, 11.25deg, 22.5deg etc. The values are from -1 to 1 in Q15.
 */
int16_t sin90[TABLE_SIZE+1] = {
  0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
  0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
  0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
  0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
  0x7fff
};
 




// FAST TRIG STUFF END



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

    ActivateButton(uint32_t _pin)
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

    void setup()
    {
        pinMode(pin, INPUT_PULLUP);
        curr_state_unfiltered = digitalRead(pin);
        prev_state_unfiltered = curr_state_unfiltered;
        curr_state = curr_state_unfiltered;
        prev_state = curr_state_unfiltered;
    }

    void mainLoopService()
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
};

typedef struct
{
    /* CARTESIAN PLANE REPRESENTING SCREEN WITH ORIGIN AT TOP LEFT
   *        (0,0) -> +-----------+
   *                 |           |
   *                 | +---------| <- y_origin
   *                 | |         | 
   *                 | |         | y_size
   *                 +-+---------+
   *           x_origin^   x_size
   */
    int x_size;
    int y_size;
    int x_origin;
    int y_origin;
    int x_max_size; // Full size of screen
    int y_max_size; // Full size of screen
} ScreenArea_t;

typedef struct
{
    /* CARTESIAN PLANE WITH ORIGIN AT SHOULDER JOINT
   *   ____
   *       \         +-----------+
   *(0,0)>+ |        |           |
   *   ____/         |           |
   *                 |           | y_size
   *                 |           |
   *   (x_origin, -> +-----------+
   *    y_origin)        x_size
   */
    float x_origin;
    float y_origin;
    float x_size;
    float y_size;
} TabletArea_t;

typedef struct
{
    float exp_filter_time_constant_ms;
    int exp_filter_activate;
} FilterParams_t;

typedef struct
{
    int A_0;
    int A_90;
    int B_0;
    int B_90;
    int B_180;
    FilterParams_t filter_params;
    uint64_t first_write_timestamp;
    uint64_t last_write_timestamp;
    TabletArea_t tablet_area;
    ScreenArea_t screen_size;
} StorageVars_t;

typedef struct
{
    float len1;
    float len2;
    float x_raw;
    float y_raw;
    float x_raw_prev;
    float y_raw_prev;
} SensorVars_t;

StorageVars_t storage_vars;

const int WRITTEN_SIGNATURE = 0xBEEFDEED;
uint16_t storedAddress = 0;

const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

CustomAS5600 as5600Sensor;

int POTA_PIN = 1;
int POTB_PIN = 2;
int ACTIVATE_PIN = 7;
int SELECT_PIN = 3;
int G_PIN = 10;
int R_PIN = 9;
int B_PIN = 8;

float rValue = 0;
float gValue = 0;
float bValue = 0;

float valueA = 0;
float valueB = 0;

float prevValueA = -1;
float prevValueB = -1;
float prevPrevValueA = -1;
float prevPrevValueB = -1;

float expFilterWeight = 1;
float expFilterWeightXY = 1;

float angleA = 0;
float angleB = 0;

int rawAngleA = 0;
int rawAngleB = 0;

float l1 = 60;
float l2 = 60;

float x_raw = 0;
float y_raw = 0;
float x_rawPrev = 0;
float y_rawPrev = 0;

int A_0 = 3954;
int A_90 = 3954 - (4096 / 4);
int B_90 = 3872 - (4096 / 4);
int B_180 = 3872;

int pollRate_hz = 1000;
int period_uS = (1000 * 1000) / pollRate_hz;
unsigned long lastRunTime_uS = 0;
unsigned long currentTime_uS = 0;

float curr_filter_time_uS;
float last_filter_time_uS;

int screenWidth = 1920;
int screenHeight = 1080;

float x_origin = 40;
float x_width = 67.7;
float x_max = x_origin + x_width;
float y_height = screenHeight * 1.0 / screenWidth * x_width;
float y_origin = y_height / 2;
float y_max = y_origin - y_height;

int l_count = 0;

ActivateButton activateButton = ActivateButton(ACTIVATE_PIN);

void setup()
{
    // put your setup code here, to run once:
    pinMode(G_PIN, OUTPUT);
    pinMode(R_PIN, OUTPUT);
    pinMode(B_PIN, OUTPUT);
    activateButton.setup();
    pinMode(SELECT_PIN, OUTPUT);
    analogWriteResolution(12);
    Serial.begin(115200);

    loadFlashStorage(&storage_vars);

    Wire.begin();
    Wire.setClock(800000UL);
    Wire.setTimeout(100);

    analogReadResolution(12);
    analogReference(AR_DEFAULT);

    AbsMouse.init(storage_vars.screen_size.x_max_size, storage_vars.screen_size.y_max_size, false);

    // Prepare Sensor A (shoulder sensor)
    digitalWrite(SELECT_PIN, HIGH);
    delayMicroseconds(50);
    uint8_t err;
    err = as5600Sensor.as5600Setup();
    if (err != 0)
    {
        debugPrint("SENSOR A ERROR CODE: ");
        debugPrint(String(err, DEC));
        debugPrint("\n");
    }

    prevPrevValueA = as5600Sensor.getRawAngleFast();
    prevValueA = prevPrevValueA;
    valueA = prevValueA;

    // Prepare Sensor B (elbow sensor)
    digitalWrite(SELECT_PIN, LOW);
    delayMicroseconds(50);
    as5600Sensor.as5600Setup();

    if (err != 0)
    {
        debugPrint("SENSOR B ERROR CODE: ");
        debugPrint(String(err, DEC));
        debugPrint("\n");
    }

    prevPrevValueB = as5600Sensor.getRawAngleFast();
    prevValueB = prevPrevValueB;
    valueB = prevValueB;

    // Reset filter
    curr_filter_time_uS = micros();
    last_filter_time_uS = micros();
}

void loop()
{

    currentTime_uS = micros();

    if (currentTime_uS - lastRunTime_uS > period_uS)
    {
        /*l_count ++;
        if (l_count % 1 == 0){
            Serial.printf("loop_period: %d\n", currentTime_uS - lastRunTime_uS);
        }*/
        lastRunTime_uS = currentTime_uS;

        //valueA = analogSample(POTA_PIN,ADCSamples);
        //valueB = analogSample(POTB_PIN,ADCSamples);

        prevPrevValueA = prevValueA;
        prevPrevValueB = prevValueB;

        prevValueA = valueA;
        prevValueB = valueB;

        

        // READ ANGLE A
        digitalWrite(SELECT_PIN, HIGH);
        delayMicroseconds(10);
        valueA = as5600Sensor.getRawAngleFast();
        //int valueA_median = median_of_3(valueA, prevPrevValueB, prevValueA);

        // READ ANGLE B
        digitalWrite(SELECT_PIN, LOW);
        delayMicroseconds(10);
        valueB = as5600Sensor.getRawAngleFast();
        //int valueB_median = median_of_3(valueB, prevPrevValueB, prevValueB);

        //Serial.println(valueA_median);

        debugPrint("Raw: " + String(valueA, 2) + "," + String(valueB, 2) + "\n");

        // Sensor ADC counts to angle in degrees
        angleA = mapf(valueA, storage_vars.A_0, storage_vars.A_90, 0.0, 90.0);
        angleB = mapf(valueB, storage_vars.B_0, storage_vars.B_90, 0.0, 90.0);

        float angleB2 = angleA + angleB;

        x_raw = l1 * cos(angleA * PI / 180) + l2 * cos(angleB2 * PI / 180);
        y_raw = l1 * sin(angleA * PI / 180) + l2 * sin(angleB2 * PI / 180);

        curr_filter_time_uS = micros();
        if (storage_vars.filter_params.exp_filter_activate != 0){
            x_raw = expFilter(x_raw, x_rawPrev, storage_vars.filter_params.exp_filter_time_constant_ms, (curr_filter_time_uS - last_filter_time_uS)/1000.0);
            y_raw = expFilter(y_raw, y_rawPrev, storage_vars.filter_params.exp_filter_time_constant_ms, (curr_filter_time_uS - last_filter_time_uS)/1000.0);
        }

        last_filter_time_uS = curr_filter_time_uS;

        x_rawPrev = x_raw;
        y_rawPrev = y_raw;

        float x_mapped = constrain(
            mapf(
                x_raw,
                storage_vars.tablet_area.x_origin,
                storage_vars.tablet_area.x_origin + storage_vars.tablet_area.x_size,
                storage_vars.screen_size.x_origin,
                storage_vars.screen_size.x_origin + storage_vars.screen_size.x_size),
            storage_vars.screen_size.x_origin,
            storage_vars.screen_size.x_origin + storage_vars.screen_size.x_size);

        float y_mapped = constrain(
            mapf(
                y_raw,
                storage_vars.tablet_area.y_origin + storage_vars.tablet_area.y_size, // flipped because screen y axis is inverted
                storage_vars.tablet_area.y_origin, 
                storage_vars.screen_size.y_origin,
                storage_vars.screen_size.y_origin + storage_vars.screen_size.y_size),
            storage_vars.screen_size.y_origin,
            storage_vars.screen_size.y_origin + storage_vars.screen_size.y_size);

        debugPrint(" A angle: " + String(angleA, DEC) + " B angle: " + String(angleB, DEC) + "\n");

        activateButton.mainLoopService();

        if (activateButton.mouse_pressed && !activateButton.prev_mouse_pressed)
        {
            debugPrint("mouse pressed\n");
            AbsMouse.press();
        }
        if (!activateButton.mouse_pressed && activateButton.prev_mouse_pressed)
        {
            debugPrint("mouse released\n");
            AbsMouse.release();
            AbsMouse.report();
        }

        if (activateButton.mouse_move_activated)
        {
            //debugPrint("mouse active\n");
            AbsMouse.move(round(x_mapped), round(y_mapped));
            AbsMouse.report();
        }
        else
        {
            //debugPrint("mouse not active\n");
        }

        ledService();

        recvWithStartEndMarkers();
        showNewData();
    }
}


float expFilter(float current, float prev, float time_constant, float loop_period)
{
    // Calculate factor from time constant
    float factor = exp(-loop_period/time_constant);
    return prev * (factor) + current * (1 - factor);
}

int median_of_3(int a, int b, int c)
{
    int the_max = max(max(a, b), c);
    int the_min = min(min(a, b), c);
    // unnecessarily clever code
    int the_median = the_max ^ the_min ^ a ^ b ^ c;
    return (the_median);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int sort_desc(const void *cmp1, const void *cmp2)
{
    // Need to cast the void * to int *
    int a = *((int *)cmp1);
    int b = *((int *)cmp2);
    // The comparison
    return a > b ? -1 : (a < b ? 1 : 0);
    // A simpler, probably faster way:
    //return b - a;
}

void ledService()
{

    float t_const = 0.0001;
    float a_const = 1500;

    unsigned long current_time = millis();

    //Mouse Button Pressed
    if (activateButton.mouse_pressed)
    {
        analogWrite(R_PIN, 1000);
        analogWrite(G_PIN, 100);
        analogWrite(B_PIN, 100);
        return;
    }

    // Active
    if (activateButton.mouse_move_activated)
    {
        analogWrite(R_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (0.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const / 2 + a_const / 1.8);
        analogWrite(G_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (1.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const / 2 + a_const / 1.8);
        analogWrite(B_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (2.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const / 2 + a_const / 1.8);
        return;
    }

    // Idle
    t_const = 0.0001;
    a_const = 0;
    //analogWrite(R_PIN, sinf(float(current_time * t_const)) * a_const / 2 + a_const / 2);
    analogWrite(R_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (0.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const * 0.0 + a_const / 1.8);

    a_const = 50;
    //analogWrite(G_PIN, sinf(float(current_time * t_const + (3 * PI / 3))) * a_const / 2 + a_const / 1.8);
    analogWrite(G_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (1.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const * 0.0 + a_const / 1.8);

    a_const = 300;
    //analogWrite(B_PIN, sinf(float(current_time * t_const + (4 * PI / 3))) * a_const / 2 + a_const / 1.8);
    analogWrite(B_PIN, sin1(  int16_t(  (fmodf(  (current_time * t_const + (2.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) * a_const * 0.0 + a_const / 1.8);
    //Serial.println(  sin1(  int16_t(  (fmodf(  (current_time * t_const + (2.0 / 3)  ),  2.0) - 1.0) * UINT16_MAX)) / float(INT16_MAX) );
    //Serial.println(sin1(fmodf((current_time * t_const + (2.0 / 3)), 1.0)*65535) * a_const / 2 + a_const / 1.8);

    return;
}

bool loadFlashStorage(StorageVars_t *storage_vars)
{

    int signature;
    EEPROM.get(storedAddress, signature);
    if (signature == WRITTEN_SIGNATURE)
    {
        // Flash has already been written
        // Read from flash and load to storage_vars

        StorageVars_t retrievedStorageVars;
        EEPROM.get(storedAddress + sizeof(signature), *storage_vars);
        //storage_vars = &retrievedStorageVars;
    }
    else
    {
        // Flash has not been written yet

        // Set Default Tablet Area
        TabletArea_t tabletArea;
        tabletArea.x_origin = 40;
        tabletArea.x_size = 67.7;
        tabletArea.y_origin = -20;
        tabletArea.y_size = 40;

        ScreenArea_t screenArea;
        screenArea.x_size = 1920;
        screenArea.y_size = 1080;
        screenArea.x_origin = 0;
        screenArea.y_origin = 0;
        screenArea.x_max_size = 1920;
        screenArea.y_max_size = 1080;

        FilterParams_t filter_params = {
            .exp_filter_time_constant_ms = 20,
            .exp_filter_activate = 0,
        };

        // Set Default Sensor Home
        storage_vars->A_0 = 0;
        storage_vars->A_90 = (int)(4096 / 4);
        storage_vars->B_0 = 0;
        storage_vars->B_90 = 0 - (4096 / 4);
        storage_vars->B_180 = 0;
        storage_vars->first_write_timestamp = 0;
        storage_vars->last_write_timestamp = 0;
        storage_vars->tablet_area = tabletArea;
        storage_vars->screen_size = screenArea;
        storage_vars->filter_params = filter_params;

        // Write to Flash
        saveFlashStorage(*storage_vars);

        return true;
    }

    return false;
}

bool saveFlashStorage(StorageVars_t storage_vars)
{

    // Write to Flash
    EEPROM.put(storedAddress, WRITTEN_SIGNATURE);
    EEPROM.put(storedAddress + sizeof(int), storage_vars);

    if (!EEPROM.getCommitASAP())
    {
        debugPrint("CommitASAP not set. Need commit()");
        EEPROM.commit();
    }

    return false;
}

bool printStorageVars(StorageVars_t *storage_vars)
{

    Serial.println("A_0: " + String(storage_vars->A_0, DEC));
    Serial.println("A_90: " + String(storage_vars->A_90, DEC));
    Serial.println("B_0: " + String(storage_vars->B_0, DEC));
    Serial.println("B_90: " + String(storage_vars->B_90, DEC));
    Serial.println("B_180: " + String(storage_vars->B_90, DEC));
    Serial.println("first_write_timestamp: " + String(int(storage_vars->first_write_timestamp), DEC));
    Serial.println("last_write_timestamp: " + String(int(storage_vars->last_write_timestamp), DEC));
    Serial.println("filter_params:");
    Serial.println("    exp_filter_time_constant_ms: " + String(storage_vars->filter_params.exp_filter_time_constant_ms));
    Serial.println("    exp_filter_activate: " + String(storage_vars->filter_params.exp_filter_activate));
    Serial.println("tablet_area: ");
    Serial.println("    x_origin: " + String(storage_vars->tablet_area.x_origin));
    Serial.println("    y_origin: " + String(storage_vars->tablet_area.y_origin));
    Serial.println("    x_size: " + String(storage_vars->tablet_area.x_size));
    Serial.println("    y_size: " + String(storage_vars->tablet_area.y_size));
    Serial.println("screen_size: ");
    Serial.println("    x_max_size: " + String(storage_vars->screen_size.x_max_size));
    Serial.println("    y_max_size: " + String(storage_vars->screen_size.y_max_size));
    Serial.println("    x_origin: " + String(storage_vars->screen_size.x_origin));
    Serial.println("    y_origin: " + String(storage_vars->screen_size.y_origin));
    Serial.println("    x_size: " + String(storage_vars->screen_size.x_size));
    Serial.println("    y_size: " + String(storage_vars->screen_size.y_size));
}

// Credit to Robin2 for recvWithStartEndMarkers and showNewData functions
// https://forum.arduino.cc/t/serial-input-basics-updated/382007
void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false)
    {
        rc = Serial.read();

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

void showNewData()
{
    if (newData == true)
    {
        Serial.println(receivedChars);
        if (strlen(receivedChars) >= 1)
        {

            char *startPointer;
            char *endPointer;
            switch (receivedChars[0])
            {
            case 'V':
                Serial.println(VERSION);
                break;

            case 'A':
                Serial.println("A: " + String(valueA, DEC));
                break;

            case 'B':
                Serial.println("B: " + String(valueB, DEC));
                break;

            case 'C': // Calibrate zero
                storage_vars.A_0 = valueA;
                storage_vars.A_90 = (storage_vars.A_0 - 1024) % 4096;
                storage_vars.B_0 = valueB;
                storage_vars.B_90 = (storage_vars.B_0 + 1024) % 4096;
                saveFlashStorage(storage_vars);
                printStorageVars(&storage_vars);
                break;

            case 'D': // Update tablet area

                startPointer = receivedChars + 1;
                storage_vars.tablet_area.x_origin = strtod(startPointer, &endPointer);

                startPointer = endPointer;
                storage_vars.tablet_area.y_origin = strtod(startPointer, &endPointer);

                startPointer = endPointer;
                storage_vars.tablet_area.x_size = strtod(startPointer, &endPointer);

                startPointer = endPointer;
                storage_vars.tablet_area.y_size = strtod(startPointer, &endPointer);

                saveFlashStorage(storage_vars);
                printStorageVars(&storage_vars);
                break;

            case 'E': // Update Screen Size

                startPointer = receivedChars + 1;
                storage_vars.screen_size.x_max_size = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.screen_size.y_max_size = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.screen_size.x_origin = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.screen_size.y_origin = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.screen_size.x_size = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.screen_size.y_size = strtol(startPointer, &endPointer, 10);

                saveFlashStorage(storage_vars);
                printStorageVars(&storage_vars);
                break;

            case 'F': // Update Filter Params

                startPointer = receivedChars + 1;
                storage_vars.filter_params.exp_filter_activate = strtol(startPointer, &endPointer, 10);

                startPointer = endPointer;
                storage_vars.filter_params.exp_filter_time_constant_ms = strtod(startPointer, &endPointer);

                saveFlashStorage(storage_vars);
                printStorageVars(&storage_vars);
                break;

            default:
                break;
            }
        }
        if (receivedChars)
            newData = false;
    }
}

int16_t sin1(int16_t angle)
{
  int16_t v0, v1;
  if(angle < 0) { angle += INT16_MAX; angle += 1; }
  v0 = (angle >> INTERP_BITS);
  if(v0 & FLIP_BIT) { v0 = ~v0; v1 = ~angle; } else { v1 = angle; }
  v0 &= TABLE_MASK;
  v1 = sin90[v0] + (int16_t) (((int32_t) (sin90[v0+1]-sin90[v0]) * (v1 & INTERP_MASK)) >> INTERP_BITS);
  if((angle >> INTERP_BITS) & NEGATE_BIT) v1 = -v1;
  return v1;
}
 
int16_t cos1(int16_t angle)
{
  if(angle < 0) { angle += INT16_MAX; angle += 1; }
  return sin1(angle - (int16_t)(((int32_t)INT16_MAX * 270) / 360));
}