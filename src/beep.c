#include "driver_buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_psu.h"
#include "math.h"

// Define the beat and a short gap between notes (in ms)
#define QUARTER_NOTE_MS 135
#define GAP_MS 1

void ascent_beep(void) {
    note(NOTE_C, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_E, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_G, OCTAVE_4, 75);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_C, OCTAVE_5, 75);
    vTaskDelay(pdMS_TO_TICKS(500));
    note(NOTE_G, OCTAVE_5, 750);
}
void break_beep(void) {
    note(NOTE_C, OCTAVE_5, 1000);
    vTaskDelay(pdMS_TO_TICKS(25));
}

void digit_beep(void) {
    note(NOTE_C, OCTAVE_4, 500);
    vTaskDelay(pdMS_TO_TICKS(25));
}
void high_beep(void) {
    note(NOTE_G, OCTAVE_5, 100);
    vTaskDelay(pdMS_TO_TICKS(25));
}

void low_beep(void) {
    note(NOTE_C, OCTAVE_4, 100);
    vTaskDelay(pdMS_TO_TICKS(25));

}
void battery_beep(void) {
    float voltage = psu_read_battery_voltage();
    
    // Convert to integer and decimal parts (multiply by 100 to get 2 decimal places)
    int voltage_int = (int)(voltage * 100);
    
    // Extract individual digits
    int tens = (voltage_int / 1000) % 10;     // First digit (tens place)
    int ones = (voltage_int / 100) % 10;      // Second digit (ones place)
    int tenths = (voltage_int / 10) % 10;     // First decimal place
    int hundredths = voltage_int % 10;        // Second decimal place
    
    // Play beeps for tens place (if any)
    if (tens > 0) {
        for (int i = 0; i < tens; i++) {
            high_beep();
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        // Pause between tens and ones
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Play beeps for ones place
    for (int i = 0; i < ones; i++) {
        high_beep();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    // Short pause to indicate decimal point
    vTaskDelay(pdMS_TO_TICKS(500));
    digit_beep();
    vTaskDelay(pdMS_TO_TICKS(500));
    // For tenths place (0-9)
    for (int i = 0; i < tenths; i++) {
        high_beep();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    // Short pause before hundredths
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // For hundredths place (0-9)
    for (int i = 0; i < hundredths; i++) {
        high_beep();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    // Final pause
    vTaskDelay(pdMS_TO_TICKS(500));
}
void error_beep(void) {
    note(NOTE_D, OCTAVE_6, 100);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_B, OCTAVE_5, 100);
    vTaskDelay(pdMS_TO_TICKS(25));
    note(NOTE_GS, OCTAVE_5, 100);
    vTaskDelay(pdMS_TO_TICKS(25));
}

void wait_beep(void) {
    note(NOTE_C, OCTAVE_5, 100);
    vTaskDelay(pdMS_TO_TICKS(75));
    note(NOTE_G, OCTAVE_4, 100);
    vTaskDelay(pdMS_TO_TICKS(500));
    
}

// Helper function to play a note for a given duration factor (duration = factor * quarter note).
static void play_tone(note_t n, octave_t o, float factor) {
    // Calculate the note duration in milliseconds
    uint32_t duration = (uint32_t)(QUARTER_NOTE_MS * factor);
    // Play the note (this function is assumed to drive the buzzer for "duration" ms)
    note(n, o, duration);
    // Short pause between notes
    vTaskDelay(pdMS_TO_TICKS(GAP_MS));
}

// Helper function for a rest (silence) lasting "factor" beats.
static void rest_tone(float factor) {
    uint32_t duration = (uint32_t)(QUARTER_NOTE_MS * factor);
    vTaskDelay(pdMS_TO_TICKS(duration + GAP_MS));
}

void megolavania(void) {
    buzzer_init();
    
    // *** Repeat Section: Bars 1-2 [repeat twice] *** 
    // Bar 1: "D D"
    for (int i = 0; i < 2; i++) {  // Repeat the two-bar phrase
        // Bar 1
        play_tone(NOTE_D, OCTAVE_4, 1.0);  // "D"
        play_tone(NOTE_D, OCTAVE_4, 1.0);  // "D"

        // Bar 2: "d z A z"
        play_tone(NOTE_D, OCTAVE_5, 1.0);  // "d" (one octave up)
        rest_tone(1.0);                    // "z" (rest for one beat)
        play_tone(NOTE_A, OCTAVE_4, 1.0);    // "A"
        rest_tone(1.0);                    // "z"
    }

    // *** Bar 3: "^G z =G z F3/2 D F G   C C" ***
    play_tone(NOTE_GS, OCTAVE_4, 1.0); // "^G" (G sharp)
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "=G" (G natural)
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_F, OCTAVE_4, 1.5);        // "F3/2" (1.5 beats)
    play_tone(NOTE_D, OCTAVE_4, 1.0);        // "D"
    play_tone(NOTE_F, OCTAVE_4, 1.0);        // "F"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "G"
    play_tone(NOTE_C, OCTAVE_4, 1.0);        // "C"
    play_tone(NOTE_C, OCTAVE_4, 1.0);        // "C"

    // *** Bar 4: "d z A z" ***
    play_tone(NOTE_D, OCTAVE_5, 1.0);        // "d"
    rest_tone(1.0);                        // "z"
    play_tone(NOTE_A, OCTAVE_4, 1.0);        // "A"
    rest_tone(1.0);                        // "z"

    // *** Bar 5: "^G z =G z F3/2 D F G   B, B," ***
    play_tone(NOTE_GS, OCTAVE_4, 1.0); // "^G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "=G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_F, OCTAVE_4, 1.5);        // "F3/2"
    play_tone(NOTE_D, OCTAVE_4, 1.0);        // "D"
    play_tone(NOTE_F, OCTAVE_4, 1.0);        // "F"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "G"
    play_tone(NOTE_B, OCTAVE_3, 1.0);        // "B," (B one octave lower)
    play_tone(NOTE_B, OCTAVE_3, 1.0);        // "B,"

    // *** Bar 6: "d z A z" ***
    play_tone(NOTE_D, OCTAVE_5, 1.0);        // "d"
    rest_tone(1.0);                        // "z"
    play_tone(NOTE_A, OCTAVE_4, 1.0);        // "A"
    rest_tone(1.0);                        // "z"

    // *** Bar 7: "^G z =G z F3/2 D F G   ^A, ^A," ***
    play_tone(NOTE_GS, OCTAVE_4, 1.0); // "^G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "=G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_F, OCTAVE_4, 1.5);        // "F3/2"
    play_tone(NOTE_D, OCTAVE_4, 1.0);        // "D"
    play_tone(NOTE_F, OCTAVE_4, 1.0);        // "F"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "G"
    play_tone(NOTE_AS, OCTAVE_3, 1.0);  // "^A," (A sharp, one octave lower)
    play_tone(NOTE_AS, OCTAVE_3, 1.0);  // "^A,"

    // *** Bar 8: "d z A z" ***
    play_tone(NOTE_D, OCTAVE_5, 1.0);        // "d"
    rest_tone(1.0);                        // "z"
    play_tone(NOTE_A, OCTAVE_4, 1.0);        // "A"
    rest_tone(1.0);                        // "z"

    // *** Bar 9: "^G z =G z F3/2 D F G" ***
    play_tone(NOTE_GS, OCTAVE_4, 1.0); // "^G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "=G"
    rest_tone(1.0);                       // "z"
    play_tone(NOTE_F, OCTAVE_4, 1.5);        // "F3/2"
    play_tone(NOTE_D, OCTAVE_4, 1.0);        // "D"
    play_tone(NOTE_F, OCTAVE_4, 1.0);        // "F"
    play_tone(NOTE_G, OCTAVE_4, 1.0);        // "G"
    
    // (Optionally add an ending rest)
    rest_tone(1.0);
}

void megolavania_task(void)
{
    while (1) {
        megolavania();
    }
}