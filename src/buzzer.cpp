#include <Arduino.h>
#include "buzzer.h"

void buzzer_init() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void play_tune() {
  tone(BUZZER_PIN, 261.63); // Play 261.63 Hz tone (C)
  delay(500);  
  tone(BUZZER_PIN, 293.66); // Play 293.66 Hz tone (D)
  delay(500);
  tone(BUZZER_PIN, 329.63); // Play 329.63 Hz tone (E)
  delay(500);
  tone(BUZZER_PIN, 349.23); // Play 349.23 Hz tone (F)
  delay(500);   
  tone(BUZZER_PIN, 392.00); // Play 392.00 Hz tone (G)
  delay(500); 
  tone(BUZZER_PIN, 440);    // Play 440 Hz tone (A)
  delay(500); 
  tone(BUZZER_PIN, 493.88); // Play 493.88 Hz tone (B)
  delay(500);    
  tone(BUZZER_PIN, 523.25); // Play 523.25 Hz tone (C)
  delay(500);
  noTone(BUZZER_PIN);        // Stop tone
  delay(500);                // Wait 0.5 seconds
}
