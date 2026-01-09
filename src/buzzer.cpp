#include <Arduino.h>
#include "buzzer.h"

void buzzer_init() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void play_tune() {
  // Startup jingle melody
  tone(BUZZER_PIN, 523.25); // C5
  delay(200);
  tone(BUZZER_PIN, 659.25); // E5
  delay(200);
  tone(BUZZER_PIN, 783.99); // G5
  delay(200);
  tone(BUZZER_PIN, 1046.50); // C6
  delay(400);
  noTone(BUZZER_PIN);
  delay(100);
  
  tone(BUZZER_PIN, 783.99); // G5
  delay(200);
  tone(BUZZER_PIN, 1046.50); // C6
  delay(600);
  noTone(BUZZER_PIN);
}
