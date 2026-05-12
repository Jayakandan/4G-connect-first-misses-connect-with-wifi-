/*
 * Test sketch to play low_battery.wav every 5 seconds
 * Uses the same speaker configuration as main.cpp
 */

#include <Arduino.h>
#include <Audio.h>
#include <SPI.h>
#include <SD.h>

// Audio configuration - same as main.cpp
AudioPlaySdWav playSdWav1;   // Function to play SD card file
AudioMixer4 mixer1;          // Mixer for left channel
AudioMixer4 mixer2;          // Mixer for right channel
AudioOutputI2S2 i2s2_1;      // I2S audio output

// Audio connections - same as main.cpp
AudioConnection patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection patchCord2(playSdWav1, 1, mixer2, 0);
AudioConnection patchCord3(mixer1, 0, i2s2_1, 0);
AudioConnection patchCord4(mixer2, 0, i2s2_1, 1);

// Timing variables
unsigned long lastPlayTime = 0;
const unsigned long playInterval = 5000; // 5 seconds

// Audio file to play
const char* audioFile = "low_battery.wav";

void setup() {
  // Configure I2S audio pins - critical for Teensy I2S audio output
  CORE_PIN2_CONFIG  = 0;
  CORE_PIN3_CONFIG  = 0;
  CORE_PIN4_CONFIG  = 0;
  CORE_PIN5_CONFIG  = 0;
  CORE_PIN33_CONFIG = 0;

  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing SD card...");

  // Initialize SD card using built-in SD card slot
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("SD card initialized successfully");

  // Initialize Audio Library
  AudioMemory(40);
  delay(200);

  // Set mixer gain - same as main.cpp
  mixer1.gain(0, 4);
  mixer2.gain(0, 4);

  Serial.println("Audio system initialized");
  Serial.println("Starting playback loop...");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();

  // Play audio every 5 seconds
  if (currentTime - lastPlayTime >= playInterval) {
    lastPlayTime = currentTime;

    // Configure I2S pins before playback
    CORE_PIN2_CONFIG  = 2;
    CORE_PIN3_CONFIG  = 2;
    CORE_PIN4_CONFIG  = 2;
    CORE_PIN5_CONFIG  = 2;
    CORE_PIN33_CONFIG = 2;

    Serial.print("Playing file: ");
    Serial.println(audioFile);

    // Start playback
    if (!playSdWav1.play(audioFile)) {
      Serial.println("Error: Failed to start playback.");
    } else {
      delay(5);

      // Wait until the audio finishes playing
      while (playSdWav1.isPlaying()) {
        // Just wait
      }

      delay(500);
      Serial.println("Done playing file");
      Serial.println();
    }

    // Reset I2S pins after playback
    CORE_PIN2_CONFIG  = 0;
    CORE_PIN3_CONFIG  = 0;
    CORE_PIN4_CONFIG  = 0;
    CORE_PIN5_CONFIG  = 0;
    CORE_PIN33_CONFIG = 0;
  }
}
