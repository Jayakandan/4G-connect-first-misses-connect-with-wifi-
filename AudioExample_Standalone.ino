/*
 * Teensy 4.1 Audio Playback Example
 * Extracted from TeensyValidator Project
 *
 * Hardware Setup:
 * - Teensy 4.1 microcontroller
 * - I2S2 Audio Codec connected to:
 *   - Pin 20: LRCLK (Word Clock)
 *   - Pin 21: BCLK (Bit Clock)
 *   - Pin 7:  DIN (Data Out)
 *   - Pin 23: MCLK (Master Clock)
 * - SD Card on BUILTIN_SDCARD
 * - Audio files stored on SD card in WAV format
 */

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>

// ========================================
// AUDIO HARDWARE CONFIGURATION
// ========================================

// Audio components from Teensy Audio Design Tool
AudioPlaySdWav   playSdWav1;  // SD card WAV player
AudioMixer4      mixer1;      // Mixer for left channel
AudioMixer4      mixer2;      // Mixer for right channel
AudioOutputI2S2  i2s2_1;      // I2S2 hardware output

// Audio connections (stereo output)
AudioConnection patchCord1(playSdWav1, 0, mixer1, 0);  // Left channel
AudioConnection patchCord2(playSdWav1, 1, mixer2, 0);  // Right channel
AudioConnection patchCord3(mixer1, 0, i2s2_1, 0);      // Left to I2S2
AudioConnection patchCord4(mixer2, 0, i2s2_1, 1);      // Right to I2S2

// ========================================
// GLOBAL VARIABLES
// ========================================

// Audio file list (stored on SD card)
const char *wavefile[9] = {
  "buzzer.wav",
  "Invalid_entry_2.wav",
  "low_battery.wav",
  "wifi_connected.wav",
  "sos_signal.wav",
  "beep.wav",
  "buzzer_double_sound.wav",
  "buzzer_triple_sound.wav",
  "Your_ticket_has_been_validated.wav"
};

// Audio playback control
bool isPlayingAudio = false;
bool playSpeaker = false;
int fileIndex = 0;
int gainValue = 4;  // Default gain (0-10 range typical)

const unsigned long audioPlaybackTimeout = 5000;  // 5 second timeout
unsigned long audioPlaybackStartTime = 0;

// Pin configuration for disabling alternate functions during playback
#define CORE_PIN2_CONFIG  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04
#define CORE_PIN3_CONFIG  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05
#define CORE_PIN4_CONFIG  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06
#define CORE_PIN5_CONFIG  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08
#define CORE_PIN33_CONFIG IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_07

// ========================================
// AUDIO PLAYBACK FUNCTION
// ========================================

void playWavFile()
{
  // Set pins to ALT2 mode for I2S2 (CRITICAL for audio to work!)
  CORE_PIN2_CONFIG  = 2;
  CORE_PIN3_CONFIG  = 2;
  CORE_PIN4_CONFIG  = 2;
  CORE_PIN5_CONFIG  = 2;
  CORE_PIN33_CONFIG = 2;

  // Check if audio is already playing
  if (isPlayingAudio)
  {
    Serial.println("Audio is already playing. Skipping request.");
    return;
  }

  const char *filename;

  // Select the file based on fileIndex
  switch (fileIndex)
  {
    case 1:
      filename = "buzzer.wav";
      break;
    case 2:
      filename = "Invalid_entry_2.wav";
      break;
    case 3:
      filename = "low_battery.wav";
      break;
    case 4:
      filename = "wifi_connected.wav";
      break;
    case 5:
      filename = "sos_signal.wav";
      break;
    case 6:
      filename = "beep.wav";
      break;
    case 7:
      filename = "buzzer_double_sound.wav";
      break;
    case 8:
      filename = "buzzer_triple_sound.wav";
      break;
    case 9:
      filename = "Your_ticket_has_been_validated.wav";
      break;
    default:
      Serial.println("Invalid file index. Please choose between 1 and 9.");
      playSpeaker = false;
      return;
  }

  // Set playback flag
  isPlayingAudio = true;
  audioPlaybackStartTime = millis();

  Serial.print("Playing file: ");
  Serial.println(filename);

  // Start playback
  if (!playSdWav1.play(filename))
  {
    Serial.println("Error: Failed to start playback.");
    playSpeaker = false;
    isPlayingAudio = false;
    return;
  }

  delay(5);

  // Wait until the audio finishes playing (blocking)
  while (playSdWav1.isPlaying())
  {
    // Can add timeout check here if needed:
    // if (millis() - audioPlaybackStartTime > audioPlaybackTimeout) break;
  }

  delay(500);
  Serial.println("Done playing file");

  // Reset playback flags
  isPlayingAudio = false;
  playSpeaker = false;

  delay(10);

  // Re-enable alternate pin functions
  CORE_PIN2_CONFIG  = 0;
  CORE_PIN3_CONFIG  = 0;
  CORE_PIN4_CONFIG  = 0;
  CORE_PIN5_CONFIG  = 0;
  CORE_PIN33_CONFIG = 0;
}

// ========================================
// HELPER FUNCTION: Play specific file
// ========================================

void playAudioFile(int index)
{
  if (index >= 1 && index <= 9)
  {
    fileIndex = index;
    playSpeaker = true;
  }
  else
  {
    Serial.println("Invalid file index. Choose 1-9.");
  }
}

// ========================================
// SETUP
// ========================================

void setup()
{
  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("Teensy 4.1 Audio Playback Example");
  Serial.println("==================================");

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("ERROR: SD card initialization failed!");
    while (1);  // Halt
  }
  Serial.println("SD card initialized");

  // List WAV files on SD card
  Serial.println("\nExpected WAV files:");
  for (int i = 0; i < 9; i++)
  {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(wavefile[i]);
  }

  // Initialize audio system
  AudioMemory(40);  // Allocate 40 audio memory blocks
  delay(200);

  // Set mixer gain (0-10 typical range)
  mixer1.gain(0, gainValue);
  mixer2.gain(0, gainValue);

  Serial.println("\nAudio system initialized");
  Serial.println("Gain set to: " + String(gainValue));
  Serial.println("\nCommands:");
  Serial.println("  1-9: Play audio file");
  Serial.println("  g<value>: Set gain (e.g., 'g6' for gain=6)");
  Serial.println("==================================\n");
}

// ========================================
// MAIN LOOP
// ========================================

void loop()
{
  // Check if playback is requested
  if (playSpeaker && !isPlayingAudio)
  {
    playWavFile();
  }

  // Handle serial commands
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Play audio file (1-9)
    if (command.length() == 1 && command >= "1" && command <= "9")
    {
      int index = command.toInt();
      Serial.println("Command: Play file " + String(index));
      playAudioFile(index);
    }
    // Set gain (g<value>)
    else if (command.startsWith("g") && command.length() > 1)
    {
      String gainStr = command.substring(1);
      int newGain = gainStr.toInt();
      if (newGain >= 0 && newGain <= 10)
      {
        gainValue = newGain;
        mixer1.gain(0, gainValue);
        mixer2.gain(0, gainValue);
        Serial.println("Gain set to: " + String(gainValue));
      }
      else
      {
        Serial.println("Invalid gain. Use 0-10.");
      }
    }
    else
    {
      Serial.println("Unknown command: " + command);
    }
  }

  delay(10);
}

// ========================================
// AUDIO SYSTEM NOTES
// ========================================

/*
 * HARDWARE CONNECTIONS (I2S2):
 * =============================
 * Pin 20: LRCLK (Word Clock / Left-Right Clock)
 * Pin 21: BCLK  (Bit Clock) - Fast edges, can interfere with I2C!
 * Pin 7:  DIN   (Data Out)
 * Pin 23: MCLK  (Master Clock)
 *
 * IMPORTANT: I2C CONFLICT WARNING
 * ================================
 * If using I2C (Wire) on pins 18 (SDA) and 19 (SCL):
 * - Pin 21 (BCLK) can cause crosstalk with pin 19 (SCL)
 * - This causes I2C bus hangs and communication failures
 *
 * SOLUTION:
 * - Use Wire1 (pins 16/17) or Wire2 (pins 24/25) for I2C devices
 * - This physically separates I2C from I2S2 pins
 *
 * AUDIO FILE REQUIREMENTS:
 * ========================
 * - Format: WAV (uncompressed PCM)
 * - Sample Rate: 44.1 kHz recommended
 * - Bit Depth: 16-bit recommended
 * - Channels: Mono or Stereo supported
 * - Storage: SD card root directory
 *
 * AUDIO MEMORY:
 * =============
 * - AudioMemory(40) allocates 40 blocks
 * - Each block = 128 samples @ 44.1kHz = 2.9ms
 * - Total buffer = 40 * 2.9ms = 116ms
 * - Increase if audio glitches occur
 *
 * MIXER GAIN:
 * ===========
 * - Range: 0.0 to ~32.0 (floating point)
 * - Default: 1.0 (unity gain)
 * - Example values:
 *   - 0.5 = half volume (-6dB)
 *   - 1.0 = normal volume (0dB)
 *   - 2.0 = double volume (+6dB)
 *   - 4.0 = 4x volume (+12dB)
 *   - 10.0 = very loud (clipping likely)
 *
 * PLAYBACK BLOCKING:
 * ==================
 * - playWavFile() is BLOCKING (waits for audio to finish)
 * - For non-blocking, remove the while(isPlaying()) loop
 * - Check playSdWav1.isPlaying() in main loop instead
 *
 * TROUBLESHOOTING:
 * ================
 * 1. "Failed to start playback" error:
 *    - Check SD card is formatted FAT32
 *    - Verify WAV file exists on SD card
 *    - Ensure file is valid WAV format
 *
 * 2. Audio glitches/stuttering:
 *    - Increase AudioMemory() value (try 60-80)
 *    - Reduce SD card read overhead
 *    - Check for CPU-intensive tasks blocking audio
 *
 * 3. No audio output:
 *    - Verify I2S2 codec wiring
 *    - Check mixer gain is not zero
 *    - Test with known-good WAV file
 *
 * 4. I2C devices not working:
 *    - Move I2C to Wire1 (pins 16/17)
 *    - See I2S/I2C conflict notes above
 */
