/*
 * Teensy 4.1 - SD Card Audio Player
 * Automatically finds and plays WAV files from SD card
 *
 * Hardware:
 * - Teensy 4.1 with built-in SD card slot
 * - I2S2 Audio Codec (pins 7, 20, 21, 23)
 * - WAV files on SD card root directory
 *
 * Usage:
 * - List all WAV files on SD card
 * - Type number to play file
 * - Type 'l' to list files again
 * - Type 'p' to pause/resume
 * - Type 's' to stop playback
 */

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>

// ========================================
// AUDIO HARDWARE SETUP
// ========================================

AudioPlaySdWav   playSdWav1;  // SD card WAV player
AudioMixer4      mixer1;      // Left channel mixer
AudioMixer4      mixer2;      // Right channel mixer
AudioOutputI2S2  i2s2_1;      // I2S2 output

// Audio connections
AudioConnection patchCord1(playSdWav1, 0, mixer1, 0);
AudioConnection patchCord2(playSdWav1, 1, mixer2, 0);
AudioConnection patchCord3(mixer1, 0, i2s2_1, 0);
AudioConnection patchCord4(mixer2, 0, i2s2_1, 1);

// ========================================
// GLOBAL VARIABLES
// ========================================

const int MAX_FILES = 50;
String wavFiles[MAX_FILES];
int fileCount = 0;
bool isPlaying = false;
int currentGain = 4;

// ========================================
// SCAN SD CARD FOR WAV FILES
// ========================================

void scanSDForWavFiles()
{
  Serial.println("\n========================================");
  Serial.println("Scanning SD card for WAV files...");
  Serial.println("========================================");

  fileCount = 0;
  File root = SD.open("/");

  if (!root)
  {
    Serial.println("ERROR: Failed to open root directory!");
    return;
  }

  if (!root.isDirectory())
  {
    Serial.println("ERROR: Root is not a directory!");
    return;
  }

  // Scan all files in root directory
  while (true)
  {
    File entry = root.openNextFile();
    if (!entry)
    {
      break;  // No more files
    }

    String fileName = entry.name();

    // Check if file is a WAV file (case insensitive)
    if (fileName.endsWith(".wav") || fileName.endsWith(".WAV"))
    {
      if (fileCount < MAX_FILES)
      {
        wavFiles[fileCount] = fileName;
        fileCount++;
      }
    }

    entry.close();
  }

  root.close();

  // Display results
  Serial.print("Found ");
  Serial.print(fileCount);
  Serial.println(" WAV file(s):");
  Serial.println("----------------------------------------");

  if (fileCount == 0)
  {
    Serial.println("No WAV files found on SD card!");
    Serial.println("Please add .wav files to SD card root.");
  }
  else
  {
    for (int i = 0; i < fileCount; i++)
    {
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(wavFiles[i]);
    }
  }

  Serial.println("========================================\n");
}

// ========================================
// PLAY WAV FILE
// ========================================

void playWavFile(int index)
{
  if (index < 0 || index >= fileCount)
  {
    Serial.println("ERROR: Invalid file number!");
    return;
  }

  String filename = wavFiles[index];

  Serial.println("----------------------------------------");
  Serial.print("Playing: ");
  Serial.println(filename);
  Serial.println("----------------------------------------");

  // Stop any currently playing audio
  playSdWav1.stop();
  delay(10);

  // Start playback
  if (playSdWav1.play(filename.c_str()))
  {
    isPlaying = true;
    Serial.println("Playback started");

    // Show playback status
    delay(100);

    // Display file info
    Serial.print("Duration: ");
    Serial.print(playSdWav1.lengthMillis() / 1000.0, 1);
    Serial.println(" seconds");
  }
  else
  {
    Serial.println("ERROR: Failed to play file!");
    Serial.println("Check if file is valid WAV format.");
    isPlaying = false;
  }
}

// ========================================
// STOP PLAYBACK
// ========================================

void stopPlayback()
{
  if (isPlaying)
  {
    playSdWav1.stop();
    isPlaying = false;
    Serial.println("Playback stopped");
  }
  else
  {
    Serial.println("No audio playing");
  }
}

// ========================================
// SHOW PLAYBACK STATUS
// ========================================

void showStatus()
{
  if (isPlaying && playSdWav1.isPlaying())
  {
    Serial.print("Playing... ");
    Serial.print(playSdWav1.positionMillis() / 1000.0, 1);
    Serial.print(" / ");
    Serial.print(playSdWav1.lengthMillis() / 1000.0, 1);
    Serial.println(" seconds");
  }
  else if (isPlaying)
  {
    Serial.println("Playback finished");
    isPlaying = false;
  }
  else
  {
    Serial.println("Status: Idle");
  }
}

// ========================================
// SET GAIN
// ========================================

void setGain(int gain)
{
  if (gain < 0 || gain > 32)
  {
    Serial.println("ERROR: Gain must be 0-32");
    return;
  }

  currentGain = gain;
  mixer1.gain(0, currentGain);
  mixer2.gain(0, currentGain);

  Serial.print("Gain set to: ");
  Serial.println(currentGain);
}

// ========================================
// PRINT HELP
// ========================================

void printHelp()
{
  Serial.println("\n========================================");
  Serial.println("SD CARD AUDIO PLAYER - COMMANDS");
  Serial.println("========================================");
  Serial.println("1-50     Play file by number");
  Serial.println("l        List all WAV files");
  Serial.println("s        Stop playback");
  Serial.println("?        Show playback status");
  Serial.println("g<num>   Set gain (e.g., g6)");
  Serial.println("h        Show this help");
  Serial.println("========================================\n");
}

// ========================================
// SETUP
// ========================================

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  TEENSY 4.1 SD CARD AUDIO PLAYER");
  Serial.println("========================================");

  // Initialize SD card
  Serial.print("Initializing SD card... ");

  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("FAILED!");
    Serial.println("\nERROR: SD card not found or failed to mount");
    Serial.println("Please check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - SD card is formatted as FAT32");
    Serial.println("  - SD card has WAV files");
    while (1) delay(1000);
  }

  Serial.println("OK");

  // Initialize audio system
  Serial.print("Initializing audio... ");
  AudioMemory(40);
  mixer1.gain(0, currentGain);
  mixer2.gain(0, currentGain);
  Serial.println("OK");

  Serial.print("Audio gain: ");
  Serial.println(currentGain);
  Serial.println();

  // Scan for WAV files
  scanSDForWavFiles();

  // Show help
  printHelp();

  if (fileCount > 0)
  {
    Serial.print("Ready! Type 1-");
    Serial.print(fileCount);
    Serial.println(" to play a file");
  }
}

// ========================================
// MAIN LOOP
// ========================================

void loop()
{
  // Handle serial commands
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    if (command.length() == 0)
    {
      return;
    }

    // List files
    if (command == "l")
    {
      scanSDForWavFiles();
    }
    // Stop playback
    else if (command == "s")
    {
      stopPlayback();
    }
    // Show status
    else if (command == "?")
    {
      showStatus();
    }
    // Help
    else if (command == "h")
    {
      printHelp();
    }
    // Set gain
    else if (command.startsWith("g"))
    {
      String gainStr = command.substring(1);
      int newGain = gainStr.toInt();
      setGain(newGain);
    }
    // Play file by number
    else if (command.length() >= 1 && isDigit(command.charAt(0)))
    {
      int fileNum = command.toInt();
      if (fileNum >= 1 && fileNum <= fileCount)
      {
        playWavFile(fileNum - 1);
      }
      else
      {
        Serial.print("ERROR: Enter number between 1 and ");
        Serial.println(fileCount);
      }
    }
    else
    {
      Serial.print("Unknown command: ");
      Serial.println(command);
      Serial.println("Type 'h' for help");
    }
  }

  // Auto-detect when playback finishes
  static bool wasPlaying = false;
  if (isPlaying && !playSdWav1.isPlaying())
  {
    if (wasPlaying)
    {
      Serial.println("\n✓ Playback finished\n");
      isPlaying = false;
      wasPlaying = false;
    }
  }
  else if (isPlaying)
  {
    wasPlaying = true;
  }

  delay(50);
}

// ========================================
// NOTES
// ========================================

/*
 * WAV FILE REQUIREMENTS:
 * ======================
 * - Format: WAV (PCM uncompressed)
 * - Sample Rate: 44.1kHz or 48kHz recommended
 * - Bit Depth: 16-bit recommended
 * - Channels: Mono or Stereo
 * - File Location: SD card root directory
 * - Max filename length: 31 characters
 *
 * SD CARD FORMATTING:
 * ===================
 * - File System: FAT32 (recommended)
 * - Also supports: exFAT, FAT16
 * - Max card size: 32GB for FAT32
 *
 * AUDIO CONNECTIONS (I2S2):
 * ==========================
 * Pin 20: LRCLK (Word Clock)
 * Pin 21: BCLK (Bit Clock)
 * Pin 7:  DIN (Data Out)
 * Pin 23: MCLK (Master Clock)
 *
 * WARNING - I2C CONFLICT:
 * =======================
 * If using I2C devices, avoid Wire (pins 18/19)
 * Pin 21 (BCLK) interferes with pin 19 (SCL)
 * Use Wire1 (pins 16/17) or Wire2 (pins 24/25) instead
 *
 * TROUBLESHOOTING:
 * ================
 * 1. "SD card not found":
 *    - Check SD card is inserted
 *    - Try reformatting as FAT32
 *    - Check card is not write-protected
 *
 * 2. "Failed to play file":
 *    - Verify file is valid WAV format
 *    - Check sample rate (44.1kHz works best)
 *    - Try converting with Audacity
 *
 * 3. Audio stutters/glitches:
 *    - Increase AudioMemory (try 60-80)
 *    - Use lower sample rate
 *    - Check SD card speed (Class 10)
 *
 * 4. No audio output:
 *    - Verify I2S connections
 *    - Check mixer gain (try g10)
 *    - Test with known-good WAV file
 *
 * CONVERTING AUDIO FILES:
 * =======================
 * Use Audacity (free):
 * 1. Open your audio file
 * 2. File > Export > Export as WAV
 * 3. Settings:
 *    - Format: WAV (Microsoft)
 *    - Encoding: Signed 16-bit PCM
 *    - Sample Rate: 44100 Hz
 * 4. Copy to SD card
 */
