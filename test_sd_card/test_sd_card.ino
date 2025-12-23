/*
 * SD Card Test for ESP32
 * Tests if SD card can be read via SPI
 * 
 * Hardware:
 * - ESP32 (TTGO Camera Plus)
 * - SD Card on SPI bus
 * - CS: GPIO 0
 * - MOSI: GPIO 19
 * - MISO: GPIO 22
 * - SCK: GPIO 21
 */

#include <SPI.h>
#include <SD.h>

// SD Card pins (matching ESPHome config)
#define SD_CS_PIN 0
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 22
#define SD_SCK_PIN 21

void runTests() {
  Serial.println("\n\n========================================");
  Serial.println("ESP32 SD Card Test");
  Serial.println("========================================\n");
  
  // Initialize SPI with custom pins
  // SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN); // Move to setup
  
  Serial.println("Initializing SD card...");
  Serial.print("  CS Pin: ");
  Serial.println(SD_CS_PIN);
  
  // Try to initialize SD card
  if (!SD.begin(SD_CS_PIN, SPI, 400000)) {
    Serial.println("ERROR: SD card initialization failed!");
    Serial.println("Possible causes:");
    Serial.println("  - No SD card inserted");
    Serial.println("  - Wrong wiring");
    Serial.println("  - Card not formatted");
    Serial.println("  - Bad SD card");
    return;
  }
  
  Serial.println("SUCCESS: SD card initialized!\n");
  
  // Get card info
  uint8_t cardType = SD.cardType();
  Serial.print("Card Type: ");
  switch(cardType) {
    case CARD_NONE:
      Serial.println("NONE");
      return;
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SDSC");
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print("Card Size: ");
  Serial.print(cardSize);
  Serial.println(" MB");
  
  uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
  Serial.print("Total Space: ");
  Serial.print(totalBytes);
  Serial.println(" MB");
  
  uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
  Serial.print("Used Space: ");
  Serial.print(usedBytes);
  Serial.println(" MB\n");
  
  // Test 1: Check if /animations directory exists
  Serial.println("========================================");
  Serial.println("Test 1: Check /animations directory");
  Serial.println("========================================");
  
  File animDir = SD.open("/animations");
  if (!animDir) {
    Serial.println("ERROR: Cannot open /animations directory!");
    Serial.println("  Directory may not exist");
  } else {
    Serial.println("SUCCESS: /animations directory opened!");
    
    if (!animDir.isDirectory()) {
      Serial.println("ERROR: /animations is not a directory!");
    } else {
      Serial.println("SUCCESS: /animations is a directory!");
      
      // Count files
      int fileCount = 0;
      int jsonCount = 0;
      
      Serial.println("\nListing first 20 files in /animations:");
      Serial.println("----------------------------------------");
      
      File file = animDir.openNextFile();
      while (file && fileCount < 20) {
        if (!file.isDirectory()) {
          Serial.print("  ");
          Serial.print(file.name());
          Serial.print(" (");
          Serial.print(file.size());
          Serial.println(" bytes)");
          
          String filename = String(file.name());
          if (filename.endsWith(".json")) {
            jsonCount++;
          }
          fileCount++;
        }
        file.close();
        file = animDir.openNextFile();
      }
      
      // Count remaining files
      while (file) {
        if (!file.isDirectory()) {
          fileCount++;
          String filename = String(file.name());
          if (filename.endsWith(".json")) {
            jsonCount++;
          }
        }
        file.close();
        file = animDir.openNextFile();
      }
      
      Serial.println("----------------------------------------");
      Serial.print("Total files found: ");
      Serial.println(fileCount);
      Serial.print("JSON files: ");
      Serial.println(jsonCount);
    }
    animDir.close();
  }
  
  Serial.println();
  
  // Test 2: Try to read a specific animation file
  Serial.println("========================================");
  Serial.println("Test 2: Read specific animation file");
  Serial.println("========================================");
  
  const char* testFile = "/animations/anim_blackjack_idle_01.json";
  Serial.print("Trying to open: ");
  Serial.println(testFile);
  
  File file = SD.open(testFile);
  if (!file) {
    Serial.println("ERROR: Cannot open file!");
    Serial.println("  File may not exist");
  } else {
    Serial.println("SUCCESS: File opened!");
    Serial.print("  File size: ");
    Serial.print(file.size());
    Serial.println(" bytes");
    
    // Read first 200 bytes
    Serial.println("\nFirst 200 bytes of file:");
    Serial.println("----------------------------------------");
    
    int bytesRead = 0;
    while (file.available() && bytesRead < 200) {
      Serial.write(file.read());
      bytesRead++;
    }
    Serial.println("\n----------------------------------------");
    
    file.close();
  }
  
  Serial.println();
  
  // Test 3: Check /audio directory
  Serial.println("========================================");
  Serial.println("Test 3: Check /audio directory");
  Serial.println("========================================");
  
  File audioDir = SD.open("/audio");
  if (!audioDir) {
    Serial.println("ERROR: Cannot open /audio directory!");
  } else {
    Serial.println("SUCCESS: /audio directory opened!");
    
    if (audioDir.isDirectory()) {
      int audioCount = 0;
      File file = audioDir.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          audioCount++;
        }
        file.close();
        file = audioDir.openNextFile();
      }
      Serial.print("Audio files found: ");
      Serial.println(audioCount);
    }
    audioDir.close();
  }
  
  Serial.println();
  
  // Test 4: Check root directory
  Serial.println("========================================");
  Serial.println("Test 4: List root directory");
  Serial.println("========================================");
  
  File root = SD.open("/");
  if (!root) {
    Serial.println("ERROR: Cannot open root directory!");
  } else {
    Serial.println("Contents of root directory:");
    Serial.println("----------------------------------------");
    
    File file = root.openNextFile();
    while (file) {
      Serial.print("  ");
      Serial.print(file.name());
      if (file.isDirectory()) {
        Serial.println(" [DIR]");
      } else {
        Serial.print(" (");
        Serial.print(file.size());
        Serial.println(" bytes)");
      }
      file.close();
      file = root.openNextFile();
    }
    Serial.println("----------------------------------------");
    root.close();
  }
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("SD Card Test Complete!");
  Serial.println("========================================\n");
}


#define TFT_CS_PIN 12
#define DC_PIN 15

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Deselect TFT Display to avoid SPI conflict
  pinMode(TFT_CS_PIN, OUTPUT);
  digitalWrite(TFT_CS_PIN, HIGH);
  
  // Set DC pin to known state (HIGH)
  pinMode(DC_PIN, OUTPUT);
  digitalWrite(DC_PIN, HIGH);
  
  delay(100);

  // Initialize SPI with custom pins
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  Serial.println("Setup Complete. Starting tests loop...");
}

void loop() {
  runTests();
  delay(10000);
}
