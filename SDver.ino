#include <SPI.h>
#include <SD.h>

// SD utilities
Sd2Card card;
SdVolume volume;
SdFile root;

// Use pin 53 for CS on Arduino Mega
const int chipSelect = 53;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Serial.print("\nInitializing SD card...");

  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("Initialization failed. Things to check:");
    Serial.println("* Is a card inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* Did you set the correct CS pin?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // Print card type
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1: Serial.println("SD1"); break;
    case SD_CARD_TYPE_SD2: Serial.println("SD2"); break;
    case SD_CARD_TYPE_SDHC: Serial.println("SDHC"); break;
    default: Serial.println("Unknown"); break;
  }

  // Try to access volume
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card.");
    while (1);
  }

  Serial.print("Clusters:          "); Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  "); Serial.println(volume.blocksPerCluster());
  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  uint32_t volumesize = volume.blocksPerCluster() * volume.clusterCount() / 2;
  Serial.print("Volume type is:    FAT"); Serial.println(volume.fatType(), DEC);
  Serial.print("Volume size (KB):  "); Serial.println(volumesize);
  Serial.print("Volume size (MB):  "); Serial.println(volumesize / 1024);
  Serial.print("Volume size (GB):  "); Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  root.ls(LS_R | LS_DATE | LS_SIZE);
}

void loop() {
  // Nothing in loop
}
