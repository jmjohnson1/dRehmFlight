#include "RingBuf.h"
#include "SdFat.h"

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points (usec) for 100 samples/sec
#define LOG_INTERVAL_USEC 10000

// Size to log 256 byte lines at 100 Hz for more than ten minutes.
#define LOG_FILE_SIZE 256 * 100 * 600  // 150,000,000 bytes.

// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512
#define LOG_FILENAME "SdioLogger.csv"

SdFs sd;
FsFile file;


// Ring buffer for filetype FsFile (The filemanager that will handle the data stream)
RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

void logData() {
	// Initialize the SD
	if (!sd.begin(SD_CONFIG)) {
		sd.initErrorHalt(&Serial); // Prints message to serial if SD can't init
	}
	// Open or create file - truncate existing
	if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC) { // NOTE: Lookup meaning
		Serial.println("open failed\n");
		return;
	}
}
