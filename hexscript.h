#ifndef HEXSCRIPT_H
#define HEXSCRIPT_H

#include <stdint.h>

#define MAX_POINTS_PER_PACKET 12
#define PACKETS_PER_SCAN 38
#define TOTAL_POINTS (MAX_POINTS_PER_PACKET * PACKETS_PER_SCAN)

typedef struct {
    float angle;//radians
    uint16_t distance;//mm
    uint8_t intensity;//0-255
} LidarPoint;

typedef struct {
    LidarPoint points[TOTAL_POINTS];
    int count;
} LidarScan;

// Initialize serial connection to LIDAR
int initLidar(const char* port, int baudrate);

// Close serial connection
void closeLidar(void);

// Parse one packet from LIDAR (12 points)
int parseLidar(LidarPoint* points, int* count);

// Get a complete 360° scan (38 packets = 456 points)
int getScan(LidarScan* scan);

#endif 