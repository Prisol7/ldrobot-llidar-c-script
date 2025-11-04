#include "hexscript.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <math.h>

static int serial_fd = -1;

int initLidar(const char* port, int baudrate) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(serial_fd);
        return -1;
    }

    // Set baudrate
    speed_t speed = B230400;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(serial_fd);
        return -1;
    }

    // Flush the buffer
    tcflush(serial_fd, TCIOFLUSH);

    printf("LIDAR initialized on %s at %d baud\n", port, baudrate);
    return 0;
}

void closeLidar(void) {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}

int parseLidar(LidarPoint* points, int* count) {
    uint8_t byte;
    int header_found = 0;
    
    *count = 0;

    // Search for header 'T' (0x54)
    while (!header_found) {
        if (read(serial_fd, &byte, 1) != 1) {
            return -1;
        }
        if (byte == 0x54) {
            header_found = 1;
        }
    }

    // Read the 46-byte packet
    uint8_t packet[46];
    int bytes_read = 0;
    while (bytes_read < 46) {
        int n = read(serial_fd, packet + bytes_read, 46 - bytes_read);
        if (n < 0) {
            fprintf(stderr, "Error reading packet: %s\n", strerror(errno));
            return -1;
        }
        bytes_read += n;
    }

    // Verify length field
    if (packet[0] != 44) {
        fprintf(stderr, "Invalid packet length: %d\n", packet[0]);
        return -1;
    }

    // Parse packet structure (little-endian)
    // uint16_t speed = packet[1] | (packet[2] << 8);
    uint16_t initial_angle_raw = packet[3] | (packet[4] << 8);
    uint16_t final_angle_raw = packet[41] | (packet[42] << 8);

    float initial_angle = initial_angle_raw / 100.0f;
    float final_angle = final_angle_raw / 100.0f;

    // Handle wraparound
    if (final_angle < initial_angle) {
        final_angle += 360.0f;
    }

    float angle_step = (final_angle - initial_angle) / 12.0f;

    // Parse 12 measurement points
    for (int i = 0; i < 12; i++) {
        int offset = 5 + (i * 3);
        uint16_t distance = packet[offset] | (packet[offset + 1] << 8);
        uint8_t intensity = packet[offset + 2];

        float angle_deg = initial_angle + (angle_step * i);
        float angle_rad = angle_deg * (M_PI / 180.0f);

        points[*count].angle = angle_rad;
        points[*count].distance = distance;
        points[*count].intensity = intensity;
        (*count)++;
    }

    return 0;
}

int getScan(LidarScan* scan) {
    scan->count = 0;

    for (int i = 0; i < PACKETS_PER_SCAN; i++) {
        int packet_count = 0;
        if (parseLidar(scan->points + scan->count, &packet_count) != 0) {
            fprintf(stderr, "Error parsing packet %d\n", i);
            return -1;
        }
        scan->count += packet_count;
    }

    return 0;
}