/**
 * USB Radio link protocol — shared between the Flipper app and the host
 * bridge (host/usbradio_bridge.py). Keep both sides in sync.
 *
 * Frame layout (both directions), little-endian:
 *   [0xF0] [0x9D] [type u8] [len u8] [payload len bytes] [chk u8]
 * chk = XOR of type, len and all payload bytes.
 */
#pragma once

#include <stdint.h>

#define UR_MAGIC1 0xF0
#define UR_MAGIC2 0x9D

#define UR_PROTO_VERSION 1

#define UR_SPECTRUM_BINS 128

/* Flipper -> host */
#define UR_MSG_PING   0x01
#define UR_MSG_CONFIG 0x02

/* Host -> Flipper */
#define UR_MSG_HELLO    0x81
#define UR_MSG_SPECTRUM 0x82
#define UR_MSG_STATUS   0x83
#define UR_MSG_PULSES   0x84

/* UR_MSG_CONFIG modes */
#define UR_MODE_IDLE     0
#define UR_MODE_SPECTRUM 1
#define UR_MODE_CAPTURE  2

#define UR_GAIN_AUTO 0xFFFF

typedef struct __attribute__((packed)) {
    uint8_t mode; /* UR_MODE_* */
    uint32_t freq_hz;
    uint32_t span_hz; /* desired spectrum span; host picks nearest supported rate */
    uint16_t gain_db_tenths; /* UR_GAIN_AUTO for AGC */
    uint8_t fps; /* spectrum frames per second */
} UrConfig;

typedef struct __attribute__((packed)) {
    uint8_t proto_version;
    uint8_t flags; /* bit0: backend can transmit (reserved for future TX) */
    char name[22]; /* zero-terminated SDR name */
} UrHello;

/* UR_MSG_SPECTRUM payload: [seq u8][bins u8 * UR_SPECTRUM_BINS]
 * bin value encodes power: dBFS = value - 130 (0 => -130 dB, 255 => +125 dB) */
#define UR_DB_OFFSET 130

/* UR_MSG_STATUS payload: [code u8][text char[], zero-terminated, optional] */
#define UR_STATUS_OK        0
#define UR_STATUS_SDR_ERROR 1

/* UR_MSG_PULSES payload: [count u8][int32 duration us * count]
 * Positive = carrier on (mark), negative = carrier off (space).
 * Host guarantees alternating signs, first pulse of a capture positive. */
#define UR_PULSES_MAX 48
