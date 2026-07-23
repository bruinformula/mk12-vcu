/*
 * audio.h
 *
 *  Created on: Mar 31, 2026
 *      Author: ishanchitale
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include <stdbool.h>
#include "vcu_state.h"
#include "i2s.h"

#define WAV_HEADER_SIZE 44
#define WAV_DATA_SIZE (startup_sound_len - WAV_HEADER_SIZE)
#define TOTAL_HALFWORDS (WAV_DATA_SIZE / 2)
#define CHUNK_SIZE_HALFWORDS 4096U
#define TEST_TONE_FRAME_COUNT 4410U
#define TEST_TONE_CHANNEL_COUNT 2U
#define TEST_TONE_HALFWORD_COUNT                                             \
  (TEST_TONE_FRAME_COUNT * TEST_TONE_CHANNEL_COUNT)
#define TEST_TONE_PERIOD_FRAMES 44U
#define BOOT_AUDIO_SELFTEST 0U
#define DIRECT_RTD_AUDIO_BUTTON 1U
#define AUDIO_RETRIGGER_GUARD_MS 750U
#define RTD_VOLUME_SHIFT 2U
#define TEST_TONE_HIGH_SAMPLE ((uint16_t)0x1000)
#define TEST_TONE_LOW_SAMPLE ((uint16_t)0xF000)

typedef struct DebugStats {
    // I2S state + error info
    uint32_t i2s_state;
    uint32_t i2s_error;
    uint32_t i2s_error_cb_hit;

    // Clock + peripheral debug
    uint32_t spi123_clock_source;
    uint32_t spi123_clock_hz;

    // Raw register snapshots
    uint32_t spi2_cr1;
    uint32_t spi2_i2scfgr;

    // Playback tracking
    uint32_t playback_kind;          // 1 = test tone, 2 = RTD sound
    uint32_t playback_started;
    uint32_t playback_finished;

    // Wave progress
    uint32_t wave_halfword_count;
    uint32_t wave_halfword_position;

    // I2S transmission stats
    uint32_t i2s_tx_hit;             // Tx complete callback hits
    uint32_t last_i2s_status;

    // DMA control/debug
    uint32_t last_dma_stop_status;
} DebugStats;

void fillTestToneBuffer();
void captureI2SDebugState();
bool audioPlaybackAllowed();
uint16_t *preparePlaybackChunk(const uint16_t *source, uint16_t count);
bool playReadyToDriveSound();
void testSpeaker();

#endif /* INC_AUDIO_H_ */
