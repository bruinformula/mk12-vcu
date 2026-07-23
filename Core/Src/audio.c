/*
 * audio.c
 *
 *  Created on: Mar 31, 2026
 *      Author: ishanchitale
 */

#include "audio.h"
#include "startup_sound.h"
#include "test_sound.h"

static DebugStats debug;
static uint32_t wavPos = 0;
static const uint16_t *wavePCM = NULL;
static uint32_t halfwordCount = 0;
static uint8_t waveFinished = 0;
static uint16_t testToneBuffer[TEST_TONE_HALFWORD_COUNT];
static uint16_t scaledPlaybackBuffer[CHUNK_SIZE_HALFWORDS];
static bool testToneInitialized = false;
static bool setReadyToDriveOnPlaybackComplete = false;
static uint32_t lastPlaybackStartTick = 0;
static bool hasStartedPlayback = false;
static bool scalePlaybackSamples = false;

void fillTestToneBuffer() {
  if (testToneInitialized) {
    return;
  }

  for (uint32_t frame = 0; frame < TEST_TONE_FRAME_COUNT; frame++) {
    uint16_t sample =
        ((frame % TEST_TONE_PERIOD_FRAMES) < (TEST_TONE_PERIOD_FRAMES / 2U))
            ? TEST_TONE_HIGH_SAMPLE
            : TEST_TONE_LOW_SAMPLE;
    uint32_t offset = frame * TEST_TONE_CHANNEL_COUNT;

    testToneBuffer[offset] = sample;
    testToneBuffer[offset + 1U] = sample;
  }

  testToneInitialized = true;
}

void captureI2SDebugState() {
  debug.i2s_state = HAL_I2S_GetState(&hi2s2);
  debug.i2s_error = HAL_I2S_GetError(&hi2s2);
  debug.spi123_clock_source = __HAL_RCC_GET_SPI123_SOURCE();
  debug.spi123_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
  debug.spi2_cr1 = hi2s2.Instance->CR1;
  debug.spi2_i2scfgr = hi2s2.Instance->I2SCFGR;
}

bool audioPlaybackAllowed() {
  uint32_t now = HAL_GetTick();
  if (hasStartedPlayback &&
      (now - lastPlaybackStartTick) < AUDIO_RETRIGGER_GUARD_MS) {
    return false;
  }

  lastPlaybackStartTick = now;
  hasStartedPlayback = true;
  return true;
}

uint16_t *preparePlaybackChunk(const uint16_t *source, uint16_t count) {
  if (!scalePlaybackSamples) {
    return (uint16_t *)source;
  }

  for (uint16_t i = 0; i < count; i++) {
    int16_t sample = (int16_t)source[i];
    scaledPlaybackBuffer[i] = (uint16_t)(sample >> RTD_VOLUME_SHIFT);
  }

  return scaledPlaybackBuffer;
}

bool playReadyToDriveSound(void) {
    // Ensure peripheral is ready and not retriggered too quickly
    if (HAL_I2S_GetState(&hi2s2) != HAL_I2S_STATE_READY ||
        !audioPlaybackAllowed()) {
        return false;
    }

    // Initialize playback state
    scalePlaybackSamples = true;
    debug.playback_kind = 2;
    debug.playback_started++;
    wavePCM = (const uint16_t *)(&startup_sound[WAV_HEADER_SIZE]);
    halfwordCount = TOTAL_HALFWORDS;
    wavPos = 0;
    waveFinished = 0;
    debug.wave_halfword_count = halfwordCount;
    debug.wave_halfword_position = 0;

    // Blocking playback loop
    while (wavPos < halfwordCount) {
        uint32_t remain = halfwordCount - wavPos;
        uint16_t thisChunk =
            (remain > CHUNK_SIZE_HALFWORDS) ? CHUNK_SIZE_HALFWORDS
                                           : (uint16_t)remain;
        const uint16_t *chunkPtr = wavePCM + wavPos;
        debug.last_i2s_status =
            HAL_I2S_Transmit(&hi2s2,
                             preparePlaybackChunk(chunkPtr, thisChunk),
                             thisChunk,
                             1000);
        captureI2SDebugState();
        if (debug.last_i2s_status != HAL_OK) {
            waveFinished = 1;
            wavPos = 0;
            debug.wave_halfword_position = 0;
            // Ensure I2S is not stuck in a bad state
            HAL_I2S_DMAStop(&hi2s2);
            return false;
        }
        wavPos += thisChunk;
        debug.wave_halfword_position = wavPos;
    }

    // Playback completed successfully
    waveFinished = 1;
    lastPlaybackStartTick = HAL_GetTick();
    debug.playback_finished++;
    debug.wave_halfword_position = halfwordCount;
    return true;
}

void testSpeaker() {
  if (!audioPlaybackAllowed()) {
    return;
  }

  if (HAL_I2S_GetState(&hi2s2) != HAL_I2S_STATE_READY) {
    captureI2SDebugState();
    if (debug.i2s_state == HAL_I2S_STATE_BUSY ||
        debug.i2s_state == HAL_I2S_STATE_BUSY_TX ||
        debug.i2s_state == HAL_I2S_STATE_BUSY_TX_RX) {
      debug.last_dma_stop_status = HAL_I2S_DMAStop(&hi2s2);
    }

    captureI2SDebugState();
    if (debug.i2s_state != HAL_I2S_STATE_READY) {
      return;
    }
  }

  setReadyToDriveOnPlaybackComplete = false;
  scalePlaybackSamples = false;
  debug.playback_kind = 1;
  debug.playback_started++;
  wavePCM = testToneBuffer;
  halfwordCount = TEST_TONE_HALFWORD_COUNT;
  wavPos = 0;
  waveFinished = 0;
  debug.wave_halfword_count = halfwordCount;
  debug.wave_halfword_position = 0;

  uint32_t remain = halfwordCount - wavPos;
  uint16_t thisChunk =
      (remain > CHUNK_SIZE_HALFWORDS) ? CHUNK_SIZE_HALFWORDS : (uint16_t)remain;
  const uint16_t *chunkPtr = wavePCM + wavPos;
  wavPos += thisChunk;
  debug.wave_halfword_position = wavPos;

  debug.last_i2s_status =
      HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)chunkPtr, thisChunk);
  captureI2SDebugState();
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
  debug.i2s_tx_hit++;
  if (hi2s->Instance == SPI2 && !waveFinished) {
    // finished one chunk
    if (wavPos < halfwordCount) {
      // Start the next chunk
      uint32_t remain = halfwordCount - wavPos;
      uint16_t thisChunk = (remain > CHUNK_SIZE_HALFWORDS)
                               ? CHUNK_SIZE_HALFWORDS
                               : (uint16_t)remain;
      const uint16_t *chunkPtr = wavePCM + wavPos;
      wavPos += thisChunk;
      debug.wave_halfword_position = wavPos;

      HAL_I2S_Transmit_DMA(&hi2s2, preparePlaybackChunk(chunkPtr, thisChunk),
                           thisChunk);
    } else {
      // entire wave is done
      waveFinished = 1;
      lastPlaybackStartTick = HAL_GetTick();
      debug.playback_finished++;
      debug.wave_halfword_position = halfwordCount;
      if (setReadyToDriveOnPlaybackComplete) {
        setReadyToDriveOnPlaybackComplete = false;
      }
    }
  }
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
  if (hi2s->Instance == SPI2) {
    debug.i2s_error_cb_hit++;
    captureI2SDebugState();
  }
}
