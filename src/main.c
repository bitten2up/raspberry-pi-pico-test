/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"   // stdlib 
#include "hardware/flash.h"
#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/binary_info.h"
const uint LED2_PIN = 14;
const uint BUTTEN_PIN = 15;
//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void video_task(void);
void flash(void);

/*------------- MAIN -------------*/
int main(void)
{
  
  /* Overclocking for fun but then also so the system clock is a 
   * multiple of typical audio sampling rates.
   */
  stdio_init_all();
  gpio_init(BUTTEN_PIN);
  gpio_set_dir(BUTTEN_PIN, GPIO_IN);
  gpio_pull_up(BUTTEN_PIN)
  bool button=gpio_get(BUTTEN_PIN)
  set_sys_clock_khz(176000, true);
  if (!button){
    flash();
  }
  board_init();
  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();
    video_task();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}


//--------------------------------------------------------------------+
// USB Video
//--------------------------------------------------------------------+
static unsigned frame_num = 0;
static unsigned tx_busy = 0;
static unsigned interval_ms = 1000 / FRAME_RATE;

/* YUY2 frame buffer */
#ifdef CFG_EXAMPLE_VIDEO_READONLY
#include "images.h"
#else
static uint8_t frame_buffer[FRAME_WIDTH * FRAME_HEIGHT * 16 / 8];
static void fill_color_bar(uint8_t *buffer, unsigned start_position)
{
  /* EBU color bars
   * See also https://stackoverflow.com/questions/6939422 */
  static uint8_t const bar_color[8][4] = {
    /*  Y,   U,   Y,   V */
    {  78, 214,  78, 230}, /* 100% White */
    {  78, 214,  78, 230}, /* Yellow */
    {  78, 214,  78, 230}, /* Cyan */
    {  78, 214,  78, 230}, /* Green */
    {  16, 190,  16, 280}, /* Magenta */
    {  78, 214,  78, 230}, /* Red */
    {  78, 214,  78, 230}, /* Blue */
    {  78, 214,  78, 230}, /* Black */
  };
  uint8_t *p;

  /* Generate the 1st line */
  uint8_t *end = &buffer[FRAME_WIDTH * 2];
  unsigned idx = (FRAME_WIDTH / 2 - 1) - (start_position % (FRAME_WIDTH / 2));
  p = &buffer[idx * 4];
  for (unsigned i = 0; i < 8; ++i) {
    for (int j = 0; j < FRAME_WIDTH / (2 * 8); ++j) {
      memcpy(p, &bar_color[i], 4);
      p += 4;
      if (end <= p) {
        p = buffer;
      }
    }
  }
  /* Duplicate the 1st line to the others */
  p = &buffer[FRAME_WIDTH * 2];
  for (unsigned i = 1; i < FRAME_HEIGHT; ++i) {
    memcpy(p, buffer, FRAME_WIDTH * 2);
    p += FRAME_WIDTH * 2;
  }
}
#endif

void video_task(void)
{
  static unsigned start_ms = 0;
  static unsigned already_sent = 0;

  if (!tud_video_n_streaming(0, 0)) {
    already_sent  = 0;
    frame_num     = 0;
    return;
  }

  if (!already_sent) {
    already_sent = 1;
    start_ms = board_millis();
#ifdef CFG_EXAMPLE_VIDEO_READONLY
    tud_video_n_frame_xfer(0, 0, (void*)&frame_buffer[(frame_num % (FRAME_WIDTH / 2)) * 4],
                           FRAME_WIDTH * FRAME_HEIGHT * 16/8);
#else
    fill_color_bar(frame_buffer, frame_num);
    tud_video_n_frame_xfer(0, 0, (void*)frame_buffer, FRAME_WIDTH * FRAME_HEIGHT * 16/8);
#endif
  }

  unsigned cur = board_millis();
  if (cur - start_ms < interval_ms) return; // not enough time
  if (tx_busy) return;
  start_ms += interval_ms;

#ifdef CFG_EXAMPLE_VIDEO_READONLY
  tud_video_n_frame_xfer(0, 0, (void*)&frame_buffer[(frame_num % (FRAME_WIDTH / 2)) * 4],
                         FRAME_WIDTH * FRAME_HEIGHT * 16/8);
#else
  fill_color_bar(frame_buffer, frame_num);
  tud_video_n_frame_xfer(0, 0, (void*)frame_buffer, FRAME_WIDTH * FRAME_HEIGHT * 16/8);
#endif
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
  (void)ctl_idx; (void)stm_idx;
  tx_busy = 0;
  /* flip buffer */
  ++frame_num;
}

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
			video_probe_and_commit_control_t const *parameters)
{
  (void)ctl_idx; (void)stm_idx;
  /* convert unit to ms from 100 ns */
  interval_ms = parameters->dwFrameInterval / 10000;
  return VIDEO_ERROR_NONE;
}
//--------------------------------------------------------------------+
// FLASH TASK
//--------------------------------------------------------------------+
// We're going to erase and reprogram a region 256k from the start of flash.
// Once done, we can access this at XIP_BASE + 256k.
#define FLASH_TARGET_OFFSET (256 * 1024)

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

void print_buf(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15)
            printf("\n");
        else
            printf(" ");
    }
}
void flash(void) {
    const bool program = true; // enable programming (may resualt in data loss)
    stdio_init_all();
    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_put(LED2_PIN, 0);
    sleep_ms(250);
    gpio_put(LED2_PIN, 1);
    if(program){
      uint8_t data[FLASH_PAGE_SIZE] = "bitten2up, made by Scarlet: She/her"

      printf("Generated random data:\n");
      print_buf(data, FLASH_PAGE_SIZE);
    // Note that a whole number of sectors must be erased at a time.
      printf("\nErasing target region...\n");
      flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
      gpio_put(LED2_PIN, 0);
      sleep_ms(250);
      gpio_put(LED2_PIN, 1);
      sleep_ms(250);
      gpio_put(LED2_PIN, 0)
      printf("Done. Read back target region:\n");
      print_buf(flash_target_contents, FLASH_PAGE_SIZE);

      printf("\nProgramming target region...\n");
      flash_range_program(FLASH_TARGET_OFFSET,  data, FLASH_PAGE_SIZE);
    
      printf("Done. Read back target region:\n");
      print_buf(flash_target_contents, FLASH_PAGE_SIZE);

      bool mismatch = false;
      for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
          if (data[i] != flash_target_contents[i])
              mismatch = true;
      }
      if (mismatch)
          printf("Programming failed!\n");
          while (1){
            gpio_put(LED2_PIN, 0);
            sleep_ms(10);
            gpio_put(LED2_PIN, 1);
          }
      else
          printf("Programming successful!\n");
          while (1){
            gpio_put(LED2_PIN, 0);
            sleep_ms(1000);
            gpio_put(LED2_PIN, 1);
          }
    }
}
//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}