
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/regs/dreq.h"
#include "camera_capture.pio.h"

/********** PIN MAP (adjust here if needed) **********/
#define OV_SDA   0
#define OV_SCL   1
#define OV_PCLK  2   // DCLK
#define OV_PWDN  3
#define OV_RST   4
#define OV_HREF  5
#define OV_VSYNC 6

#define OV_D0    8
#define OV_D1    9
#define OV_D2   10
#define OV_D3   11
#define OV_D4   12
#define OV_D5   13
#define OV_D6   14
#define OV_D7   15

/********** CAMERA SETTINGS **********/
#define I2C_PORT i2c0
#define I2C_ADDR 0x30  // 7-bit SCCB address (0x60>>1)

// Choose resolution (frame size) that streams over USB FS.
// Start with VGA (640x480). USB 12Mbps can handle ~1-2 FPS RGB565.
static const uint16_t FRAME_WIDTH  = 640;
static const uint16_t FRAME_HEIGHT = 480;

// OV2640 will be set to RGB565; camera outputs 2 bytes per pixel on 8-bit bus.
// We stream as raw bytes (little-endian as delivered by the sensor).

/********** SIMPLE HOST PROTOCOL **********
Each frame:
  0xAA 0x55 'R' '5'  (magic 4 bytes)
  uint16 width_le
  uint16 height_le
  uint32 payload_len_le (= width*height*2)
  payload bytes...
*******************************************/

static inline void sccb_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, I2C_ADDR, buf, 2, false);
}

static void ov2640_reset_pins(void) {
    gpio_init(OV_RST);
    gpio_set_dir(OV_RST, GPIO_OUT);
    gpio_put(OV_RST, 0);
    sleep_ms(10);
    gpio_put(OV_RST, 1);
    sleep_ms(10);

    gpio_init(OV_PWDN);
    gpio_set_dir(OV_PWDN, GPIO_OUT);
    gpio_put(OV_PWDN, 0); // power up
}

static void ov2640_init_basic(void) {
    // soft reset via SCCB
    sccb_write(0x12, 0x80);
    sleep_ms(50);

    // Select DSP bank
    sccb_write(0xFF, 0x00);
    // Turn on all clocks
    sccb_write(0xD5, 0x03);

    // Output format RGB565
    sccb_write(0xDA, 0x10); // set to RGB565
    sccb_write(0xC2, 0x0C); // enable downsampling, clock divider

    // Set resolution windowing for VGA (640x480)
    // This is a minimal config known to work on many OV2640 modules.
    // For other sizes, more registers are normally changed; we keep it concise.
    sccb_write(0xFF, 0x01); // sensor bank
    sccb_write(0x12, 0x40); // SVGA mode
    sccb_write(0x17, 0x11); // HSTART
    sccb_write(0x18, 0x75); // HSTOP
    sccb_write(0x19, 0x01); // VSTART
    sccb_write(0x1A, 0x97); // VSTOP
    sccb_write(0x32, 0x36); // HREF

    // Back to DSP bank
    sccb_write(0xFF, 0x00);
    // DVP PCLK divider (try to keep under ~12-15MHz for PIO)
    sccb_write(0xD3, 0x04); // clock divider (higher value => slower PCLK)

    // Color matrix / gamma defaults are OK
    // Enable RGB565
    sccb_write(0xDA, 0x10);

    // Disable JPEG (raw RGB)
    sccb_write(0xE0, 0x04); // reset DSP
    sccb_write(0xE1, 0x67);
    sccb_write(0xE5, 0x1F);
    sccb_write(0xE0, 0x00); // end reset

    sleep_ms(100);
}

static void i2c_init_cam(void) {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(OV_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OV_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OV_SDA);
    gpio_pull_up(OV_SCL);
}

typedef struct {
    PIO pio;
    uint sm;
    int dma_chan;
} cam_cap_t;

static void cam_pio_init(cam_cap_t *cc) {
    cc->pio = pio0;
    uint offset = pio_add_program(cc->pio, &cam_cap_program);
    cc->sm = pio_claim_unused_sm(cc->pio, true);

    // Configure IN pins base to D0
    pio_sm_config c = cam_cap_program_get_default_config(offset);
    sm_config_set_in_pins(&c, OV_D0);           // IN base D0..D7
    sm_config_set_jmp_pin(&c, OV_HREF);         // JMP PIN uses HREF
    sm_config_set_clkdiv(&c, 1.0f);             // run at sysclk

    // Shift: autopush each 8 bits
    sm_config_set_in_shift(&c, true, true, 8);

    // Map PCLK to 'clk' pin (used in WAIT)
    pio_gpio_init(cc->pio, OV_PCLK);
    sm_config_set_clk_pins(&c, OV_PCLK);

    // Map VSYNC as a 'pin' for WAIT (PIO can wait on any GPIO # via exec ctrl)
    // We'll set pin dirs and init for data pins and control pins
    for (int pin = OV_D0; pin <= OV_D7; ++pin) pio_gpio_init(cc->pio, pin);
    pio_gpio_init(cc->pio, OV_HREF);
    pio_gpio_init(cc->pio, OV_VSYNC);

    // Set all those as inputs
    pio_sm_set_consecutive_pindirs(cc->pio, cc->sm, OV_D0, 8, false);
    pio_sm_set_consecutive_pindirs(cc->pio, cc->sm, OV_HREF, 1, false);
    pio_sm_set_consecutive_pindirs(cc->pio, cc->sm, OV_VSYNC, 1, false);
    pio_sm_set_consecutive_pindirs(cc->pio, cc->sm, OV_PCLK, 1, false);

    pio_sm_init(cc->pio, cc->sm, offset, &c);
    pio_sm_set_enabled(cc->pio, cc->sm, true);
}

#define LINE_BYTES (FRAME_WIDTH * 2)   // RGB565

// Two line buffers for ping-pong DMA
static uint8_t linebuf_a[LINE_BYTES];
static uint8_t linebuf_b[LINE_BYTES];

static void send_header(uint16_t w, uint16_t h, uint32_t size) {
    uint8_t hdr[12];
    hdr[0]=0xAA; hdr[1]=0x55; hdr[2]='R'; hdr[3]='5';
    hdr[4]=w & 0xFF; hdr[5]=w>>8;
    hdr[6]=h & 0xFF; hdr[7]=h>>8;
    hdr[8]= size & 0xFF;
    hdr[9]= (size>>8) & 0xFF;
    hdr[10]=(size>>16)& 0xFF;
    hdr[11]=(size>>24)& 0xFF;
    // use stdio_usb raw write
    for (int i=0;i<12;i++) putchar_raw(hdr[i]);
}

static void send_bytes(const uint8_t *buf, size_t n) {
    for (size_t i=0;i<n;i++) putchar_raw(buf[i]);
}

static void wait_vsync_low(void) {
    // Wait for VSYNC high -> low (frame start)
    while (gpio_get(OV_VSYNC)) tight_loop_contents();
    while (!gpio_get(OV_VSYNC)) break; // already low
}

int main() {
    stdio_init_all();
    sleep_ms(1500); // time for USB to enumerate

    // Init control pins
    gpio_init(OV_VSYNC); gpio_set_dir(OV_VSYNC, GPIO_IN);
    gpio_init(OV_HREF);  gpio_set_dir(OV_HREF,  GPIO_IN);
    gpio_init(OV_PCLK);  gpio_set_dir(OV_PCLK,  GPIO_IN);
    for (int pin=OV_D0; pin<=OV_D7; ++pin) { gpio_init(pin); gpio_set_dir(pin, GPIO_IN); }

    i2c_init_cam();
    ov2640_reset_pins();
    ov2640_init_basic();

    // Setup PIO to capture
    cam_cap_t CC = {0};
    cam_pio_init(&CC);

    // DMA to pull from PIO RX FIFO into line buffers
    int dma_a = dma_claim_unused_channel(true);
    int dma_b = dma_claim_unused_channel(true);
    dma_channel_config dcfg = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&dcfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dcfg, false);
    channel_config_set_write_increment(&dcfg, true);
    channel_config_set_dreq(&dcfg, pio_get_dreq(CC.pio, CC.sm, false)); // RX
    dma_channel_configure(dma_a, &dcfg,
        linebuf_a,                         // dst
        &cc.pio->rxf[CC.sm],               // src
        LINE_BYTES,                        // transfers
        false);

    dma_channel_config dcfg2 = dcfg;
    dma_channel_configure(dma_b, &dcfg2, linebuf_b, &cc.pio->rxf[CC.sm], LINE_BYTES, false);

    printf("OV2640 stream RGB565 %dx%d via USB CDC. Magic AA55'R5'.\n", FRAME_WIDTH, FRAME_HEIGHT);

    while (true) {
        // Wait frame start
        while (gpio_get(OV_VSYNC)) tight_loop_contents();
        while (!gpio_get(OV_VSYNC)) break; // now in frame

        send_header(FRAME_WIDTH, FRAME_HEIGHT, (uint32_t)FRAME_WIDTH*FRAME_HEIGHT*2);

        // For each line: start DMA A then B alternately while HREF toggles.
        for (uint16_t y = 0; y < FRAME_HEIGHT; ++y) {
            // Wait for HREF high (line active)
            while (!gpio_get(OV_HREF)) tight_loop_contents();

            // Start A
            dma_channel_set_trans_count(dma_a, LINE_BYTES, false);
            dma_start_channel_mask(1u << dma_a);

            // Wait DMA A done (one full line)
            dma_channel_wait_for_finish_blocking(dma_a);

            // Push A to USB
            send_bytes(linebuf_a, LINE_BYTES);

            // Wait HREF low (line end) to resync
            while (gpio_get(OV_HREF)) tight_loop_contents();
        }
    }
    return 0;
}
