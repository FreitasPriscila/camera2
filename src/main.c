#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/regs/dreq.h"
#include "camera_capture.pio.h"

/********** DEFINIÇÕES DE PINOS (ajuste aqui se necessário) **********/
#define OV_SDA   0
#define OV_SCL   1
#define OV_PCLK  2  // DCLK
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

/********** CONFIGURAÇÕES DA CÂMARA **********/
#define I2C_PORT i2c0
#define I2C_ADDR 0x30
#define FRAME_WIDTH  320
#define FRAME_HEIGHT 240
#define BYTES_PER_PIXEL 2

#define LINE_BYTES (FRAME_WIDTH * BYTES_PER_PIXEL)
static uint8_t linebuf_a[LINE_BYTES];

// --- Funções da câmara (sem alterações) ---
static inline void sccb_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, I2C_ADDR, buf, 2, false);
}
static void ov2640_reset_pins(void) {
    gpio_init(OV_RST); gpio_set_dir(OV_RST, GPIO_OUT); gpio_put(OV_RST, 0); sleep_ms(10); gpio_put(OV_RST, 1); sleep_ms(10);
    gpio_init(OV_PWDN); gpio_set_dir(OV_PWDN, GPIO_OUT); gpio_put(OV_PWDN, 0);
}
static void ov2640_init_basic(void) {
    sccb_write(0x12, 0x80); sleep_ms(50);
    sccb_write(0xFF, 0x00); sccb_write(0xD5, 0x03); sccb_write(0xDA, 0x10); sccb_write(0xC2, 0x0C);
    sccb_write(0xFF, 0x01); sccb_write(0x12, 0x40); sccb_write(0x17, 0x11); sccb_write(0x18, 0x75); sccb_write(0x19, 0x01); sccb_write(0x1A, 0x97); sccb_write(0x32, 0x36);
    sccb_write(0xFF, 0x00); sccb_write(0xD3, 0x04); sccb_write(0xDA, 0x10);
    sccb_write(0xE0, 0x04); sccb_write(0xE1, 0x67); sccb_write(0xE5, 0x1F); sccb_write(0xE0, 0x00);
    sleep_ms(100);
}
static void i2c_init_cam(void) {
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(OV_SDA, GPIO_FUNC_I2C); gpio_set_function(OV_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OV_SDA); gpio_pull_up(OV_SCL);
}
typedef struct { PIO pio; uint sm; } cam_cap_t;
static void cam_pio_init(cam_cap_t *cc) {
    cc->pio = pio0;
    uint offset = pio_add_program(cc->pio, &cam_cap_program);
    cc->sm = pio_claim_unused_sm(cc->pio, true);
    cam_cap_program_init(cc->pio, cc->sm, offset, OV_D0, OV_PCLK, OV_HREF, OV_VSYNC);
}
static void send_header(uint16_t w, uint16_t h, uint32_t size) {
    uint8_t hdr[12];
    hdr[0]=0xAA; hdr[1]=0x55; hdr[2]='R'; hdr[3]='5';
    hdr[4]=w & 0xFF; hdr[5]=w>>8; hdr[6]=h & 0xFF; hdr[7]=h>>8;
    hdr[8]=size & 0xFF; hdr[9]=(size>>8)&0xFF; hdr[10]=(size>>16)&0xFF; hdr[11]=(size>>24)&0xFF;
    for (int i=0;i<12;i++) putchar_raw(hdr[i]);
}
static void send_bytes(const uint8_t *buf, size_t n) {
    for (size_t i=0;i<n;i++) putchar_raw(buf[i]);
}

int main() {
    // Initialize USB serial and wait for connection
    stdio_init_all();
    stdio_usb_init();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000); // Extra delay after connection
    printf("--- INICIANDO DIAGNÓSTICO DA CÂMERA ---\n");

    // Configurar pinos de controlo
    gpio_init(OV_VSYNC); gpio_set_dir(OV_VSYNC, GPIO_IN);
    printf("1. Pino VSYNC configurado.\n");

    i2c_init_cam();
    printf("2. I2C inicializado.\n");
    
    ov2640_reset_pins();
    ov2640_init_basic();
    printf("3. Configuração da câmera (OV2640) enviada via I2C.\n");

    cam_cap_t CC = {0};
    cam_pio_init(&CC);
    printf("4. PIO para captura de imagem inicializado.\n");

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dcfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dcfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dcfg, false);
    channel_config_set_write_increment(&dcfg, true);
    channel_config_set_dreq(&dcfg, pio_get_dreq(CC.pio, CC.sm, false));
    dma_channel_configure(dma_chan, &dcfg, NULL, &CC.pio->rxf[CC.sm], LINE_BYTES, false);
    printf("5. DMA configurado.\n");

    printf("6. Entrando no loop principal. Aguardando VSYNC...\n\n");

    while (true) {
        // Espera o início do quadro (VSYNC)
        while (gpio_get(OV_VSYNC));
        while (!gpio_get(OV_VSYNC));

        // Se o código chegar aqui, deve começar a enviar dados
        send_header(FRAME_WIDTH, FRAME_HEIGHT, (uint32_t)FRAME_WIDTH*FRAME_HEIGHT*2);

        for (uint16_t y = 0; y < FRAME_HEIGHT; ++y) {
            dma_channel_set_write_addr(dma_chan, linebuf_a, true);
            dma_channel_wait_for_finish_blocking(dma_chan);
            send_bytes(linebuf_a, LINE_BYTES);
        }
    }
    return 0;
}

