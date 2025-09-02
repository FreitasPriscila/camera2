
OV2640 -> Raspberry Pi Pico W (USB RAW RGB565)

Resumo
------
Captura vídeo da câmera OV2640 via barramento paralelo de 8 bits usando PIO do RP2040
e envia cada frame (RGB565) para o computador pela USB (CDC). Resolução default: 640x480.

Pinos (ajuste em src/main.c se necessário)
------------------------------------------
SDA = GP0
SCL = GP1
PCLK(DCLK) = GP2
PWDN = GP3
RST = GP4
HREF = GP5
VSYNC = GP6
D0..D7 = GP8..GP15

Como compilar
-------------
1) Instale o Pico SDK.
2) Configure o CMake com `-DPICO_SDK_PATH=/caminho/para/pico-sdk`
3) `mkdir build && cd build`
4) `cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk`
5) `make`
6) Grave o UF2 no Pico W.

Como receber no PC
------------------
Qualquer programa que leia da porta serial USB e grave em arquivo.
O fluxo de cada frame é:
  magic: 0xAA 0x55 'R' '5'
  width (uint16 LE)
  height (uint16 LE)
  payload_len (uint32 LE) = width*height*2
  payload (bytes RGB565)

Exemplo Linux:
  stty -F /dev/ttyACM0 raw -echo -onlcr 115200
  (a taxa não importa para USB CDC, mas mantém compatibilidade)
  cat /dev/ttyACM0 > frames.raw

Depois separe os frames lendo os cabeçalhos.

Observação
---------
USB FS tem ~1.2 MB/s útil. Em 640x480 RGB565 (~614,400 bytes/frame),
espere ~1-2 FPS.
