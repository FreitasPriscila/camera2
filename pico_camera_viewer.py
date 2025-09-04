import serial
import numpy as np
from PIL import Image
import struct

# --- CONFIGURE AQUI ---
# Altere 'COM3' para a porta serial correta da sua Raspberry Pi Pico
# No Windows: 'COM3', 'COM4', etc.
# No Linux: '/dev/ttyACM0', etc.
# No macOS: '/dev/cu.usbmodem...', etc.
PORT = 'COM6' 
BAUDRATE = 115200 # A taxa de bauds não importa para USB CDC, mas é necessária
# --- FIM DA CONFIGURAÇÃO ---

# Definições que devem ser idênticas às do firmware da Pico
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
BYTES_PER_PIXEL = 2
FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL

# O "cabeçalho mágico" que o firmware envia antes de cada quadro
MAGIC_HEADER = b'\xAA\x55\x52\x35'

def main():
    try:
        # Abre a conexão serial
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Conectado a {PORT}. Aguardando dados da câmera...")
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial {PORT}: {e}")
        print("Verifique se a porta está correta e se a Pico está conectada.")
        return

    while True:
        # 1. Procurar pelo cabeçalho mágico
        buffer = b''
        while True:
            byte = ser.read(1)
            if not byte:
                print("Timeout: Nenhum dado recebido. A Pico está enviando imagens?")
                continue
            buffer += byte
            if len(buffer) > len(MAGIC_HEADER):
                buffer = buffer[1:] # Mantém o buffer do tamanho do cabeçalho
            
            if buffer == MAGIC_HEADER:
                print("Cabeçalho encontrado! Recebendo quadro...")
                break
        
        # 2. Ler as dimensões e o tamanho do payload (12 bytes de cabeçalho total)
        # 4 bytes de magia + 2 de largura + 2 de altura + 4 de tamanho
        header_payload = ser.read(8) 
        if len(header_payload) < 8:
            print("Erro: Cabeçalho incompleto recebido.")
            continue
            
        # Desempacota os dados usando struct (formato little-endian)
        width, height, payload_len = struct.unpack('<HHI', header_payload)
        
        print(f"Metadados do quadro: {width}x{height}, Tamanho: {payload_len} bytes")
        
        # Validação simples
        if width != FRAME_WIDTH or height != FRAME_HEIGHT or payload_len != FRAME_SIZE:
            print("Erro: Incompatibilidade de metadados! Verifique as definições no script e no firmware.")
            continue # Volta a procurar por um novo cabeçalho

        # 3. Ler os bytes da imagem
        image_data = ser.read(payload_len)
        if len(image_data) < payload_len:
            print(f"Erro: Quadro incompleto. Esperado {payload_len}, recebido {len(image_data)}")
            continue

        # 4. Converter os bytes para uma imagem e exibir
        try:
            # Interpreta os dados como um array numpy no formato RGB565 big-endian
            # O OV2640 envia os bytes em ordem big-endian para RGB565
            img_array = np.frombuffer(image_data, dtype=np.uint16).reshape((height, width))
            
            # Converte de numpy para uma imagem do Pillow no modo correto
            # O modo 'RGB;16-BE' diz ao Pillow que são dados RGB, 16 bits por pixel, Big-Endian
            img = Image.fromarray(img_array, mode='RGB;16-BE')

            # Mostra a imagem
            img.show()
            
            print("Quadro exibido com sucesso!")

        except Exception as e:
            print(f"Erro ao processar a imagem: {e}")

if __name__ == '__main__':
    main()
