"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""

import wave
import serial

# Configurações da serial
SERIAL_PORT = 'COM6'  # Porta serial
BAUD_RATE = 921200    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono
SAMPLE_RATE = 19940   # Taxa de amostragem em Hz
RECORD_SECONDS = 2    # Tempo de gravação
WAVE_NAME = "C:\\Users\\Vitor\\Downloads\\AudioEstetoscópio.wav"  # Nome do arquivo de saída

# Número total de amostras a serem gravadas

TOTALSAMPLES = SAMPLE_RATE * RECORD_SECONDS
# Criar e configurar o arquivo WAV
waveform = wave.open(WAVE_NAME, 'wb')
waveform.setnchannels(CHANNELS)
waveform.setsampwidth(2)  # Tamanho do sample em bytes (16 bits = 2 bytes)
waveform.setframerate(SAMPLE_RATE)

try:
    # Abrir a porta serial com timeout
    port = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print("...Iniciando captura de dados da serial...")

    # Ler e gravar dados
    for i in range(TOTALSAMPLES):
        valor_serial = port.read(CHUNK)  # Ler o tamanho de uma amostra completa
        waveform.writeframes(valor_serial)  # Gravar os dados no arquivo WAV
    print("___Captura e gravação finalizadas___")

finally:
    port.close()
    waveform.close()


print(f"Arquivo {WAVE_NAME} criado com sucesso.")
