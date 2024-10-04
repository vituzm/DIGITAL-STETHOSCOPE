"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""
import serial
import wave 

# Configurações da serial
SERIAL_PORT = 'COM3'  # Porta serial
BAUD_RATE = 230400    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono
SAMPLE_RATE = 8000   # Taxa de amostragem em Hz
RECORD_SECONDS = 10   # Tempo de gravação
input_wave_name  = "C:\\Users\\Vitor\\Downloads\\pulmaoaa.wav"  # Nome do arquivo de saída
output_wave_name = "C:\\Users\\Vitor\\Downloads\\pulmaoaa FILTRADO.wav"
TOTALSAMPLES = SAMPLE_RATE * RECORD_SECONDS # Número total de amostras a serem gravadas

# Criar e configurar o arquivo WAV
waveform = wave.open(input_wave_name, 'wb')
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

print(f"Arquivo {input_wave_name} criado com sucesso.")
