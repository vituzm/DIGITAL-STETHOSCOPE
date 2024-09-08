"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""

import wave 
import serial
import numpy as np
import cmsisdsp as dsp
import matplotlib.pyplot as plt
from scipy import signal 

# Configurações da serial
SERIAL_PORT = 'COM3'  # Porta serial
BAUD_RATE = 460800    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono
SAMPLE_RATE = 15000   # Taxa de amostragem em Hz
RECORD_SECONDS = 5   # Tempo de gravação
input_wave_name  = "C:\\Users\\Vitor\\Downloads\\20segundos S1S2.wav"  # Nome do arquivo de saída
output_wave_name = "C:\\Users\\Vitor\\Downloads\\20segundos S1S2 FILTRADO.wav"


# Número total de amostras a serem gravadas

TOTALSAMPLES = SAMPLE_RATE * RECORD_SECONDS
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

# Frequências para o filtro passa-faixa
w = [15, 250]

# Secoes byquad do filtro Butterworth passa-faixa
secoes = 10

# Atenuação na banda de rejeição (em dB)
attenuation_stopband = 150 

# Projeto do filtro passa-faixa
sos = signal.iirfilter(
    N = secoes, 
    Wn = w, 
    btype ='band', 
    ftype ='cheby2', 
    rs = attenuation_stopband,
    fs = SAMPLE_RATE, 
    output='sos')

# Coeficientes b1, b2 ... do filtro passa-faixa
NUMERADOR = np.array([
    [0.3189842366757, 0, 0],
    [1, -1.999961380843, 1],
    [0.3189842366757, 0, 0],
    [1, -1.988800736477, 1],
    [0.28928576234, 0, 0],
    [1, -1.999967906269, 1],
    [0.28928576234, 0, 0],
    [1, -1.986531302625, 1],
    [0.2324974610043, 0, 0],
    [1, -1.999979060849, 1],
    [0.2324974610043, 0, 0],
    [1, -1.97939324409, 1],
    [0.1517644269888, 0, 0],
    [1, -1.999991041153, 1],
    [0.1517644269888, 0, 0],
    [1, -1.952166055553, 1],
    [0.05497448138232, 0, 0],
    [1, -1.99999891036, 1],
    [0.05497448138232, 0, 0],
    [1, -1.637980853257, 1],
    [1, 0, 0]
])

# Coeficientes a1, a2 ... do filtro passa-faixa
DENOMINADOR = np.array([
    [1, 0, 0],
    [1, -1.990727776199, 0.9928297437502],
    [1, 0, 0],
    [1, -1.997550834792, 0.9977560877147],
    [1, 0, 0],
    [1, -1.977271268252, 0.9792374469158],
    [1, 0, 0],
    [1, -1.992852566386, 0.9930699927101],
    [1, 0, 0],
    [1, -1.966276616914, 0.9679801879131],
    [1, 0, 0],
    [1, -1.987449619096, 0.9876984761149],
    [1, 0, 0],
    [1, -1.960203939386, 0.9615195445052],
    [1, 0, 0],
    [1, -1.980602269125, 0.9809223855052],
    [1, 0, 0],
    [1, -1.971401883989, 0.9718926701358],
    [1, 0, 0],
    [1, -1.962134367413, 0.9629892655753],
    [1, 0, 0]
])
# Montar o array SOS com os coeficientes 
sos = np.hstack([NUMERADOR, DENOMINADOR])

# Ler o arquivo WAV original
with wave.open(input_wave_name, 'rb') as wave_in:
    channels = 1
    sample_width = 2
    framerate = SAMPLE_RATE
    num_frames = wave_in.getnframes()

    audio_data = wave_in.readframes(num_frames)
    audio_int = (np.frombuffer(audio_data, dtype=np.int16))*17

    filtered_samples = signal.sosfilt(sos, audio_int)

    # Normalizar os dados filtrados para 16 bits
    filtered_samples = np.clip(filtered_samples, -32768, 32767)  # Limitar valores para 16-bit PCM
    filtered_samples = filtered_samples.astype(np.int16)
    

# Salvar o áudio filtrado em um novo arquivo WAV
with wave.open(output_wave_name, 'wb') as wave_out:
    wave_out.setnchannels(channels)
    wave_out.setsampwidth(sample_width)
    wave_out.setframerate(framerate)
    wave_out.writeframes(filtered_samples.tobytes())
    print(f"Arquivo filtrado salvo como {output_wave_name}")

