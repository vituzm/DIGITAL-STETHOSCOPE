"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""

import wave 
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
RECORD_SECONDS = 15   # Tempo de gravação
input_wave_name  = "C:\\Users\\Vitor\\Downloads\\pulmao.wav"  # Nome do arquivo de saída
output_wave_name = "C:\\Users\\Vitor\\Downloads\\pulmao FILTRADO.wav"
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


# Frequências para o filtro passa-faixa
w = [80, 800]

# Secoes byquad do filtro Butterworth passa-faixa
secoes = 18

# Atenuação na banda de rejeição (em dB)
attenuation_stopband = 80

# Projeto do filtro passa-faixa
sos = signal.iirfilter(
    N = secoes, 
    Wn = w, 
    btype ='band', 
    ftype ='cheby2', 
    rs = attenuation_stopband,
    fs = SAMPLE_RATE, 
    output='sos')

# Coeficientes b1, b2 ... do filtro passa-faixa (NUM)
NUM = np.array([
    [0.8624198568921, 0, 0],
    [1, -1.998670826668, 1],
    [0.8624198568921, 0, 0],
    [1, -1.888086620568, 1],
    [0.8460114408911, 0, 0],
    [1, -1.998735632308, 1],
    [0.8460114408911, 0, 0],
    [1, -1.882517082897, 1],
    [0.82103877482, 0, 0],
    [1, -1.99886147684, 1],
    [0.82103877482, 0, 0],
    [1, -1.86994955622, 1],
    [0.7841933038325, 0, 0],
    [1, -1.99904005788, 1],
    [0.7841933038325, 0, 0],
    [1, -1.846676605314, 1],
    [0.7302088800371, 0, 0],
    [1, -1.999257176309, 1],
    [0.7302088800371, 0, 0],
    [1, -1.804047177023, 1],
    [0.651403122932, 0, 0],
    [1, -1.999491321984, 1],
    [0.651403122932, 0, 0],
    [1, -1.720144621802, 1],
    [0.5386986683487, 0, 0],
    [1, -1.999713389608, 1],
    [0.5386986683487, 0, 0],
    [1, -1.528828001397, 1],
    [0.3889954546249, 0, 0],
    [1, -1.999889347166, 1],
    [0.3889954546249, 0, 0],
    [1, -0.9720823575347, 1],
    [0.2369655318849, 0, 0],
    [1, -1.999987250873, 1],
    [0.2369655318849, 0, 0],
    [1, 1.000464398742, 1],
    [1, 0, 0]
])

# Coeficientes do denominador (DEN)
DEN = np.array([
    [1, 0, 0],
    [1, -1.891760963501, 0.9804499558419],
    [1, 0, 0],
    [1, -1.995609509674, 0.9972774801027],
    [1, 0, 0],
    [1, -1.850802901145, 0.9402862726036],
    [1, 0, 0],
    [1, -1.990166632035, 0.9917807275078],
    [1, 0, 0],
    [1, -1.802227392722, 0.8950044447261],
    [1, 0, 0],
    [1, -1.98458645157, 0.9861004094676],
    [1, 0, 0],
    [1, -1.741039709033, 0.8397668069517],
    [1, 0, 0],
    [1, -1.978751776626, 0.9801255926472],
    [1, 0, 0],
    [1, -1.661550732935, 0.7691472101068],
    [1, 0, 0],
    [1, -1.972605452494, 0.9738093768699],
    [1, 0, 0],
    [1, -1.966221053753, 0.9672406277127],
    [1, 0, 0],
    [1, -1.558794546743, 0.6782816849278],
    [1, 0, 0],
    [1, -1.959933615913, 0.9607756072767],
    [1, 0, 0],
    [1, -1.43380333782, 0.5675188073371],
    [1, 0, 0],
    [1, -1.954521188554, 0.9552191991802],
    [1, 0, 0],
    [1, -1.304960680982, 0.452751288454],
    [1, 0, 0],
    [1, -1.951240879494, 0.9518569463744],
    [1, 0, 0],
    [1, -1.216773371593, 0.3738062390593],
    [1, 0, 0]
])
# Montar o array SOS com os coeficientes 
sos = np.hstack([NUM, DEN])

# Ler o arquivo WAV original
with wave.open(input_wave_name, 'rb') as wave_in:
    channels = 1
    sample_width = 2
    framerate = SAMPLE_RATE
    num_frames = wave_in.getnframes()

    audio_data = wave_in.readframes(num_frames)
    audio_int = (np.frombuffer(audio_data, dtype=np.int16))*50

    filtered_samples = signal.sosfilt(sos, audio_int)

    # Normalizar os dados filtrados para 16 bits
    filtered_samples = np.clip(filtered_samples, -32768, 32767)  # Limitar valores para 16-bit PCM
    filtered_samples = filtered_samples.astype(np.int16)

wave_in.close(input_wave_name)

# Salvar o áudio filtrado em um novo arquivo WAV
with wave.open(output_wave_name, 'wb') as wave_out:
    wave_out.setnchannels(channels)
    wave_out.setsampwidth(sample_width)
    wave_out.setframerate(framerate)
    wave_out.writeframes(filtered_samples.tobytes())
    print(f"Arquivo filtrado salvo como {output_wave_name}")

wave_out.close()