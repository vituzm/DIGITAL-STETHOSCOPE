"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""

import wave
import serial
import numpy as np
import cmsisdsp as dsp
import matplotlib.pyplot as plt

# Configurações da serial
SERIAL_PORT = 'COM3'  # Porta serial
BAUD_RATE = 460800    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono
SAMPLE_RATE = 15000   # Taxa de amostragem em Hz
RECORD_SECONDS = 20   # Tempo de gravação
input_wave_name  = "C:\\Users\\Vitor\\Downloads\\10segundos S1S2.wav"  # Nome do arquivo de saída
output_wave_name = "C:\\Users\\Vitor\\Downloads\\10segundos S1S2 FILTRADO.wav"

"""
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
"""

# Inicializar a instância do filtro
S = dsp.arm_biquad_cascade_df2T_instance_f32()

pCoeffs = np.array([
    0.09984020717417, 0, 0, -1.898254411361, 0.9405508494265,  # Primeiro estágio
    0.09984020717417, 0, 0, -1.998105787452, 0.9981453321343,  # Segundo estágio
    0.09705588985272, 0, 0, -1.797510332218, 0.83676024162,    # Terceiro estágio
    0.09705588985272, 0, 0, -1.994481714277, 0.9945219846887,  # Quarto estágio
    0.09486224171169, 0, 0, -1.991145688869, 0.9911873270674,  # Quinto estágio
    0.09486224171169, 0, 0, -1.720697906556, 0.7569619126399,  # Sexto estágio
    0.0933586107854,  0, 0, -1.988438107666, 0.9884813513232,  # Sétimo estágio
    0.0933586107854,  0, 0, -1.669526287086, 0.7033457954234,  # Oitavo estágio
    0.09259624200386, 0, 0, -1.986861912185, 0.9869062955672,  # Nono estágio
    0.09259624200386, 0, 0, -1.644113607758, 0.6765287005334   # Décimo estágio
], dtype=np.float32)

# Número de estágios biquad
numStages = 10

# Estado do filtro (inicializado com zeros)
state = np.zeros(2 * numStages, dtype=np.float32)

dsp.arm_biquad_cascade_df2T_init_f32(S, numStages, pCoeffs, state)

# Ler o arquivo WAV original
with wave.open(input_wave_name, 'rb') as wave_in:
    channels = wave_in.getnchannels()
    sample_width = wave_in.getsampwidth()
    framerate = wave_in.getframerate()
    num_frames = wave_in.getnframes()
    
    audio_data = wave_in.readframes(num_frames)
    audio_samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
    
    print(num_frames)
    
    # Aplicar o filtro IIR
    filtered_samples = dsp.arm_biquad_cascade_df2T_f32(S, audio_samples)
    plt.figure()
    plt.plot(audio_samples[1:500000])
    plt.show() 
    
    # Normalizar o sinal filtrado para o intervalo de int16
    filtered_samples = np.clip(filtered_samples, -32768, 32767)

    # Converter de volta para int16
    filtered_samples_int16 = filtered_samples.astype(np.int16)
    
    
# Salvar o áudio filtrado em um novo arquivo WAV
with wave.open(output_wave_name, 'wb') as wave_out:
    wave_out.setnchannels(channels)
    wave_out.setsampwidth(sample_width)
    wave_out.setframerate(framerate)
    wave_out.writeframes(filtered_samples_int16.tobytes())

print(f"Arquivo filtrado salvo como {output_wave_name}")