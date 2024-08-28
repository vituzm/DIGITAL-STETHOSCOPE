"""
Programa para a criação do arquivo wave de acordo com o 
tempo desejado
"""

import wave
import serial
import numpy as np
from cmsisdsp import arm_biquad_cascade_df2T_f32, arm_biquad_cascade_df2T_init_f32


# Configurações da serial
SERIAL_PORT = 'COM7'  # Porta serial
BAUD_RATE = 460800    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono
SAMPLE_RATE = 15000   # Taxa de amostragem em Hz
RECORD_SECONDS = 20   # Tempo de gravação
input_wave_name  = "G:\\TCC\\Audios gravados\\Gravações P4 tricúspide\\10segundos S1S2.wav"  # Nome do arquivo de saída

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

# Número de estágios biquad
numStages = 10

# Estado do filtro (inicializado com zeros)
state = np.zeros(4 * numStages, dtype=np.float32)

# Estrutura do filtro
class arm_biquad_casd_df2T_instance_f32:
    def __init__(self, numStages, pCoeffs, pState):
        self.numStages = numStages
        self.pCoeffs = pCoeffs
        self.pState = pState


# Inicializar a instância do filtro
S = arm_biquad_casd_df2T_instance_f32(numStages, pCoeffs, state)

# Inicializar o filtro biquad
arm_biquad_cascade_df2T_init_f32(S, numStages, pCoeffs, state)

# Caminho para o arquivo WAV filtrado
output_wave_name = "G:\\TCC\\Audios gravados\\Gravações P4 tricúspide\\10segundos S1S2 FILTRADO.wav"  # Nome do arquivo de saída


# Abrir o arquivo WAV original para leitura
with wave.open(input_wave_name, 'rb') as wave_in:
    channels = wave_in.getnchannels()
    sample_width = wave_in.getsampwidth()
    framerate = wave_in.getframerate()
    num_frames = wave_in.getnframes()

    # Ler todos os dados de áudio
    audio_data = wave_in.readframes(num_frames)
    audio_samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

    # Aplicar o filtro IIR
    filtered_samples = arm_biquad_cascade_df2T_f32(S, audio_samples)

    # Converter de volta para int16
    filtered_samples_int16 = filtered_samples.astype(np.int16)

# Salvar o áudio filtrado em um novo arquivo WAV
with wave.open(output_wave_name, 'wb') as wave_out:
    wave_out.setnchannels(channels)
    wave_out.setsampwidth(sample_width)
    wave_out.setframerate(framerate)
    wave_out.writeframes(filtered_samples_int16.tobytes())

print(f"Arquivo filtrado salvo como {output_wave_name}")