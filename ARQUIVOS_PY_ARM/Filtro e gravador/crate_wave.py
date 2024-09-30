import wave
import numpy as np
import scipy.signal as signal

CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16           # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1          # Canal mono

def create_wave_file(filename, framerate):
    """Cria e configura um arquivo WAV para gravação."""
    waveform = wave.open(filename, 'wb')
    waveform.setnchannels(CHANNELS)
    waveform.setsampwidth(CHUNK)
    waveform.setframerate(framerate)
    return waveform

def apply_filter(input_filename, output_filename, sos):
    """Aplica o filtro e salva o áudio filtrado em um novo arquivo WAV."""
    with wave.open(input_filename, 'rb') as wave_in:
        num_frames = wave_in.getnframes()
        framerate = wave_in.getframerate()

        audio_data = wave_in.readframes(num_frames) # Le as amostras de áudio 
        audio_int = (np.frombuffer(audio_data, dtype=np.int16)) # Coloca para int16 

        filtered_samples = signal.sosfilt(sos, audio_int) # Filtragem do sinal 

        # Normalizar os dados filtrados para 16 bits
        filtered_samples = np.clip(filtered_samples, -32768, 32767)
        filtered_samples = filtered_samples.astype(np.int16)

    with wave.open(output_filename, 'wb') as wave_out:
        wave_out.setnchannels(CHANNELS)
        wave_out.setsampwidth(CHUNK)
        wave_out.setframerate(framerate)
        wave_out.writeframes(filtered_samples.tobytes())
        print(f"Arquivo filtrado salvo como {output_filename}")

    wave_in.close()
    wave_out.close()