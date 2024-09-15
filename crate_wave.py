import wave
import numpy as np
import scipy.signal as signal

def create_wave_file(filename, channels, sampwidth, framerate):
    """Cria e configura um arquivo WAV para gravação."""
    waveform = wave.open(filename, 'wb')
    waveform.setnchannels(channels)
    waveform.setsampwidth(sampwidth)
    waveform.setframerate(framerate)
    return waveform

def apply_filter(input_filename, output_filename, sos, SAMPLE_RATE):
    """Aplica o filtro e salva o áudio filtrado em um novo arquivo WAV."""
    with wave.open(input_filename, 'rb') as wave_in:
        channels = 1
        sample_width = 2
        framerate = SAMPLE_RATE
        num_frames = wave_in.getnframes()

        audio_data = wave_in.readframes(num_frames)
        audio_int = (np.frombuffer(audio_data, dtype=np.int16))

        filtered_samples = signal.sosfilt(sos, audio_int)

        # Normalizar os dados filtrados para 16 bits
        filtered_samples = np.clip(filtered_samples, -32768, 32767)
        filtered_samples = filtered_samples.astype(np.int16)

    with wave.open(output_filename, 'wb') as wave_out:
        wave_out.setnchannels(channels)
        wave_out.setsampwidth(sample_width)
        wave_out.setframerate(framerate)
        wave_out.writeframes(filtered_samples.tobytes())
        print(f"Arquivo filtrado salvo como {output_filename}")
