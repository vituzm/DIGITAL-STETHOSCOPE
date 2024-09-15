import config_serial 
import crate_wave 
import filter_design

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
FILTER_BAND = [80, 800]
SECTIONS = 18
ATTENUATION_STOPBAND = 80


def main():

    # Criação do arquivo WAV para gravação
    waveform = crate_wave.create_wave_file(input_wave_name, CHANNELS, CHUNK, SAMPLE_RATE)

    try:
        # Configuração e uso da porta serial
        port = config_serial.setup_serial(SERIAL_PORT, BAUD_RATE)
        config_serial.record_from_serial(port, waveform, TOTALSAMPLES, CHUNK)
    finally:
        port.close()
        waveform.close()

    print(f"Arquivo {input_wave_name} criado com sucesso.")

    # Aplicação e salvamento dos filtros
    SOS = filter_design.design_filter_1(FILTER_BAND, SECTIONS, ATTENUATION_STOPBAND, SAMPLE_RATE)
    crate_wave.apply_filter(input_wave_name, output_wave_name, SOS, SAMPLE_RATE)

if __name__ == "__main__":
    main()
