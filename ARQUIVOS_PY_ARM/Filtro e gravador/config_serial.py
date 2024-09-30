import serial

# Configurações da serial
SERIAL_PORT = 'COM3'  # Porta serial
BAUD_RATE = 460800    # Configuração da taxa de transmissão da serial
CHUNK = 2             # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)

def setup_serial():
    """Configura e retorna a conexão serial."""
    return serial.Serial(SERIAL_PORT, BAUD_RATE)

def record_from_serial(waveform, total_samples, port):
    """Captura dados da serial e grava no arquivo WAV."""
    print("...Iniciando captura de dados da serial...")
    for _ in range(total_samples):
        valor_serial = port.read(CHUNK)
        waveform.writeframes(valor_serial)
    print("___Captura e gravação finalizadas___")