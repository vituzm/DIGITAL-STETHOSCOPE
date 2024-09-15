import serial

def setup_serial(port, baud_rate):
    """Configura e retorna a conexão serial."""
    return serial.Serial(port, baud_rate)

def record_from_serial(serial_port, waveform, total_samples, chunk_size):
    """Captura dados da serial e grava no arquivo WAV."""
    print("...Iniciando captura de dados da serial...")
    for _ in range(total_samples):
        valor_serial = serial_port.read(chunk_size)
        waveform.writeframes(valor_serial)
    print("___Captura e gravação finalizadas___")