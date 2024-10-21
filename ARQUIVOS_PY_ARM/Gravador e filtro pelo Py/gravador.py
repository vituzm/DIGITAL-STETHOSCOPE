import serial
import wave 
import struct
from tkinter import *
import serial.tools.list_ports

# Parametros globais
BAUD_RATE = 230400                                              # Configuração da taxa de transmissão da serial
CHUNK = 2                                                       # Tamanho do buffer agora é 2 bytes por vez (16 bits = 2 bytes)
FORMAT = 16                                                     # Usando 16 bits por amostra (não precisa ser mudado, só é informativo)
CHANNELS = 1                                                    # Canal mono
SAMPLE_RATE = 8000                                              # Taxa de amostragem em Hz
input_wave_name  = "C:\\Users\\Vitor\\Downloads\\pulmaoaa.wav"  # Nome do arquivo de saída
GAIN = 3  # Fator de ganho

# Função para aplicar o ganho e limitar o valor
def aplicar_ganho_e_limitar(valor, ganho):
    # Aplicar ganho
    valor_amplificado = int(valor * ganho)
    # Limitar o valor para não exceder o intervalo de -32768 a 32767
    return max(min(valor_amplificado, 32767), -32768)

# Função para listar portas seriais disponíveis
def listar_portas():
    portas = serial.tools.list_ports.comports()
    return [porta.device for porta in portas]

def main():  
    global SERIAL_PORT
    
    # Verificar se uma porta foi selecionada
    SERIAL_PORT = variable.get()
    if SERIAL_PORT == "Selecione uma porta serial":
        status_label.config(text="Por favor, selecione uma porta serial.")
        return

    # Mensagem indicando a gravação 
    status_label.config(text="Gravando...")

    # Captura o valor de RECORD_SECONDS a partir do slider
    RECORD_SECONDS = record_seconds_var.get()  # Obtém o valor do slider
    TOTALSAMPLES = SAMPLE_RATE * RECORD_SECONDS  # Número total de amostras a serem gravadas

    # Criar e configurar o arquivo WAV
    waveform = wave.open(input_wave_name, 'wb')
    waveform.setnchannels(CHANNELS)
    waveform.setsampwidth(2)  # Tamanho do sample em bytes (16 bits = 2 bytes)
    waveform.setframerate(SAMPLE_RATE)

    try:
        # Abrir a porta serial com timeout
        port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=100)
        print("...Iniciando captura de dados da serial...")

        # Ler e gravar dados
        for i in range(TOTALSAMPLES):
            valor_serial = port.read(CHUNK)  # Ler o tamanho de uma amostra completa
            
            # Desempacotar o valor lido para um inteiro (assumindo little-endian e signed int16)
            valor_int = struct.unpack('<h', valor_serial)[0]
            
            # Aplicar ganho e limitar o valor
            valor_ajustado = aplicar_ganho_e_limitar(valor_int, GAIN)
            
            # Reempacotar o valor ajustado
            valor_final = struct.pack('<h', valor_ajustado)
            
            # Gravar os dados no arquivo WAV
            waveform.writeframes(valor_final)  
        print("___Captura e gravação finalizadas___")
    except:
        status_label.config(text="Porta serial não abriu")
    finally:
        port.close()
        waveform.close()

    print(f"Arquivo {input_wave_name} criado com sucesso.")


global status_label 
window = Tk()

# Definir tamanho fixo da janela (largura x altura)
window.title("Configuração de gravação")
window.geometry("350x300")

# Label fixo para Status
status_fixed_label = Label(window, text="Status:")
status_fixed_label.place(x=20, y=20)  # Posição do label fixo

# Label para status da gravação
status_label = Label(window, text="Selecione os parâmetros e inicie a gravação.")
status_label.place(x=60, y=20)  # Posição do label de status

# Label para o slider
slider_label = Label(window, text="Duração da gravação (segundos):")  # Título do slider
slider_label.place(x=20, y=60)  # Posição do label do slider

#slider
record_seconds_var = IntVar(value=1)  # Valor padrão para o slider
escala = Scale(
    window,
    from_= 1, 
    to = 100,
    length = 200,
    variable = record_seconds_var,
    orient= 'horizontal')
escala.place(x=20, y=80)  # Posição do slider

# Combobox para selecionar a porta
portas = listar_portas()
variable = StringVar(window)
variable.set("Selecione uma porta serial")

question_menu = OptionMenu(window, variable, *portas) 
question_menu.place(x=20, y=140)  # Posição do menu de opções
    
# Botão para iniciar o processo principal
btn_start = Button(window, text="Começar gravação", command=main)
btn_start.place(x=20, y=200)  # Posição do botão

window.mainloop()
