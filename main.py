"""Arquivo principal do projeto."""
import config_serial 
import crate_wave 
import filter_design
from tkinter import *

SAMPLE_RATE = 8000   # Taxa de amostragem em Hz
RECORD_SECONDS = 15   # Tempo de gravação  

TOTALSAMPLES = SAMPLE_RATE * RECORD_SECONDS # Número total de amostras a serem gravadas

def toggle_state(state):
    global audio, audio_filtrado, pulmao
    pulmao = state

    if state:  # Se o estado for True, define os nomes para pulmão
        audio = "C:\\Users\\Vitor\\Downloads\\pulmao.wav"
        audio_filtrado = "C:\\Users\\Vitor\\Downloads\\pulmao FILTRADO.wav"
    else:  # Se o estado for False, define os nomes para coração
        audio  = "C:\\Users\\Vitor\\Downloads\\coracao.wav"
        audio_filtrado = "C:\\Users\\Vitor\\Downloads\\coracao FILTRADO.wav"

def main():
    # Mensagem indicando a gravação 
    status_label.config(text="Gravando...")

    # Criação do arquivo WAV para gravação
    waveform = crate_wave.create_wave_file(audio, SAMPLE_RATE)

    try:
        # Configuração e uso da porta serial
        port = config_serial.setup_serial()
        config_serial.record_from_serial(waveform, TOTALSAMPLES, port)
    finally:
        port.close()
        waveform.close()

    print(f"Arquivo {audio} criado com sucesso.")

    # Aplicação e salvamento dos filtros
    SOS = filter_design.design_filter_1(pulmao, SAMPLE_RATE)
    crate_wave.apply_filter(audio, audio_filtrado, SOS)
    
    # Mensagem indicando o fim da gravação e opção de nova gravação
    status_label.config(text="Gravação concluída! Pronto para uma nova gravação.")


# Interface gráfica 
def create_gui():
    global status_label
    window = Tk()

    window.title("Configuração de Filtro")
    # Label para status da gravação
    status_label = Label(window, text="Selecione o filtro e inicie a gravação.")
    status_label.pack(pady=5)

    # Definir tamanho fixo da janela (largura x altura)
    window.geometry("300x200")

    # Botão para configurar o filtro do Pulmão
    btn_pulmao = Button(window, text="Filtro para o Pulmão", command=lambda: toggle_state(True))
    btn_pulmao.pack(pady=5)

    # Botão para configurar o filtro do Coração
    btn_coracao = Button(window, text="Filtro para o Coração", command=lambda: toggle_state(False))
    btn_coracao.pack(pady=5)

    # Botão para iniciar o processo principal
    btn_start = Button(window, text="Começar gravação", command=main)
    btn_start.pack(pady=20)

    window.mainloop()

if __name__ == "__main__":
    create_gui()
