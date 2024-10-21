import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Definir a frequência de corte (faixa de 100 Hz a 800 Hz)
lowcut = 100
highcut = 800
fs = 8000.0  # Taxa de amostragem

# Definir a ordem do filtro e a atenuação fora da banda passante
secoes = 20
rs = 80  # Atenuação na banda de rejeição em dB

# Gerar coeficientes usando um filtro Chebyshev Tipo II de ordem 20
sos = signal.cheby2(secoes, rs, [lowcut, highcut], btype='bandpass', fs=fs, output='sos')

# Calcular a resposta em frequência
frequencies, response = signal.sosfreqz(sos, worN=2000, fs=fs)

# Plotar a resposta em frequência
plt.figure()
plt.plot(frequencies, 20 * np.log10(abs(response)), 'b')
plt.title('Resposta em Frequência do Filtro Chebyshev Tipo II: Pulmão')
plt.xlabel('Frequência [Hz]')
plt.ylabel('Magnitude [dB]')
plt.grid()
plt.axvline(lowcut, color='red', linestyle='--')
plt.axvline(highcut, color='red', linestyle='--')
plt.xlim(0, fs/4)
plt.ylim((-rs -20), 5)  # Ajuste conforme necessário
plt.show()
