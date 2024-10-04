from scipy import signal

# Definir a frequência de corte (faixa de 15 Hz a 250 Hz)
lowcut = 100
highcut = 800
fs = 8000.0  # Taxa de amostragem

# Definir a ordem do filtro e a atenuação fora da banda passante
secoes = 20
rs = 80  # Atenuação na banda de rejeição em dB

# Gerar coeficientes usando um filtro Chebyshev Tipo II de ordem 40
z, p, k = signal.cheby2(secoes, rs, [lowcut, highcut], btype='bandpass', fs=fs, output='zpk')

k = k * 2

# Converter para SOS
sos = signal.zpk2sos(z, p, k)

# Formatar e imprimir apenas b0, b1, b2, a1, a2 com no máximo 32 casas decimais para cada seção
for section in sos:
    b0, b1, b2, a0, a1, a2 = section
    print(f'{b0*10:.32f}, {b1*10:.32f}, {b2*10:.32f}, {-a1:.32f}, {-a2:.32f},')
