from scipy.signal import cheby2

# Definir a frequência de corte (faixa de 15 Hz a 250 Hz)
lowcut = 15.0
highcut = 250.0
fs = 8000.0  # Taxa de amostragem

# Definir a ordem do filtro e a atenuação fora da banda passante
secoes = 20
rs = 80  # Atenuação na banda de rejeição em dB

# Gerar coeficientes usando um filtro Chebyshev Tipo II de ordem 40
sos = cheby2(secoes, rs, [lowcut, highcut], btype='band', fs=fs, output='sos')

# Formatar e imprimir apenas b0, b1, b2, a1, a2 com no máximo 10 casas decimais para cada seção
for section in sos:
    b0, b1, b2, a0, a1, a2 = section
    print(f'{b0:.32f}, {b1:.32f}, {b2:.32f}, {-a1:.32f}, {-a2:.32f},')
