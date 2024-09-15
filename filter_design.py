import scipy.signal as signal
import numpy as np

def design_filter_1(FILTER_BAND, SECTIONS, ATTENUATION_STOPBAND, SAMPLE_RATE):
    """Projeta o primeiro filtro passa-faixa e retorna os coeficientes SOS."""

    sos = signal.iirfilter(
        N=SECTIONS,
        Wn=FILTER_BAND,
        btype='band',
        ftype='cheby2',
        rs=ATTENUATION_STOPBAND,
        fs=SAMPLE_RATE,
        output='sos'
    )
        
    # Coeficientes b1, b2 ... do filtro passa-faixa (NUM)
    NUM = np.array([
    [0.8624198568921, 0, 0],
    [1, -1.998670826668, 1],
    [0.8624198568921, 0, 0],
    [1, -1.888086620568, 1],
    [0.8460114408911, 0, 0],
    [1, -1.998735632308, 1],
    [0.8460114408911, 0, 0],
    [1, -1.882517082897, 1],
    [0.82103877482, 0, 0],
    [1, -1.99886147684, 1],
    [0.82103877482, 0, 0],
    [1, -1.86994955622, 1],
    [0.7841933038325, 0, 0],
    [1, -1.99904005788, 1],
    [0.7841933038325, 0, 0],
    [1, -1.846676605314, 1],
    [0.7302088800371, 0, 0],
    [1, -1.999257176309, 1],
    [0.7302088800371, 0, 0],
    [1, -1.804047177023, 1],
    [0.651403122932, 0, 0],
    [1, -1.999491321984, 1],
    [0.651403122932, 0, 0],
    [1, -1.720144621802, 1],
    [0.5386986683487, 0, 0],
    [1, -1.999713389608, 1],
    [0.5386986683487, 0, 0],
    [1, -1.528828001397, 1],
    [0.3889954546249, 0, 0],
    [1, -1.999889347166, 1],
    [0.3889954546249, 0, 0],
    [1, -0.9720823575347, 1],
    [0.2369655318849, 0, 0],
    [1, -1.999987250873, 1],
    [0.2369655318849, 0, 0],
    [1, 1.000464398742, 1],
    [1, 0, 0]
])

    # Coeficientes do denominador (DEN)
    DEN = np.array([
    [1, 0, 0],
    [1, -1.891760963501, 0.9804499558419],
    [1, 0, 0],
    [1, -1.995609509674, 0.9972774801027],
    [1, 0, 0],
    [1, -1.850802901145, 0.9402862726036],
    [1, 0, 0],
    [1, -1.990166632035, 0.9917807275078],
    [1, 0, 0],
    [1, -1.802227392722, 0.8950044447261],
    [1, 0, 0],
    [1, -1.98458645157, 0.9861004094676],
    [1, 0, 0],
    [1, -1.741039709033, 0.8397668069517],
    [1, 0, 0],
    [1, -1.978751776626, 0.9801255926472],
    [1, 0, 0],
    [1, -1.661550732935, 0.7691472101068],
    [1, 0, 0],
    [1, -1.972605452494, 0.9738093768699],
    [1, 0, 0],
    [1, -1.966221053753, 0.9672406277127],
    [1, 0, 0],
    [1, -1.558794546743, 0.6782816849278],
    [1, 0, 0],
    [1, -1.959933615913, 0.9607756072767],
    [1, 0, 0],
    [1, -1.43380333782, 0.5675188073371],
    [1, 0, 0],
    [1, -1.954521188554, 0.9552191991802],
    [1, 0, 0],
    [1, -1.304960680982, 0.452751288454],
    [1, 0, 0],
    [1, -1.951240879494, 0.9518569463744],
    [1, 0, 0],
    [1, -1.216773371593, 0.3738062390593],
    [1, 0, 0]
    ])
    # Montar o array SOS com os coeficientes 
    sos = np.hstack([NUM, DEN])

    return sos