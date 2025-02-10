import matplotlib.pyplot as plt
import math

def gerar_graficos(dados):
    """
    Gera gráficos cartesiano e polar a partir dos dados fornecidos.
    :param dados: Lista de tuplas (ângulo, distância).
    """
    # Conversão para coordenadas cartesianas
    x = []
    y = []
    for angulo, distancia in dados:
        x.append(distancia * math.cos(math.radians(angulo)))
        y.append(distancia * math.sin(math.radians(angulo)))
    
    # Gráfico cartesiano
    plt.figure(figsize=(8, 8))
    plt.scatter(x, y, color='b', label='Pontos mapeados')
    plt.plot(x, y, linestyle='-', color='r', alpha=0.6, label='Trajetória')
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.title("Mapeamento a partir de Dados do CSV (Cartesiano)")
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.legend()
    plt.grid()
    plt.show()

    # Gráfico polar
    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, polar=True)
    for angulo, distancia in dados:
        ax.plot(math.radians(angulo), distancia, 'bo')  # Pontos
    ax.set_title("Mapeamento a partir de Dados do CSV (Polar)")
    plt.show()