
# O código deve ser utilizado no COLAB!!!


import math 
import matplotlib.pyplot as plt


# Dados simulados: [Ângulo (graus), Distância (cm)]
dados_simulados = [
    (0, 150), (10, 145), (20, 140), (30, 135), (40, 100), (50, 90), 
    (60, 120), (70, 150), (80, 170), (90, 200), (100, 170), 
    (110, 150), (120, 120), (130, 90), (140, 100), (150, 135), 
    (160, 140), (170, 145), (180, 150)                                # Um for interando esses dados já basta para a plotagem em um gráfico polar.
]

# Converte os dados para coordenadas cartesianas
x = []  # Cria vetores para facilitar na plotagem do gráfico no plano cartesiano
y = []

for angulo, distancia in dados_simulados:
    x.append(distancia * math.cos(math.radians(angulo)))  # É necessário utilizar está fóruma para tranformar coordenadas polares em pontos cartesianos
    y.append(distancia * math.sin(math.radians(angulo)))  # math.radians transforma o ângulo em radianos

# Criar gráfico 2D (cartesiano)
plt.figure(figsize=(8, 8))
plt.scatter(x, y, color='b', label='Pontos mapeados')  # Pontos mapeados
plt.plot(x, y, linestyle='-', color='r', alpha=0.6, label='Trajetória')  # Linhas conectando os pontos
plt.axhline(0, color='black', linewidth=0.5)  # Linha central
plt.axvline(0, color='black', linewidth=0.5)  # Linha central
plt.title("Simulação de Mapeamento com Sensor Ultrassônico")
plt.xlabel("X (cm)")
plt.ylabel("Y (cm)")
plt.legend()
plt.grid()
plt.show()

# Criar gráfico polar (radial)
plt.figure(figsize=(8, 8))
ax = plt.subplot(111, polar=True)
for angulo, distancia in dados_simulados: # Oia o for aqui, ele facilita a plotagem no caso das coordenadas polares
    ax.plot(math.radians(angulo), distancia, 'bo')  # Pontos
ax.set_title("Simulação de Mapeamento (Gráfico Polar)")
plt.show()
