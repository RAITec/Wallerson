# Bibliotecas de Interesse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import seaborn as sns
import imageio
import math
import networkx as nx
import time
from scipy.interpolate import interp1d

# Importação do Mapa
fig = plt.figure(figsize=(15,15), dpi=100)
ax0 = fig.add_subplot(131, aspect='equal')
img0 = mpimg.imread('Mapa0.png')
ax0.imshow(img0, cmap='Greys', origin='upper');

ax1 = fig.add_subplot(132, aspect='equal')
img1 = mpimg.imread('Mapa2.png')
ax1.imshow(img1, cmap='Greys', origin='upper');

ax2 = fig.add_subplot(133, aspect='equal')
img2 = mpimg.imread('Mapa1.png')
ax2.imshow(img2, cmap='Greys', origin='upper');

threshold = 0.5
img0 = 1-img0
img1 = 1-img1
img2 = 1-img2
img0[img0 > threshold] = 1
img1[img1 > threshold] = 1
img2[img2 > threshold] = 1
img0[img0 <= threshold] = 0
img1[img1 <= threshold] = 0
img2[img2 <= threshold] = 0

# Dimensões do mapa informado em metros (X, Y)
mapCoppelia = 20
map_dims = mapCoppelia*np.array([30, 30])
sy, sx = img0.shape[:2] / map_dims

# Tamanho da célula do nosso Grid (em metros)
cell_size = mapCoppelia/2

rMC = (map_dims/cell_size)/mapCoppelia # Ajuste Mapa->Coppelia

rows, cols = (map_dims / cell_size).astype(int)

grid0 = np.zeros((rows, cols))
grid1 = np.zeros((rows, cols))
grid2 = np.zeros((rows, cols))
heatMap0 = np.zeros((rows, cols))
heatMap1 = np.zeros((rows, cols))
heatMap2 = np.zeros((rows, cols))

# Preenchendo o Grid
for r in range(rows):
    for c in range(cols):
        xi = int(c*cell_size*sx)
        xf = int(xi + cell_size*sx)
        yi = int(r*cell_size*sy)
        yf = int(yi + cell_size*sy)
        grid0[r, c] = np.sum(img0[yi:yf,xi:xf])
        grid1[r, c] = np.sum(img1[yi:yf,xi:xf])
        grid2[r, c] = np.sum(img2[yi:yf,xi:xf])
        if grid0[r, c]>threshold: grid0[r,c] = heatMap0[r,c] = 10
        else: grid0[r,c] = heatMap0[r,c] = 0
        if grid1[r, c]>threshold: grid1[r,c] = heatMap1[r,c] = 10
        else: grid1[r,c] = heatMap1[r,c] = 0
        if grid2[r, c]>threshold: grid2[r,c] = heatMap2[r,c] = 10
        else: grid2[r,c] = heatMap2[r,c] = 0
        

# Definição dos pesos de cada grid
for idGrid in range(9, 0, -1):
    for r in range(rows):
        for c in range(cols):
            # Mapa 0
            if heatMap0[r, c] == 0:
                nObs = 0
                try:
                    for i in range(r-1, r+2):
                        for j in range(c-1, c+2):
                            if heatMap0[i,j]>idGrid:
                                nObs = idGrid
                except:
                    'Do Nothing'
                heatMap0[r,c] = nObs
            # Mapa 1
            if heatMap1[r, c] == 0:
                nObs = 0
                try:
                    for i in range(r-1, r+2):
                        for j in range(c-1, c+2):
                            if heatMap1[i,j]>idGrid:
                                nObs = idGrid
                except:
                    'Do Nothing'
                heatMap1[r,c] = nObs
            # Mapa 2
            if heatMap2[r, c] == 0:
                nObs = 0
                try:
                    for i in range(r-1, r+2):
                        for j in range(c-1, c+2):
                            if heatMap2[i,j]>idGrid:
                                nObs = idGrid
                except:
                    'Do Nothing'
                heatMap2[r,c] = nObs

fig = plt.figure(figsize=(15,5), dpi=100);
ax0 = fig.add_subplot(131);
ax0 = sns.heatmap(heatMap0, cmap="YlOrBr", xticklabels=False, yticklabels=False, cbar=False);
ax1 = fig.add_subplot(132);
ax1 = sns.heatmap(heatMap1, cmap="YlOrBr", xticklabels=False, yticklabels=False, cbar=False);
ax2 = fig.add_subplot(133);
ax2 = sns.heatmap(heatMap2, cmap="YlOrBr", xticklabels=False, yticklabels=False);


# Criando o Grafo para o nosso Grid
G0 = nx.grid_2d_graph(rows, cols);
G1 = nx.grid_2d_graph(rows, cols);
G2 = nx.grid_2d_graph(rows, cols);

# Validação dos grids (C-Space)
for r in range(rows):
    for c in range(cols):
        # Mapa 0
        if heatMap0[r][c] > 6:
            G0.remove_node((r,c))
        # Mapa 1
        if heatMap1[r][c] > 7:
            G1.remove_node((r,c))
        # Mapa 2
        if heatMap2[r][c] > 8:
            G2.remove_node((r,c))

fig = plt.figure(figsize=(15,15), dpi=100)
ax0 = fig.add_subplot(131, aspect='equal')
ax1 = fig.add_subplot(132, aspect='equal')
ax2 = fig.add_subplot(133, aspect='equal')

# Grid
obj0 = ax0.imshow(grid0, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]))
obj1 = ax1.imshow(grid1, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]))
obj2 = ax2.imshow(grid2, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]))

ax0.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
ax1.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
ax2.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
ax0.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
ax1.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
ax2.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
ax0.set_yticks(np.arange(0, map_dims[0]+1, cell_size))
ax1.set_yticks(np.arange(0, map_dims[0]+1, cell_size))
ax2.set_yticks(np.arange(0, map_dims[0]+1, cell_size))

# Os vértices serão plotados no centro da célula
mapaAllPos0 = {node:(node[1]*cell_size+cell_size/2, map_dims[0]-node[0]*cell_size-cell_size/2) for node in G0.nodes()}
nx.draw(G0, mapaAllPos0, font_size=3, with_labels=True, node_size=50, node_color="c", ax=ax0)

mapaAllPos1 = {node:(node[1]*cell_size+cell_size/2, map_dims[0]-node[0]*cell_size-cell_size/2) for node in G1.nodes()}
nx.draw(G1, mapaAllPos1, font_size=3, with_labels=True, node_size=50, node_color="c", ax=ax1)

mapaAllPos2 = {node:(node[1]*cell_size+cell_size/2, map_dims[0]-node[0]*cell_size-cell_size/2) for node in G2.nodes()}
nx.draw(G2, mapaAllPos2, font_size=3, with_labels=True, node_size=50, node_color="c", ax=ax2)



try:
    import sim
except:
    print ('"sim.py" could not be imported.')

def Rz(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                      [ np.sin(theta), np.cos(theta) , 0 ],
                      [ 0            , 0             , 1 ]])