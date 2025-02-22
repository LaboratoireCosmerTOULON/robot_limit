import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time

import cldLib as cld


# Création de la matrice de transformation
wMo = cld.homogeneousMatrix(1, 0, 0, 45, 30, 60)  # Ex. rotation + translation

# ---- on definiti la vitesse de commande dans le repere du robot----#
v0=np.array([1.0,0.4,0.0])
w0=np.array([0.0,0.0,0.2])

# --- on deduit le deplacement du robot si on applique cette vitesse  pendant dt secondes---#
dt=1
v=np.copy(v0)
w=np.copy(w0)


# on va donc limiter les valeurs de x, y et z
xlim = 2
ylim = 2
zlim = 2

# on definit la matrice de transformation initiale
xmin = -xlim
xmax = xlim

ymin = -xlim
ymax = xlim


# Paramètres de la grille
xlim = (-2, 2)
ylim = (-2, 2)
xmin, xmax = xlim
ymin, ymax = ylim

x = np.linspace(xlim[0], xlim[1], 20)  # 20 points pour un champ discret
y = np.linspace(ylim[0], ylim[1], 20)
X, Y = np.meshgrid(x, y)

# Initialisation des tableaux pour le champ de vecteurs
Vx = np.zeros_like(X)
Vy = np.zeros_like(Y)

# Remplissage des valeurs du champ vectoriel
for i in range(len(x)):
    for j in range(len(y)):
        V = cld.potentiel(X[i, j], Y[i, j], xmin, xmax, ymin, ymax)
        Vx[i, j] = V[0]
        Vy[i, j] = V[1]

# Affichage du champ de vecteurs
plt.figure(figsize=(8, 6))
plt.quiver(X, Y, Vx, Vy, color="blue", angles="xy", scale_units="xy", scale=1)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Champ de vecteurs du potentiel")
plt.grid(True, linestyle="--", alpha=0.5)

plt.show()
