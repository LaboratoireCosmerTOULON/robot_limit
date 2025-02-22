import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time

import cldLib as cld


# Création de la matrice de transformation
wMo = cld.homogeneousMatrix(1, 0, 0, 45, 30, 60)  # Ex. rotation + translation
# Points d'origine en coordonnées homogènes
oO = np.array([0, 0, 0, 1])
oX = np.array([1, 0, 0, 1])
oY = np.array([0, 1, 0, 1])
oZ = np.array([0, 0, 1, 1])

# Application de la transformation
cO = wMo @ oO
cX = wMo @ oX
cY = wMo @ oY
cZ = wMo @ oZ

print("Origine transformée:", cO)
print("X transformé:", cX)
print("Y transformé:", cY)
print("Z transformé:", cZ)
print("Matrice de transformation:\n", wMo)

# ---- on definiti la vitesse de commande dans le repere du robot----#
v0=np.array([1.0,0.0,0.0])
w0=np.array([0.0,0.0,0.1])

# --- on deduit le deplacement du robot si on applique cette vitesse  pendant dt secondes---#
dt=1

v=np.copy(v0)
w=np.copy(w0)
o2Mo = cld.homogeneous_from_twist(w, v, dt)
print("Matrice homogène après intégration :\n", o2Mo)
wMo2 = wMo @ np.linalg.inv(o2Mo)

# Affichage du repère
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
cld.frame(wMo,1,ax)
cld.frame(wMo2,1,ax)

# on repete cette vitesse pendant 10 secondes
for i in range(10):
    wMo2 = wMo2 @ np.linalg.inv(o2Mo)
    cld.frame(wMo2, 1,ax )
    time.sleep(0.1)

plt.show()





