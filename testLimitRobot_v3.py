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

fig = plt.figure(3)
ax = fig.add_subplot(111, projection='3d')

xmin = -xlim
xmax = xlim

ymin = -xlim
ymax = xlim

for i in range(10):

   # on est a la position wMo
   # on repere la distance a laquelle on est des limites
   # simplement axe/axe
   vpot = cld.potentiel(wMo[0,3], wMo[1,3], xmin, xmax, ymin, ymax)
   
   # ici c'est la somme des champ de potentiel appliqué à la position courante 
   wvpot = np.array([vpot[0],vpot[1],0,0,0,0])

   # il faut ensuite transformer ce vecteur de potentiel du monde au robot   
   # matrice de transformation pour la vitesse potentielle
   # du monde au robot
   oVw = cld.adj_from_homogeneous(np.linalg.inv(wMo))

   # on applique la transformation pour exprimer le potentiel dans le repere du robot
   ovpot = oVw @ wvpot
   
   # on ajoute la vitesse de commande
   vrobot = v - ovpot[0:3]

   print('v', v) 
   print('wvpot', wvpot)
   print('ovpot', ovpot[0:3])  
   print('vrobot', vrobot)
   

   o2Mo = cld.homogeneous_from_twist(w, vrobot, dt)
   wMo = wMo @np.linalg.inv(o2Mo)
   cld.frame(wMo, 1,ax )


#affichage du champ de potentiel
# Paramètres de la grille
xlim = (-2, 2)
ylim = (-2, 2)
xmin, xmax = xlim
ymin, ymax = ylim

x = np.linspace(xlim[0], xlim[1], 10)
y = np.linspace(ylim[0], ylim[1], 10)
X, Y = np.meshgrid(x, y)

# Initialisation des matrices pour stocker Vx et Vy
Vx = np.zeros_like(X)
Vy = np.zeros_like(Y)

# Remplissage des matrices Vx et Vy en appliquant la fonction potentiel()
for i in range(len(x)):
    for j in range(len(y)):
        V = cld.potentiel(X[i, j], Y[i, j], xmin, xmax, ymin, ymax)
        Vx[i, j] = V[0]  # Première composante du vecteur
        Vy[i, j] = V[1]  # Deuxième composante du vecteur

Vz = np.zeros_like(X)  # Composante Z nulle (on reste dans un champ 2D)

# Création de la figure 3D

# Tracer les flèches en 3D
ax.quiver(X, Y, np.zeros_like(X), Vx, Vy, Vz, color="grey", length=0.5, normalize=True)

# Labels et titre
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("Potentiel")
ax.set_title("Champ de vecteurs du potentiel en 3")

plt.show()




