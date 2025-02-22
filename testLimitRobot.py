import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time

import cldLib as cld


# Création de la matrice de transformation
wMo = cld.homogeneousMatrix(1, 0, 0, 45, 30, 60)  # Ex. rotation + translation

# ---- on definiti la vitesse de commande dans le repere du robot----#
v0=np.array([-1.0,0.0,0.0])
w0=np.array([0.0,0.0,-0.1])

# --- on deduit le deplacement du robot si on applique cette vitesse  pendant dt secondes---#
dt=1

v=np.copy(v0)
w=np.copy(w0)
oMo2 = cld.homogeneous_from_twist(w, v, dt)
print("Matrice homogène après intégration :\n", oMo2)
wMo2 = wMo @ oMo2

# Affichage du repère
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
cld.frame(wMo,1,ax)
cld.frame(wMo2,1,ax)

# on repete cette vitesse pendant 10 secondes
for i in range(10):
    wMo2 = wMo2 @ oMo2
    cld.frame(wMo2, 1,ax )
    time.sleep(0.1)

plt.show()






#---------------------------------------------------------------------#
#-- on veut éviter que le robot sorte de la fenetre de visualisation--#
wMo = cld.homogeneousMatrix(1.0, 0.0, 0.0, 45.0, 30.0, 60.0)  # Ex. rotation + translation
v=np.copy(v0)
w=np.copy(w0)
# on va donc limiter les valeurs de x, y et z
xlim = 2
ylim = 2
zlim = 2
# on definit la matrice de transformation initiale

fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')

for i in range(10):
   
    # on voit ou on va arriver si on applique la vitesse
    oMo2 = cld.homogeneous_from_twist(w, v, dt)
    wMo_predict = wMo @oMo2
    wTo = wMo_predict[:3, 3]
    print('ITER ',i, 'T', wTo, ' -ok ', iter, 'v = ', v)
    
    alpha = 0.01
    # tant qu'on rentre dans le mur, on ralentit
    iter = 0 
    while np.abs(wTo[0])>xlim :
        iter += 1
        if iter > 100:
            break
        v[0] = alpha*v[0] 
        print('over limit x for ',i, 'T', wTo, ' - try reduce', iter, 'v = ', v)
        oMo2_try = cld.homogeneous_from_twist(w, v, dt)
        wMo_predict = wMo @oMo2_try
        wTo = wMo_predict[:3, 3]

    iter = 0 
    while np.abs(wTo[1])>ylim :
        iter += 1
        if iter > 100:
            break
        v[1] = alpha*v[1] 
        print('over limit y for ',i, 'T', wTo, ' - try reduce', iter, 'v = ', v)
        oMo2_try = cld.homogeneous_from_twist(w, v, dt)
        wMo_predict = wMo @oMo2_try
        wTo = wMo_predict[:3, 3]


    o2Mo = cld.homogeneous_from_twist(w, v, dt)
    wMo = wMo @oMo2
    cld.frame(wMo, 1,ax )

plt.show()







