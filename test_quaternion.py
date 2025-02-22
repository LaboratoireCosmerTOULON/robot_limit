import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time
import cldLib as cld

# Création de la matrice de transformation
pose = np.array([0, 0, 0, 10, 20, 30])
wMo = cld.homogeneousMatrix(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])  # Ex. rotation + translation
print("pose", pose)
print("Matrice de la pose :\n", wMo)

t_wo,euler_wo = cld.matrix_to_translation_euler(wMo);
print("Translation et angles d'Euler :\n", t_wo, euler_wo)


# ---- on definiti la vitesse de commande dans le repère du robot----#
v0=np.array([1.0,0.0,0.0])
w0=np.array([0.0,0.0,0.0])

# --- on deduit le deplacement du robot si on applique cette vitesse  pendant dt secondes---#
dt=0.1
v=np.copy(v0)
w=np.copy(w0)

oMo2 = cld.homogeneous_from_twist(w, v, dt)
print("Matrice homogène du déplacement :\n", oMo2)
t_oo2,euler_oo2 = cld.matrix_to_translation_euler(oMo2);
print("Translation et angles d'Euler du mouvement :\n", t_oo2, euler_oo2)



wMo2 = wMo @ oMo2
print("Matrice homogène apres déplacement :\n", wMo2)

t_wo2,euler_wo2 = cld.matrix_to_translation_euler(wMo2);
print("Translation et angles d'Euler apres mouvement :\n", t_wo2, euler_wo2)

####################
t_wo,q_wo= cld.matrix_to_translation_quaternion(wMo)
print("Translation et quaternion :\n", t_wo, q_wo)

t_wo2,q_wo2= cld.matrix_to_translation_quaternion(wMo2)
print("Translation et quaternion apres mouvement :\n", t_wo2, q_wo2)
####################



