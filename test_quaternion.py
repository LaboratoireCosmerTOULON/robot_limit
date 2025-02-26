import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time
import cldLib as cld
from quaternion import as_float_array, quaternion


# Création de la matrice de transformation
w_M_r0_pose = np.array([0, 0, 0, 0, 0, 0])
w_M_r0 = cld.homogeneousMatrix(*w_M_r0_pose)  # Ex. rotation + translation
print("pose", w_M_r0_pose)
print("Matrice de la pose :\n", w_M_r0)

w_T_r0, w_R_r0_euler = cld.matrix_to_translation_euler(w_M_r0);
print("Translation et angles d'Euler :\n", w_T_r0,w_R_r0_euler)

# ---- on definiti la vitesse de commande dans le repère du robot----#
r0_v_r0 = np.array([1.0,0.0,0.0])
r0_w_r0 = np.array([0.0,0.0,0.0])

# --- on deduit le deplacement du robot si on applique cette vitesse  pendant dt secondes---#
dt=1
r0_M_r1 = cld.homogeneous_from_twist(r0_w_r0, r0_v_r0, dt)
print(f"{r0_M_r1=}")
r0_T_r1,r0_R_r1_euler = cld.matrix_to_translation_euler(r0_M_r1);
print("Translation et angles d'Euler du mouvement :\n", r0_T_r1,r0_R_r1_euler)

w_M_r1 = w_M_r0 @ r0_M_r1
print("Matrice homogène apres déplacement :\n", w_M_r1)

w_T_r1, w_R_r1_euler = cld.matrix_to_translation_euler(w_M_r1);
print("Translation et angles d'Euler apres mouvement :\n", w_T_r1,w_R_r1_euler)

# Affichage du repère
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
cld.frame(w_M_r0,1,ax)
cld.frame(w_M_r1,1,ax)
plt.show()



###################
# QUATERNION
####################
# La position de depart est la position du robot r0 
# La position a atteindre est la position du robot r1


w_T_r0, w_R_r0_quat = cld.matrix_to_translation_quaternion(w_M_r0)
print("Quaternion : position de départ dans le repere monde :\n", w_T_r0, w_R_r0_quat)

w_T_r1, w_R_r1_quat = cld.matrix_to_translation_quaternion(w_M_r1)
print("Quaternion : position à rejoindre dans le repere monde :\n", w_T_r1, w_R_r1_quat)

r0_T_r1, r0_R_r1_quat = cld.matrix_to_translation_quaternion(r0_M_r1)
print("Quaternion : erreur en trans et quaternion :\n", r0_T_r1, r0_R_r1_quat)

#######################
# Calcule de la commande version Algebre de Lie
#######################

# Convert quaternion to u, theta vector
w_R_r0_u_theta = cld.quaternion_to_u_theta_vector(w_R_r0_quat)
w_R_r1_u_theta = cld.quaternion_to_u_theta_vector(w_R_r1_quat)

print("Quaternion q1 -> u, theta vector:", w_R_r0_u_theta)
print("Quaternion q2 -> u, theta vector:", w_R_r1_u_theta)


#######################
# Calcule de la commande version Martin
#######################

# calcule de l'erreur en quaternion dans le monde
epsilon_orientation = w_R_r0_quat * w_R_r1_quat.inverse()
print( f"{epsilon_orientation=}" )

epsilon_translation = w_T_r0 - w_T_r1
print( f"{epsilon_translation=}" )

Kp = 0.5
v_commande = Kp*epsilon_translation
w_commande = Kp*np.array(as_float_array(epsilon_orientation)[1:])
print(f"{v_commande=}")
print(f"{w_commande=}")

# integration du deplacement du a la commande 
dt = 0.1
r1_M_r2 = cld.homogeneous_from_twist(w_commande, v_commande, dt)
print(f"{r1_M_r2=}")
r1_T_r2,r1_R_r2_euler = cld.matrix_to_translation_euler(r1_M_r2);
print("Mouvement du a une commande durant 0.1s :\n", r1_T_r2,r1_R_r2_euler)

# position incrémentale
w_M_r2 = w_M_r1 @ r1_M_r2
print(f"{w_M_r2=}")
w_T_r2,w_R_r2_euler = cld.matrix_to_translation_euler(w_M_r2);
print("Position atteinte apres une commande :\n", w_T_r2,w_R_r2_euler)

# Affichage du repère
fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')
cld.frame(w_M_r0,1,ax)
cld.frame(w_M_r1,1,ax)

plt.ion()

seuil = 0.1
Kp = 1

# on repete cette vitesse pendant 10 secondes
for i in range(20):

    w_T_r2, w_R_r2_quat = cld.matrix_to_translation_quaternion(w_M_r2)    
    
    # calcule de l'erreur en quaternion dans le monde
    epsilon_orientation = w_R_r0_quat * w_R_r2_quat.inverse()
    print( f"{epsilon_orientation=}" )
    


    epsilon_translation = w_T_r0 - w_T_r2
    print( f"{epsilon_translation=}" )


    if np.linalg.norm(as_float_array(epsilon_orientation))<seuil or np.linalg.norm(epsilon_translation)<seuil : 
        print ( "STOP")
        print (f"{np.linalg.norm(as_float_array(epsilon_orientation))}")
        print (f"{np.linalg.norm(as_float_array(epsilon_translation))}")
        break
   
    v_commande = Kp*epsilon_translation
    w_commande = Kp*np.array(as_float_array(epsilon_orientation)[1:])
    print(f"{v_commande=}")
    print(f"{w_commande=}")
    r1_M_r2 = cld.homogeneous_from_twist(w_commande, v_commande, dt)
    
    
    w_M_r2 = w_M_r2 @ r1_M_r2
    cld.frame(w_M_r2, 1,ax )
    plt.pause(1)
    
#plt.show()





