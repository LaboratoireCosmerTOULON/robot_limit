import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time
import cldLib as cld
from quaternion import as_float_array, quaternion


# Création de la matrice de transformation
w_M_r0_pose = np.array([0, 0, 0, 30, 30, 0])
w_M_r0 = cld.homogeneousMatrix(*w_M_r0_pose)  # Ex. rotation + translation
print("pose", w_M_r0_pose)
print("Matrice de la pose :\n", w_M_r0)

w_T_r0, w_R_r0_euler = cld.matrix_to_translation_euler(w_M_r0);
print("Translation et angles d'Euler :\n", w_T_r0,w_R_r0_euler)

# ---- on definiti la vitesse de commande dans le repère du robot----#
r0_v_r0 = np.array([0.0,0.0,0.0])
r0_w_r0 = np.array([0.0,0.0,10.0])

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
ax = fig.add_subplot(131, projection='3d')
cld.frame(w_M_r0,1,ax)
cld.frame(w_M_r1,1,ax)
#plt.show()



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
# Calcule de la commande version Claire
#######################




# erreur en pose
r1_M_r0 = np.linalg.inv(w_M_r1)*w_M_r0
r1_T_r0, r1_R_r0_u_theta = cld.matrix_to_translation_utheta(r1_M_r0)
print("U, theta vector:", r1_R_r0_u_theta )


# Affichage du repère
fig = plt.figure(1)
ax = fig.add_subplot(132, projection='3d')
cld.frame(w_M_r0,1,ax)
cld.frame(w_M_r1,1,ax)

plt.ion()

seuil = 0.01
Kp = 0.5

w_M_r2=np.copy(w_M_r0)

# on repete cette vitesse pendant 10 secondes
for i in range(20):
    print("Claire")
      
    r2_M_r1 = np.linalg.inv(w_M_r2)@w_M_r1
    print(f"{r2_M_r1=}")

    r2_T_r1, r2_R_r1_u_theta = cld.matrix_to_translation_utheta(r2_M_r1)      
    epsilon_utheta = r2_R_r1_u_theta
    epsilon_translation = r2_T_r1
    print (f"{epsilon_utheta}")
    print (f"{epsilon_translation}")
    
    if np.linalg.norm(as_float_array(epsilon_utheta))<seuil and np.linalg.norm(epsilon_translation)<seuil : 
        print ( "STOP")
        print (f"{np.linalg.norm(as_float_array(epsilon_utheta))}")
        print (f"{np.linalg.norm(as_float_array(epsilon_translation))}")
        break

    v_commande = Kp*epsilon_translation
    w_commande = Kp*epsilon_utheta
    print(f"{v_commande=}")
    print(f"{w_commande=}")
    r2_M_r1 = cld.homogeneous_from_twist(w_commande, v_commande, dt)
    w_M_r2 = w_M_r2 @ r2_M_r1
    cld.frame(w_M_r2, 1,ax )
    plt.pause(1)



#######################
# Calcule de la commande version Martin
#######################

# integration du deplacement du a la commande 
dt = 0.1

# Affichage du repère
fig = plt.figure(1)
ax = fig.add_subplot(133, projection='3d')

cld.frame(w_M_r0,1,ax)
cld.frame(w_M_r1,1,ax)

plt.ion()
Kp = 1
seuil = 0.01

w_M_r2=np.copy(w_M_r0)

# on repete cette vitesse pendant 10 secondes
for i in range(20):
    print("Martin")

    w_T_r2, w_R_r2_quat = cld.matrix_to_translation_quaternion(w_M_r2)    
    
    # calcule de l'erreur en quaternion dans le monde
    epsilon_orientation = w_R_r1_quat * w_R_r2_quat.inverse()
    print( f"{epsilon_orientation=}" )
    
    epsilon_translation = w_T_r1 - w_T_r2
    print( f"{epsilon_translation=}" )

    if np.linalg.norm(as_float_array(epsilon_orientation))<seuil and np.linalg.norm(epsilon_translation)<seuil : 
        print ( "STOP")
        print (f"{np.linalg.norm(as_float_array(epsilon_orientation))}")
        print (f"{np.linalg.norm(as_float_array(epsilon_translation))}")
        break
   
    v_commande = Kp*epsilon_translation
    w_commande = Kp*np.array(as_float_array(epsilon_orientation)[1:])
    print(f"{v_commande=}")
    print(f"{w_commande=}")
    r2_M_r1 = cld.homogeneous_from_twist(w_commande, v_commande, dt)
    w_M_r2 = w_M_r2 @ r2_M_r1
    cld.frame(w_M_r2, 1,ax )
    plt.pause(1)
    
#plt.show()





