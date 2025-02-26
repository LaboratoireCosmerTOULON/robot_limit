import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import cldLib as cld
import time
from quaternion import as_float_array, quaternion


def matrix_to_translation_utheta(r1_M_r0):
    r1_T_r0, r1_R_r0_quat = cld.matrix_to_translation_quaternion(r1_M_r0)
    r1_R_r0_u_theta = cld.quaternion_to_u_theta_vector(r1_R_r0_quat)
    return r1_T_r0, r1_R_r0_u_theta

# Function to convert quaternion to axis (u) and angle (theta) as a 4-element vector
def quaternion_to_u_theta_vector(q):
    # Extract quaternion components
    w, x, y, z = q.real, q.imag[0], q.imag[1], q.imag[2]
    
    # Compute the angle theta (radians)
    theta = 2 * np.arccos(w)
    
    # Compute the unit vector u (axis of rotation)
    sin_half_theta = np.sqrt(1 - w**2)
    
    # If sin_half_theta is zero, the quaternion represents no rotation
    if sin_half_theta > 0:
        u = np.array([x / sin_half_theta, y / sin_half_theta, z / sin_half_theta])
    else:
        u = np.array([1, 0, 0])  # Arbitrary axis if no rotation
    
    # Return the unit vector u and angle theta as a 4-element vector [u_x, u_y, u_z, theta]
    return theta*u

# Example quaternions


def matrix_to_translation_quaternion(T):
    # Extraire la translation
    translation = T[:3, 3]

    # Extraire la rotation et convertir en quaternion
    rotation_matrix = T[:3, :3]
    quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)  # (x, y, z, w)
    quat = quaternion(*quat)

    return translation, quat

def matrix_to_translation_euler(T):
    # Extraire la translation
    translation = T[:3, 3]

    # Extraire la rotation et convertir en angles d'Euler (convention ZYX : yaw, pitch, roll)
    rotation_matrix = T[:3, :3]
    euler_angles = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=True)  # Yaw, Pitch, Roll en degrés

    return translation, np.array([euler_angles[2],euler_angles[1],euler_angles[0]])

def quaternion_to_euler(quaternion):
    euler_angles = R.from_quat(as_float_array(quaternion), scalar_first=True).as_euler('zyx', degrees=True)  # Yaw, Pitch, Roll en degrés
    return np.array([euler_angles[2],euler_angles[1],euler_angles[0]]) # that's why ;-)

def potentiel(x, y, xmin, xmax, ymin, ymax):
    """
    Calcule le potentiel en (x, y) et retourne un vecteur [Vx, Vy].
    
    x, y : scalaires (pas des tableaux !)
    xmin, xmax, ymin, ymax : limites scalaires
    """
    dxmin = np.abs(x - xmin)
    dxmax = np.abs(x - xmax)
    vpotx = +np.exp(-dxmin)-  np.exp(-dxmax)
   
    dymin = np.abs(y - ymin)
    dymax = np.abs(y - ymax)
    vpoty = +np.exp(-dymin) - np.exp(-dymax)

    return np.array([vpotx, vpoty])  # Retourne un vecteur [Vx, Vy]


def coordFrame():
    """ Génère les coordonnées du repère. """
    oC1 = np.array([0, 0, 0, 1])  # Origine
    oC2 = np.array([1, 0, 0, 1])  # X-axis
    oC3 = np.array([0, 1, 0, 1])  # Y-axis
    oC4 = np.array([0, 0, 1, 1])  # Z-axis
    return np.array([oC1, oC2, oC3, oC4])

def displayCoord(cCs, numFig, ax, max_val=2.0):
    """ Affiche le repère en 3D avec rotation interactive. """

    # Extraire l'origine et les directions
    origin = cCs[0][:3]
    x_dir = cCs[1][:3] - origin
    y_dir = cCs[2][:3] - origin
    z_dir = cCs[3][:3] - origin

    # Dessiner les axes
    ax.quiver(*origin, *x_dir, color='r', length=1, normalize=True, label='X-axis')
    ax.quiver(*origin, *y_dir, color='g', length=1, normalize=True, label='Y-axis')
    ax.quiver(*origin, *z_dir, color='b', length=1, normalize=True, label='Z-axis')

    # Ajustement des limites
    max_range = 1.2 * np.max(np.abs(cCs[:, :3]))
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])

    # Labels et affichage interactif
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Coordinate Frame')
   # ax.legend()
    


def rotMatrix(rx, ry, rz) :
  rot = R.from_euler('zyx',[rz,ry,rx], degrees=True)
  return rot.as_matrix()

def homogeneousMatrix(tx,ty,tz, rx, ry, rz) :
  wRo = rotMatrix( rx, ry, rz)
  wTo = np.array([[tx,ty,tz]])
  ligne4 =  np.array([[0,0,0,1]])
  M = np.concatenate((np.concatenate((wRo, wTo.T), axis=1),ligne4))
  return M

def changeFramePoints(oXs, wMo):
    oXs = np.array(oXs)  # Ensure it's a NumPy array
    cXs = (wMo @ oXs.T).T  # Apply transformation to all points
    return cXs

def frame(wMo, numFig,ax) :
  oCs = coordFrame()
  wCs = changeFramePoints(oCs,wMo)
  displayCoord(wCs, numFig,ax)
  return wCs


def skew_symmetric(omega):
    """ Convertit un vecteur de rotation en matrice antisymétrique """
    return np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])

def adj_matrix(R, p):
    """ Calcule la matrice d'adjacence pour le changement de repère """
    S_p = skew_symmetric(p)
    adj = np.block([
        [R, S_p @ R],
        [np.zeros((3, 3)), R]
    ])
    return adj

def adj_from_homogeneous(T):
    """ Calcule la matrice d'adjacence à partir d'une matrice homogène 4x4 """
    R = T[:3, :3]  # Extraction de la rotation
    p = T[:3, 3]   # Extraction de la translation
    S_p = skew_symmetric(p)
    
    Adj = np.block([
        [R, S_p @ R],
        [np.zeros((3, 3)), R]
    ])
    
    return Adj


def twist_matrix(omega, v):
    """ Crée une matrice 4x4 de twist à partir de la vitesse angulaire et linéaire """
    omega_hat = skew_symmetric(omega)
    twist = np.zeros((4,4))
    twist[:3, :3] = omega_hat  # Partie rotationnelle
    twist[:3, 3] = v           # Partie translationnelle
    return twist

def homogeneous_from_twist(omega, v, dt):
    """ Calcule la matrice homogène via l'exponentielle de matrice """
    twist = twist_matrix(omega, v)
    return expm(twist * dt)  # Exponentielle de matrice


