import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm  # Exponentielle de matrice
import time


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


