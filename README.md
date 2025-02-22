# robot_limit
Method to avoid limit of the test area


cldLib est un fichier avec toute les fonctions.
Les autres fichiers sont des fichiers de test

# Method

## Pose

Les poses sont les positions et les rotations. On les exprime en formalisme homogene et sous la forme d'un vecteur 6x1.

### forme vectorielle

Cette forme est principalement destinée à une définition des poses facilement interpretable par l'utilisateur.
Elle ne sera jamais utilisée dans les calculs.

On les exprime avec un vecteur de pose wpo = [X,Y,Z, rx,ry,rz].
rx, ry, rz sont les angles d'Euler. La rotation est construite en tournant sur z puis puis x (z,y,x).

La position X,Y,Z sont en [m].
Les rotations sont en [rad].


### formalisme homogène

Un point de l'espace est représenté par 4 scalaires : [X,Y,Z,1].
Si le dernier élément est différent de 1, alors, les 3 premiers ne sont plus les coord d'un point.
Il faut tout diviser par la valeur du dernier element pour obtenir les coordonnées.

# Matrice de Pose au Formalisme Homogène

## Introduction
En robotique et en vision par ordinateur, une **matrice de pose** est une matrice homogène utilisée pour représenter la transformation d'un repère à un autre. Elle combine **rotation** et **translation** dans un seul formalisme.

## Définition
Une matrice de pose \( \mathbf{T} \) est une **matrice homogène** \( 4 \times 4 \) qui s’écrit sous la forme :

\[
\mathbf{T} =
\begin{bmatrix}
\mathbf{R} & \mathbf{t} \\
\mathbf{0}^{T} & 1
\end{bmatrix}
\]

où :
- \( \mathbf{R} \in \mathbb{R}^{3 \times 3} \) est une matrice de **rotation**,
- \( \mathbf{t} \in \mathbb{R}^{3 \times 1} \) est un **vecteur de translation**,
- \( \mathbf{0}^{T} = [0\ 0\ 0] \) et \( 1 \) permettent d’étendre l’espace.

## Propriétés
- La matrice \( \mathbf{T} \) appartient au **groupe des transformations rigides** \( SE(3) \) (Special Euclidean group).
- L'inverse de \( \mathbf{T} \) est donné par :

\[
\mathbf{T}^{-1} =
\begin{bmatrix}
\mathbf{R}^T & -\mathbf{R}^T \mathbf{t} \\
\mathbf{0}^{T} & 1
\end{bmatrix}
\]

## Application
Une matrice homogène permet de transformer un point homogène \( \mathbf{p} \) dans un espace 3D :

\[
\mathbf{p}' = \mathbf{T} \mathbf{p}
\]

avec :

\[
\mathbf{p} =
\begin{bmatrix}
x \\
y \\
z \\
1
\end{bmatrix}
\]

et 

\[
\mathbf{p}' =
\begin{bmatrix}
x' \\
y' \\
z' \\
1
\end{bmatrix}
\]

Cela permet de transformer les coordonnées d’un point d’un repère à un autre.

## Conclusion
Le formalisme homogène est **essentiel** en robotique et en vision pour manipuler les transformations rigides de manière **élégante et compacte**.






