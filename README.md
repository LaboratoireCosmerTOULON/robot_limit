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

La position x, y, z sont en [m].

Les rotations sont en [rad].


### formalisme homogène






