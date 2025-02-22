# robot_limit
Method to avoid limit of the test area

* cldLib est un fichier avec toute les fonctions.
* Les autres fichiers sont des fichiers de test
* * test_deplacement.py montre comment créer un repere a une position donnée et le déplacer avec une vitesse edocentrée

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

On écrit les poses sous la forme d'une matrice de tranformation 4x4 wMo = [R  T ; 0_3x1 1] 

La matrice wMo permet d'exprimer la pose du repere o dans le repere M.
Pour exprimer la position d'un point oP, du repere objet dans le repere monde w, il suffit de faire le produit wP = wMo.oP.

