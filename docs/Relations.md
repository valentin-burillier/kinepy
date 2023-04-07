Une relation décrit une interaction accouplant deux liaisons. Elle est uniquement de nature affine. Cela permet entre autres de modéliser un train d'engrenage, un système pignon-crémaillère

Attention, aux solides de référence de chacune des liaisons mise en argument. Cela change les signes des paramètres cinématiques et dynamiques des liaisons.

# Engrenage

<p align="center" width="100%">
    <img width="90%" src="https://user-images.githubusercontent.com/93446869/189528398-bf80d262-3890-4425-a195-e6857afcb08c.svg">
</p>

- `Gear(rev1, rev2, r, v0=0., pressure_angle=np.pi/9)` : Accouple cinématiquement et dynamiquement deux liaisons pivots `rev1` et `rev2` afin de modéliser un train d'engrenagem. `rev1` et `rev2` doivent partager un solide en commun (en jaune sur l'image de gauche ci-dessus). L'argument `r` correspond au rapport de transmission. L'argument `v0` corespond à la valeur du déphasage entre `rev1` et `rev2` quand l'angle de `rev1` vaut zéro. Par défaut, `v0` vaut zéro. La relation cinématique s'écrit : `rev2.angle = rev1.angle x r + v0`. `angle_pressure` correspond à l'angle de pression de l'engrenage. Par défaut, il est de 20° (ou pi/9 rad).

# Système pignon-crémaillière

<p align="center" width="100%">
    <img width="90%" src="https:/m-/user-images.githubusercontent.com/93446869/189602526-e432c832-83a9-4bdc-a04e-3fced54fbf30.svg">
</p>

- `add_gearrack(rev, pri, r, v0=0.)` : Accouple cinématiquement et dynamiquement une pivot `rev` avec une glissière `pri` afin de modéliser une transmission pignon-crémaillère. `rev` et `pri` doivent partager un solide en commun (en jaune sur l'image de droite ci-dessus). La relation cinématique s'écrit : `pri.sliding = rev.angle x r + v0`. La transmission d'efforts entre les différents solides est prise en compte.

# Relation 

- `add_distant_relation(j1, j2, r, v0=0.)` : Accouple cinématiquement et dynamiquement deux liaisons chacunes de type `Prismatic` ou `Revolute`. La relation cinématique s'écrit : `j2.value = j1.value x r + v0` où `value` est soit `angle` ou `sliding`, tout dépend de la liaison considérée. La transmission d'efforts entre les différents solides est prise en compte. Cette méthode ajoute la possibilité qu'une glissière fasse agir une autre glissière ce qui peut modéliser dans certains systèmes l'action d'un vérin hydraulique sur un autre ou un système poulie-courroie dont l'une des poulies est en mouvement.
- `add_effortless_relation(j1, j2, r, v0=0.)` : Accouple cinématiquement deux liaisons chacunes de type `Prismatic` ou `Revolute`. À la différence des méthodes précédentes, il n'y a pas de transmission d'efforts entre les solides intervenant dans les liaisons `j1` et `j2`.

`r`, float : Rapport de transmission entre les liaisons
- `v0`, float : Valeur initiale du paramètre cinématique de la liaison (`angle` pour les pivots et `sliding` pour les glissières)

