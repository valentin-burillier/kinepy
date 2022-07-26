# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985073-b8f29ecb-9536-444b-bb1d-f5c63a8198e6.svg">
</p>

Le paramétrage de la pivot s'effectue grâce à ces 4 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `p1`, tuple : Coordonnée du point de pivot dans le repère de `s1`
- `p2`, tuple : Coordonnée du point de pivot dans le repère de `s2`

Il est constament possible de changer les valeurs de `p1` et `p2` même après la compilation du mécanisme. 

# Kinematics

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985253-4236026d-37ac-4d80-8c5a-075d7e914bbe.svg">
</p>

- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`
- `point`, 2darray : Coordonnées des points de pivot successifs exprimées dans le système de coordonnées global

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement est exprimé. Le pilotage d'une liaison pivot permet de fixer l'attribut `angle`.
