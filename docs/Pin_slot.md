# Parameters

*Dans les différentes représentations ci-dessous, une sphère fait contact au plan en réalité le contact est réalisé en un seul point (ie le rayon de la sphère est nul)*

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180995202-8aa4efe1-f6f4-4875-af6d-c4638de9d920.svg">
</p>

Le paramétrage d'une sphère-plan s'effectue grâce à ces 5 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant l'axe de glissement exprimé dans la base de `s1`
- `d1`, float : Distance algébrique entre l'origine de `s1` et l'axe de glissement
- `p2`, tuple : Coordonnée du point de pivot dans le repère de `s2`

Il est constamment possible de changer les valeurs de `a1`, `d1` et `p2` même après la compilation du mécanisme. 

# Kinematics

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180994983-7e22c490-3204-499d-95a2-d96de7a14656.svg">
</p>

- `point`, 2darray : Coordonnées successives du centre de la sphère exprimées dans le système de coordonnées global
- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`
- `delta`, 1darray : Valeurs successives de la distance algébrique du centre de la sphère par rapport à l'origine de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement et la valeur de glissement sont exprimés. Le pilotage d'une liaison sphère-plan permet de fixer les attributs `angle` et `delta`. Il n'est pas possible de piloter qu'une seule des 2 variables.
