# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/179976577-d70f24ea-7dc1-493e-8dd9-f10fcd847498.svg">
</p>

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant l'axe de glissement exprimé dans la base de `s1`
- `d1`, float : Distance algébrique entre l'origines de `s1` et l'axe de glissement
- `p2`, tuple : Coordonnée du point de pivot dans le repère de `s2`

Il est constamment possible de changer les paramètres initiaux en renseignant les attribus du même noms.

# Kinematic

- `point`, 2darray : Coordonnées des points de la sphère successives exprimées dans le système de coordonnées global
- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`
- `value`, 1darray : Valeurs successives de la distance algébrique du centre de la shpère par rapport à l'origine de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement et la valeur de glissement est exprimé. `__repr__` permet de savoir quel solide agit sur l'autre.

# Static/dynamic

`s1` est la référence : c'est sur lui que les efforts sont appliqué

- `force`, 1darray : Expression des efforts normaux à l'axe de glissement successifs qu'exerce `s2` sur `s1`
