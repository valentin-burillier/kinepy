# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/179937697-ef957a55-0fc3-4af7-ae4d-73a4e8240511.png">
</p>


- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `p1`, tuple : Coordonnée du point de pivot dans le repère de `s1`
- `p2`, tuple : Coordonnée du point de pivot dans le repère de `s2`

Il est constamment possible de changer les paramètres initiaux en renseignant les attribus du même noms.

# Kinematic

- `point`, 2darray : Coordonnées des points de pivot successifs exprimées dans le système de coordonnées global
- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement est exprimé. `__repr__` permet de savoir quel solide agit sur l'autre.

# Static/dynamic

`s1` est la référence : c'est sur lui que les efforts sont appliqué

- `force`, 2darray : Expression des efforts successifs qu'exerce `s2` sur `s1` au niveau de la liaison exprimées dans le système de coordonnées global

Le couple `C` peut etre : Soit flottant/entier exprimant un couple de frottement constant durant la simulation, soit un 1darray de valeur à chaque instant de la simulation, soit une fonction retournant un 1darray de valeur à chaque instant de la simulation pouvant dépendre de valeurs géométriques pas encore calculées. Cela peut permettre de modéliser l'effort d'un ressort par exemple : 

```python
C0, k = 2.4, 0.1
C = lambda : C0 - P.angle*k
```

- `applie_couple(C)` : applique un couple torsion `C` de `s2` sur `s1` au niveau de la liaison
