# Paramètres

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985073-b8f29ecb-9536-444b-bb1d-f5c63a8198e6.svg">
</p>

Le paramétrage d'une pivot s'effectue grâce à ces 4 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `p1`, tuple : Coordonnées du point de pivot dans le repère de `s1`
- `p2`, tuple : Coordonnées du point de pivot dans le repère de `s2`

Il est constamment possible de changer les valeurs de `p1` et `p2` même après la compilation du mécanisme. 

# Cinématique

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985253-4236026d-37ac-4d80-8c5a-075d7e914bbe.svg">
</p>

- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`
- `point`, 2darray : Coordonnées successives des points de pivot exprimées dans le système de coordonnées global

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement est exprimé. Le pilotage d'une liaison pivot permet de fixer l'attribut `angle`.

# Actions mécaniques internes

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189607566-5f5080d2-9295-4178-85b6-dfeda7c667f5.svg">
</p>

## Entrées

- `set_torque(t)` : Définis un couple additionnel `t` exercé par `s2` sur `s1` au niveau du point de pivot. `t` peut soit être de type int/float représentant couple constant, soit un tableau (1darray) de valeur du couple à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés. Cela peut permettre de modéliser un ressort de torsion par exemple.

## Sorties

- `force`, 2darray : Forces successives exercées par `s2` sur `s1` au niveau de la liaison exprimées dans le système de coordonnées global
- `torque`, 1darray : Couples successifs transmis par `s2` sur `s1` au niveau de la liaison lorsqu'elle est bloquée. Les valeurs prises par cet attribut ne corespondent pas au couple défini par `set_torque` : lorsque la liaison n'est pas bloquée, le couple transmis est donc toujours nul même si un couple est ajouté avec la méthode `set_torque`.
