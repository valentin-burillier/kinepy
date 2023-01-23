# Paramètres

*Dans les différentes représentations ci-dessous, une sphère fait contact au plan en réalité le contact est réalisé en un seul point (ie le rayon de la sphère est nul)*

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189603588-7434f2c8-9ec9-4c88-b530-99f3d8453576.svg">
</p>

Le paramétrage d'une sphère-plan s'effectue grâce à ces 5 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant l'axe de glissement exprimé dans la base de `s1`
- `d1`, float : Distance algébrique entre l'origine de `s1` et l'axe de glissement
- `p2`, tuple : Coordonnée du point de contact dans le repère de `s2`

Il est constamment possible de changer les valeurs de `a1`, `d1` et `p2` même après la compilation du mécanisme. 

# Cinématique

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189603637-0d328c0e-7c1f-4e5d-837a-b773e0be251d.svg">
</p>

- `point`, 2darray : Coordonnées successives du point de contact exprimées dans le système de coordonnées global
- `angle`, 1darray : Valeurs successives de l'angle de `s2` par rapport à `s1`
- `sliding`, 1darray : Valeurs successives de la distance algébrique du centre de la sphère par rapport à l'origine de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement et la valeur de glissement sont exprimés. Le pilotage d'une liaison sphère-plan permet de fixer les attributs `angle` et `sliding`. Il n'est pas possible de piloter qu'une seule des deux variables.

# Actions mécaniques internes

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189603654-a06bfa81-da29-488d-80c0-e93e4900ca51.svg">
</p>

# Entrées

- `set_torque(t)` : Définis un couple additionnel `t` exercé par `s2` sur `s1` au niveau du point de contact. `t` peut soit être de type int/float représentant couple constant, soit un tableau (1darray) de valeur du couple à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés.
- `set_tangent(t)` : Définis un effort tangentiel additionnel `t` exercé par `s2` sur `s1` le long de l'axe de glissement. `t` peut soit être de type int/float représentant un effort constant, soit un tableau (1darray) de valeur de la force à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés.

# Sorties

- `normal`, 1darray : Efforts normaux successifs exercés par `s2` sur `s1` au niveau du point de contact
- `tangent`, 1darray : Efforts tangentiels successifs transmis par `s2` sur `s1` au niveau de la liaison lorsqu'elle est bloquée. Les valeurs prises par cet attribut ne correspondent pas à l'effort tangentiel défini par `set_tangent` : lorsque la liaison n'est pas bloquée, le couple transmis est donc toujours nul même si un effort tangent est ajouté avec `set_tangent`.
- `torque`, 1darray : Couples successifs transmis par `s2` sur `s1` au niveau de la liaison lorsqu'elle est bloquée. Les valeurs prises par cet attribut ne corespondent pas au couple défini par `set_torque` : lorsque la liaison n'est pas bloquée, le couple transmis est donc toujours nul même si un couple est ajouté avec la méthode `set_torque`.
