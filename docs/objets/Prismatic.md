# Paramètres

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189606974-76c4e831-ec9c-410f-be80-dfce1439331a.svg">
</p>

Le paramétrage d'une glissière s'effectue grâce à ces 6 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s1`
- `a2`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s2`
- `d1`, float : Distance algébrique entre l'origine de `s1` et l'axe de glissement
- `d2`, float : Distance algébrique entre l'origine de `s2` et l'axe de glissement

Il est constamment possible de changer les valeurs de `a1`, `a2`, `d1` et `d2` même après la compilation du mécanisme. 

# Cinématiques

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189606990-39cba9d1-d43b-4ebf-86fb-8b87dc4e28de.svg">
</p>

- `sliding`, 1darray : Valeurs successives de la distance algébrique de l'origine de `s2` par rapport à celle de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que la valeur de glissement est exprimée. Le pilotage d'une liaison glissière permet de fixer l'attribut `sliding`.

# Actions mécaniques internes

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189607008-d5a6c649-da05-43b4-acc2-e5ecd1318bec.svg">
</p>

## Entrées

- `set_tangent(t)` : Définis un effort tangentiel additionnel `t` exercé par `s2` sur `s1` le long de l'axe de glissement. `t` peut soit être de type int/float représentant un effort constant, soit un tableau (1darray) de valeur de la force à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés. Cela peut permettre de modéliser un ressort au niveau de la glissière par exemple.

## Sorties

- `normal`, 1darray : Efforts normaux successifs exercés par `s2` sur `s1` au niveau de la liaison
- `torque`, 1darray : Couples successifs transmis par `s2` sur `s1` au point correspondant à la projection orthogonale de l'origine de `s1` sur l'axe de glissement (voir schéma). 
- `tangent`, 1darray : Efforts tangentiels successifs transmis par `s2` sur `s1` au niveau de la liaison lorsqu'elle est bloquée. Les valeurs prises par cet attribut ne correspondent pas à l'effort tangentiel défini par `set_tangent` : lorsque la liaison n'est pas bloquée, le couple transmis est donc toujours nul même si un effort tangent est ajouté avec `set_tangent`.
