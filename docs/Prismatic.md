# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180998717-29c55798-3157-4605-a70c-5fc98c37d6d7.svg">
</p>

Le paramétrage d'une glissière s'effectue grâce à ces 6 attributs :

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s1`
- `a2`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s2`
- `d1`, float : Distance algébrique entre l'origine de `s1` et l'axe de glissement
- `d2`, float : Distance algébrique entre l'origine de `s2` et l'axe de glissement

Il est constamment possible de changer les valeurs de `a1`, `a2`, `d1` et `d2` même après la compilation du mécanisme. 

# Kinematics

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180998713-5b020132-4f86-447d-b9d5-7e28a1be30cd.svg">
</p>

- `delta`, 1darray : Valeurs successives de la distance algébrique de l'origine de `s2` par rapport à celle de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que la valeur de glissement est exprimée. Le pilotage d'une liaison glissière permet de fixer l'attribut `delta`.

# Internal mechanical actions

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181913881-2f462197-1df0-4826-bf42-ef6c753074ef.svg">
</p>

- `set_tangent(t)` : Définis un effort tangentiel additionnel `t` exercé par `s2` sur `s1` le long de l'axe de glissement. `t` peut soit être de type int/float représentant un effort constant, soit un tableau (1darray) de valeur de la force à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés. Cela peut permettre de modéliser un ressort au niveau de la glissière par exemple.
- `normal`, 1darray : Efforts normals successifs exercés par `s2` sur `s1` au niveau de la liaison
- `torque`, 1darray : Couples successifs transmis par `s2` sur `s1` au point correspondant à la projection orthogonale de l'origine de `s1` sur l'axe de glissement (voir schéma). 
- `tangent`, 1darray : Efforts tangentiels successifs transmis par `s2` sur `s1` au niveau de la liaison lorsqu'elle est bloquée. Les valeurs prises par cet attribut ne correspondent pas à l'effort tangentiel défini par `set_tangent` : lorsque la liaison n'est pas bloquée, le couple transmis est donc toujours nul même si un effort tangent est ajouté avec `set_tangent`.
