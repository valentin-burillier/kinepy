# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180998717-29c55798-3157-4605-a70c-5fc98c37d6d7.svg">
</p>

Le paramétrage de la glissière s'effectue grâce à ces 6 attributs :

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
