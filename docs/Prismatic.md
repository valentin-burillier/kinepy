```python
kinepy.Prismatic(s1, s2, a1=0., d1=0., a2=0., d2=0.)
```

# Parameters

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/171271772-efb7b502-cd72-466a-a873-c091fbf4da47.png">
</p>

- `s1`, int : L'indice du premier solide
- `s2`, int : L'indice du deuxième solide
- `a1`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s1`
- `a1`, float : Angle du vecteur dirigeant la glissière exprimé dans la base de `s2`
- `d1`, float : Distance algébrique entre l'origines de `s1` et l'axe de glissement
- `d2`, float : Distance algébrique entre l'origines de `s2` et l'axe de glissement

Il est constamment possible de changer les paramètres initiaux en renseignant les attribus du même noms.

# Kinematic

- `value`, 1darray : Valeurs successives de la distance algébrique de l'origine de `s2` par rapport à celle de `s1` le long de l'axe de glissement

`s1` est la référence : c'est par rapport à lui que la valeur de glissement est exprimée.

# Static/dynamic

`s1` est la référence : c'est sur lui que les efforts sont appliqué

- `force`, 1darray : Expression des efforts normaux à l'axe de glissement successifs qu'exerce `s2` sur `s1`

La force `F` peut etre : Soit flottant/entier exprimant un effort de frottement constant durant la simulation, soit un 1darray de valeur à chaque instant de la simulation, soit une fonction retournant un 1darray de valeur à chaque instant de la simulation pouvant dépendre de valeurs géométriques pas encore calculées. Cela peut permettre de modéliser l'effort d'un ressort par exemple : 

```python
l0, k = 0.2, 0.07
F = lambda : -(G.value - l0)*k
```

- `apply_force` : Applique une force `F` exercé par `s2` sur `s1` le long de l'axe de glissement
