# Parameters

- `origin`, 2darray :
- `name`, str :
- `angle`, 1darray :
- `index`, int :
- `named_point`, dict :
- `m`, float :
- `I`, float :
- `G`, tuple :


- `add_physical_params(G=(0., 0.), m=0, I=0)` :
- `__setitem__(name)`, str :
- `__getitem__(name)`, str :
- `get_point` :

- `apply_external_force(F, p=(0., 0.))` :
- `apply_external_couple(C)` :

```pycon
>>> s1 = S.add_solid(name='Arm')
>>> s1
'Arm'
>>> s1['A'] = (5., 0.5) # Adding/Setting point 'A'
>>> s1['A']
(5., 0.5)

>>> s2 = S.add_solid(((0., 0.), (1., 0.), (0., 1.)), {'A': 0, 'B': 1, 'C': 2}, 'ABC-Triangle')
>>> s2
'ABC-Triangle'
>>> s2['B']
(1., 0.)
```

```pycon
>>> s1.physical_parameters(1., 0., (1., 1.)) # ajout/modification de parametres physiques
>>> s1.I = 0.5 # modification de l'inertie
>>> s1.m, s1.I
(1., 0.5)
>>> s1.G
(1., 1.)
>>> 
>>> g = 10.
>>> F = lambda : np.array([[0.]*101, [-s1.m*g]*101])
>>> s1.applie_external_force(F, (0., 1.)) # application d'une force sur le solide s1
```

```python
>>> # apres une simulation
>>> s1.get_point('A')
array([
>>> s1.angle
array([
>>> s1.origin
array([
```
