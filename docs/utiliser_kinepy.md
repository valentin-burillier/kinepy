Ce fichier montre les fonctionnalités et la manière d'utiliser kinepy à travers un cas réel d'utilisation. 

# Présentation du système

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189538731-1ecda8fb-899e-4579-9a6d-002796fb4a15.png">
</p>

Le système de "lève-vitre" est utiliser dans les portières de voiture pour déplacer la vitre. Le mécanisme "en ciseau" comme on peut le voir ci-dessus a été massivement utiliser jusque d'en les années 90. Il est depuis remplacer par un mécanisme fonctionnant avec des câbles.

Nous allons nous intéresser à : 
- La loi entrée/sortie du mécanisme. C'est-à-dire trouver la relation liant la hauteur de la vitre et l'angle du moteur.
- Dimmensionner le ressort permettant à la vitre de ne pas tombe et réduire l'effort pour la déplacer.

# Initialisation de l'environement de travail

On commence par importer les bibliothèques nécessaires 

```python
import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import numpy as np
import matplotlib.pyplot as plt
```

On vérifie les unités avec lesquelles on travaille. Si elles ne correspondent pas au unités voulues il est possible de les changer avec les fonctions `set_unit()` ou `set_unit_system()` ([doc units](https://github.com/valentin-burillier/kinepy/blob/main/docs/units.md)) :

```python
set_unit(SPEED, 0.001, unit='mm/s')
show_units()
```
```
Time                 : s
Length               : mm
Speed                : mm/s
Acceleration         : m/s²
Angle                : rad
Angular velocity     : rad/s
Angular acceleration : rad/s²
Mass                 : kg
Force                : N
Torque               : Nm
SpringConstant       : N/m
Inertia              : kg.m²
dimensionless        : No Unit
```

# Modélisation du mécanisme

On considère le schéma cinématique suivant :

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231535482-95562610-c59b-4ea4-958c-6272ee5c9cb6.svg">
</p>

On se place dans les hypothèses de KinePy. Les inerties des pièces sont négligées. Mais on concidère la masse de chaque solide. On créé un système dans lequel on a tous les solides qui compossent le mécanisme ([doc ajout de solide](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-solide)).

```python
sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.2)
s2 = sys.add_solid('Crank', m=0.5, g=(l, 0))
s3 = sys.add_solid('Support arm', m=0.5, g=(l, 0))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))
```

On intègre chaque liaison au mécanisme ([doc ajout de liaisons](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-liaisons)).

Le mécanisme est composé d'un train d'engrenage simple. Cet relation lie le mouvement de la liaison `r1` à celui de `r2`. On concidère un angle de pression de 20° et l'on met une valeur initiale de l'angle à 180° pour faciliter le pilotage ([doc engrenage](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-relations)).

```python
rA, rB, l = 7, 70, 130
r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2)
gear = sys.add_gear(r1, r2, -rA/rB, np.pi)
r3 = sys.add_revolute(2, 3, p1=(l, 0), p2=(l, 0))
r4 = sys.add_revolute(3, 4, p1=(2*l, 0))
ps1 = sys.add_pin_slot(0, 3)
ps2 = sys.add_pin_slot(4, 2, p2=(2*l, 0))
```

Le mécanisme est actionné par un moteur électrique au niveau de la pivot entre 1 et 0. On pilote donc cette liaison ([doc pilotage](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#pilotage-et-blocage-du-mécanisme)).

```
sys.pilot(r1)
```
```
Current input order:
(Rev(1/0): Angle)
```

On peut maintenant établir les stratégies de résolution cinématique et dynamique. 

```python
sys.compile()
```
```
Compiling...

Step 1:
Solved inputs

Step 2:
Solved relation <kinepy.objects.relations.Gear object at 0x0000021D08A9C490>

Step 3:
Identified new signed group RRP (n°1) with joints PinSlot(3/0), Rev(3/2).
Chosen 1 as sign.
Sign key: 3 RRP.

Step 4:
Identified new signed group RRP (n°1) with joints PinSlot(2/4), Rev(4/3).
Chosen 1 as sign.
Sign key: 4 RRP.

Compiling done.

Current signs :
3 RRP: 1
4 RRP: 1
```

Le système présente 2 boucles cinématiques signées. Pour déterminer les bons signes, on lance la suite du programme permettant de visualiser le système. Ainsi, on peut  savoir de manière simple si le mécanisme correspond au schéma cinématique. Après cette étape, on se rend compte qu'il est nécessaire de les changer. Pour ce faire, on utilise `change_signs()` ([doc compilation](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#compilation)).

```python
sys.change_signs({'3 RRP':-1, '4 RRP':-1})
```

On réalise une simulation de remonté de la vitre. La durée de simulation est de 3 s où 101 points sont simulés. 

On commande le systeme par un trapèze de vitesse où la vitesse maximale du moteur est de 6 rad/s.

```python
t, n = 3, 101
time = np.linspace(0, t, n)
angle = to.trapezoidal_angular_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6)
sys.solve_kinematics(angle)
```

# Optimisation de paramètre
