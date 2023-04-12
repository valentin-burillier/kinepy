Ce fichier montre les fonctionnalités et la manière d'utiliser kinepy à travers un cas réel d'utilisation. 

# Présentation du système

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189538731-1ecda8fb-899e-4579-9a6d-002796fb4a15.png">
</p>

Le système de "lève-vitre" est utiliser dans les portières de voiture pour déplacer la vitre. Le mécanisme "en ciseau" comme on peut le voir ci-dessus a été massivement utiliser jusque d'en les années 90. Il est depuis remplacer par un mécanisme fonctionnant avec des câbles.

Nous allons nous intéresser à : 
- La loi entrée/sortie du mécanisme. C'est-à-dire trouver la relation liant la hauteur de la vitre et l'angle du moteur.
- Dimmensionner le ressort permettant à la vitre de ne pas tombe et réduire l'effort pour la déplacer.

# Modélisation du mécanisme

On commence par importer les bibliothèques nécessaires 

```python
import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import numpy as np
import matplotlib.pyplot as plt
```

On vérifie les unités avec lesquelles on travaille. Si elles ne correspondent pas au unités voulues il est possible de les changer avec les méthodes `set_unit` ou `set_unit_system` :

```python
show_units()
```
```
Time                 : s
Length               : mm
Speed                : m/s
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

On considère le schéma cinématique suivant :

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231535482-95562610-c59b-4ea4-958c-6272ee5c9cb6.svg">
</p>

On se place dans les hypothèses de KinePy. Les inerties des pièces sont négligées. Mais on concidère la masse de chaque solide.

```python
sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.2)
s2 = sys.add_solid('Crank', m=0.5, g=(l, 0))
s3 = sys.add_solid('Support arm', m=0.5, g=(l, 0))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))
```

On intègre chaque liaison au mécanisme. Le mécanisme est actionné par un moteur électrique au niveau de la pivot entre 1 et 0.

```python
rA, rB, l = 7, 70, 130
r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2)
gear = sys.add_gear(r1, r2, -rA/rB, np.pi)
r3 = sys.add_revolute(2, 3, p1=(l, 0), p2=(l, 0))
r4 = sys.add_revolute(3, 4, p1=(2*l, 0))
ps1 = sys.add_pin_slot(0, 3)
ps2 = sys.add_pin_slot(4, 2, p2=(2*l, 0))

sys.pilot(r1)
```


# Paramétrages du mécanisme via kinepy

# Evaluation de performance

# Optimisation de paramètre
