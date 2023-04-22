Ce fichier montre les fonctionnalités et la manière d'utiliser kinepy à travers un cas réel d'utilisation. 

# Sommaire

- [Présentation du système](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#présentation-du-système)
- [Modélisation du mécanisme](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#modélisation-du-mécanisme)
- [Ajouts d'efforts extérieurs](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#ajouts-defforts-extérieurs)
- [Simulation du mécanisme](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#simulation-du-mécanisme)
- [Affichage du mécanisme](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#affichage-du-mécanisme)
- [Récupération de données cinématiques](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#récupération-de-données-cinématiques)
- [Récupération de données sur les efforts internes](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#récupération-de-données-sur-les-efforts-internes)
- [Optimisation de paramètres](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md#optimisation-de-paramètres)

# Présentation du système

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/189538731-1ecda8fb-899e-4579-9a6d-002796fb4a15.png">
</p>

Le système de "lève-vitre" est utiliser dans les portières de voiture pour déplacer la vitre. Le mécanisme "en ciseau" comme on peut le voir ci-dessus a été massivement utiliser jusque d'en les années 90. Il est depuis remplacer par un mécanisme fonctionnant avec des câbles.

Nous allons nous intéresser à : 
- La loi entrée/sortie du mécanisme. C'est-à-dire trouver la relation liant la hauteur de la vitre et l'angle du moteur.
- Dimmensionner le ressort permettant à la vitre de ne pas tombe et réduire l'effort pour la déplacer.

# Initialisation de l'environement de travail

On commence par importer les bibliothèques nécessaires. `units` comporte les éléments `LENGTH`, `ANGLE`... correspondant à des chaînes de caractère `'Length'`, `'Angle'`... Les utiliser évitent les erreurs d'orthographes et permet de coder rapidement avec la complétion automatique. 

```python
import kinepy as k
from kinepy.units import *
import kinepy.tools as to

import numpy as np
import matplotlib.pyplot as plt
```

On choisit et on vérifie les unités avec lesquelles on va travailler. Il est possible de les changer avec les fonctions `set_unit_system()` et/ou `set_unit()` ([doc units](https://github.com/valentin-burillier/kinepy/blob/main/docs/units.md)) :

```python
set_unit_system(DEFAULT_SYSTEM)
set_unit(SPEED, MILLIMETER_PER_SECOND)
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
Density              : g/cm³
Mass                 : kg
Force                : N
Torque               : N.m
SpringConstant       : N/m
Inertia              : kg.m²
dimensionless        : No Unit
```

# Modélisation du mécanisme

On considère le schéma cinématique du lève-vitre suivant :

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231535482-95562610-c59b-4ea4-958c-6272ee5c9cb6.svg">
</p>

On se place dans les hypothèses de KinePy. On créé un système et l'on ajoute tous les solides qui compossent le mécanisme ([doc ajout de solide](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-solide)). On approxime l'inertie des solides : la roue d'entrée est concidéré comme un cylindre et les deux bras comme des parallélépipèdes ([doc calcul de masse et d'inertie](https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#calcul-de-masse-et-dinertie)).

```python
rA, rB, l = 7, 70, 130

sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.2, j=to.cylinder_inertia(2*rA, 0.2))
s2 = sys.add_solid('Main arm', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s3 = sys.add_solid('Secondary arm', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))
```

On peut également affiché la nomenclature du système. Cela est utile lorsque le système comporte un nombre important de solide.

```python
sys.bill_of_materials()
```
```
N°	| Names
--------------------
0	| Ground
1	| Gear wheel
2	| Main arm
3	| Secondary arm
4	| Glass
```

On intègre chaque liaison au mécanisme avec un paramétrage correspondant au schéma cinématique ([doc ajout de liaisons](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-liaisons)).

Le mécanisme est composé d'un train d'engrenage simple. Cet relation lie le mouvement de la liaison `r1` à celui de `r2`. On concidère un angle de pression de 20° et l'on met une valeur initiale de l'angle à 180° pour faciliter le pilotage ([doc engrenage](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#ajout-de-relations)).

```python
r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2, p1=(0, 0))
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

On peut maintenant établir les stratégies de résolution cinématique et dynamique ([doc compilation](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md#compilation)).

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

# Ajouts d'efforts extérieurs

Le système est soumis à la gravité. De plus, on ajoute des forces s'appliquant de chaque coté de la vitre modélisant les frottements de celle-ci avec le joint d'étanchéité.

```python
sys.add_gravity()

s4.add_force((0, -2), (0, 0))
s4.add_force((0, -2), (-2*l, 0))
```

# Simulation du mécanisme

On réalise une simulation de remonté de la vitre. La durée de simulation est de 3 s où 1001 points sont simulés. 

On commande le systeme par un trapèze de vitesse où la vitesse maximale du moteur est de 6 rad/s ([doc entrées cinématiques](https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#entrées-cinématiques)).

Afin de vérifier notre entrée, on affiche ce trapèze de vitesse en calculant la dérivée ([doc dérivées](https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#derivées)). On utilise `get_unit` pour obtenir l'unité des grandeurs physiques. Cela évite les ereurs d'unité.

```python
t, n = 3, 1001

time = np.linspace(0, t, n)
angle = to.trapezoidal_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6, phy=ANGLE)

plt.grid()
plt.plot(time, to.derivative(angle, t, phy=ANGLE))
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Input angular velocity (in {get_unit(ANGULAR_VELOCITY)})')

plt.show()
```

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/233799284-6ac054c3-9fb5-4b8c-b265-ef65d35945b3.png">
</p>

# Affichage du mécanisme

Ce paragraphe va changer quand les schémas cinématiques animées seront fonctionnel. Pour l'instant, on utilise la fonction `animate` de tools basée sur la fonction FuncAnimation de matplotlib. `animate` prend en argument une liste de liste de point. On obtient ces points soit en demandant le point de la liaison, soit l'origine du repère d'un solide ou soit avec la méthode `get_point`.

```python
_ = to.animate([[s2.get_point((-rB, 0)), ps2.point, r4.point, ps1.point], [s1.get_point((-rA, 0)), s1.origin]])
plt.show()
```

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/233808218-0c52631e-2058-4273-a98b-4561d62d450d.gif">
</p>

# Récupération de données cinématiques

On affiche l'évolution temporelle de la hauteur de la vitre ainsi que ça vitesse au court du temps. Pour ce faire, on récupère l'information de hauteur en prenant l'ordonnée du repère attaché à la vitre ([doc solide](https://github.com/valentin-burillier/kinepy/blob/main/docs/objets/Solid.md#cinématique)). On en prend la dérivée pour obtenir la vitesse ([doc dérivées](https://github.com/valentin-burillier/kinepy/blob/main/docs/tools.md#derivées)).

```python
glass_heigth = s4.origin[1]
glass_speed = to.derivative(glass_heigth, t)

plt.subplot(2, 1, 1)
plt.grid()
plt.plot(time, glass_heigth)
plt.ylabel(f'Glass height (in {get_unit(LENGTH)})')

plt.subplot(2, 1, 2)
plt.grid()
plt.plot(time, glass_speed)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Glass speed (in {get_unit(SPEED)})')

plt.show()
```

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/233808355-b26b3f9f-88ae-4a06-9ddb-5fd71f79ef0c.png">
</p>

On peut vérifier certaines exigences à propos de longueur de course et de vitesse maximale de la vitre.

```python
print('Glass stroke :', round(np.max(glass_heigth) - np.min(glass_heigth)), get_unit(LENGTH))
print('Maximum glass speed :', round(np.nanmax(glass_speed)), get_unit(SPEED))
```
```
Glass stroke : 368 mm
Maximum glass speed : 156 mm/s
```

# Récupération de données sur les efforts internes

Afin de dimensionner le moteur, on affiche l'évolution du couple d'entrée au cours du temps. Pour ce faire, on récupère l'information de couple au niveau de la liaison ([doc pivot](https://github.com/valentin-burillier/kinepy/blob/main/docs/objets/Revolute.md#sorties)). Il y a un signe "-" car `r1.torque` correspond au couple exercé par le bras sur le bâti. Or, c'est l'opposée qui nous intéresse.

```python
motor_torque = -r1.torque

plt.grid()
plt.plot(time, motor_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Motor torque (in {get_unit(TORQUE)})')

plt.show()
```

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/233808383-bc2d18d0-cf35-49fe-bfe4-500d77e7de3d.png">
</p>

On peut analyser plusieurs éléments sur ce graphique :
- discontinuité car discontinuité de l'accélération du au trapèze de vitesse
- les effets d'inertie ne s'appliquent que sur la phase d'accélérations et de décélération
- le maximum est atteint vers la moitié de la course
- le couple > 0 <=> le systeme combat en permanance le poid des pièces
- Le couple au début et plus faible qu'à la fin : les effets d'inertie s'oppose à la rotation du moteur au début et l'aide à la fin 

```python
print('Maximum motor torque :', round(np.nanmax(motor_torque), 2), get_unit(TORQUE))
```
```
Maximum motor torque : 0.61 N.m
```

# Optimisation de paramètres

```python
crank_torque = motor_torque*rB/rA

k = (crank_torque[1] - crank_torque[-2])/(r2.angle[1] - r2.angle[-2])
a0 = np.nanmean(crank_torque)/k

print('Torsion spring stiffness constant :', round(k, 2), f'{get_unit(TORQUE)}/{get_unit(ANGLE)}')
print('Preload angle :', round(a0, 1), get_unit(ANGLE), '-->', round(get_value(ANGLE)*a0/2/np.pi, 1) , 'r')
```
```
Torsion spring stiffness constant : 0.21 N.m/rad
Preload angle : 25.2 rad --> 4.0 r
```

le "-" de la formule précédente est enlevé car cela correspond au couple de la manivelle sur le bâti

```python
r2.set_torque(lambda : k*(r2.angle - np.pi + a0))
sys.solve_dynamics(angle, t)
```

```python
motor_torque = -r1.torque

plt.grid()
plt.plot(time, motor_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Motor torque (in {get_unit(TORQUE)})')

plt.show()
```

<p align="center" width="100">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/233808602-275c2b79-aa10-4499-a2f0-361c203770e9.png">
</p>

```python
print('Maximum motor torque :', round(np.nanmax(np.abs(motor_torque)), 2), get_unit(TORQUE))
```
```
Maximum motor torque : 0.1 N.m
```



