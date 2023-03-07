# Contexte

[Photo de la maquette étudier et du système réel]

Blablabla les portails c'est cool

# Problématique

CDCF :
- tps ouverture
- vitesse max du moteur / portail
- accélération max moteur / portail

On cherche à visualiser l'impact du changement de longueur entre les liaisons sur la cinématique du système ? On déterminera la loi E/S entre la rotation du moteur et la rotation du ventail.

Après avoir fixer le paramétrage, on cheche à dimensionner le motoréducteur par plusieurs critères à comparé avec le CDCF :
- Type de pilotage adapté (direct, trapèze, en S)
- Le couple nécessaire à l'ouverture
- Energie consommée durant une phase d'ouverture
Cette 2ième analyse permettra de dimensionner un limiteur couple 

# Hypothèses et modélisation du système

On considère le schéma cinématique suivant :

[Shéma]

On se place dans les hypothèses de KinePy.
Les masses et inerties des pièces sont négligées à part celle du ventail que l'on modélise comme un paralépipède rectangle.

```python
import numpy as np
import matplotlib.pyplot as plt
import kinepy as k
import kinepy.tools as t

a, b, c, d, e, f = 100, 200, 30, 260, 280, 800

sys = k.System()

s1 = sys.add_solid(name='Ventail', m=50, g=(f/2, 0))
s2 = sys.add_solid(name='Bras')
s3 = sys.add_solid(name='Bielle')
```

On néglige les frottements au niveau des liaisons mise à part celle du portail par rapport au bâti que l'on modélise comme un couple de frotement sec.

```python
sys = k.System()

s1 = sys.add_solid(name='ventail', m=50, j=,  g=(f/2, 0))
s2 = sys.add_solid(name='bras')
s3 = sys.add_solid(name='bielle')

r1 = sys.add_revolute(0, 1, p1=(a, 0))
r2 = sys.add_revolute(1, 2, p1=(d, c), p2=(e, 0))
r3 = sys.add_revolute(0, 3, p1=(0, b))
r4 = sys.add_revolute(3, 2, p1=(e, 0))

r1.set_torque(1)
```

# Lois E/S

On souhaite réaliser une ouverture à 90° du portail.
Pour tracer la loi E/S, on pilote donc cinématiquement l'angle d'ouverture du portail entre 0 et 90°. 
Le mécanisme est finit d'être paramétré, on demande à KinePy d'établir la stratégie de résolution.

```python
sys.pilot(r1)

sys.compile()
```

Le système présente une boucle cinématique signée. De part la cohérence des résultats trouver avec ce qui est attendu, on détermine le signe.

```python
sys.change_signs({'1 RRR':-1}}

time = 5
input = t.direct_input(0, np.pi/2, time, 101)
sys.solve_kinematics(input)
H = s1.get_point((d, 0))
P = s1.get_point((f, 0))
```

En récupérant les informations d'intérets, on affiche la loi E/S du mécanisme.

```python
plt.title('Loi E/S')
plt.plot(r3.angle*180/np.pi, r1.angle*180/np.pi)
plt.xlabel('Angle de rotation du moteur')
plt.ylabel('Angle de rotation du ventail')
```
<p align="center" width="100%">
    <img width="40%" src="https://user-images.githubusercontent.com/93446869/223567649-d2c4ed08-a72d-4b5f-8906-363dd8c49f95.png">
</p>

En faisant varier les longueurs interliaisons on change la cinématique du mécanisme.

[Code variation + plot]

[Ccl]

# Dimensionnement du moteur

Grace à l'étude précédente, on choisit le paramétrage suivant :

[Le paramétrage]

Grace à la simulation précédente, on obtient les valeurs limites d'angle à laquelle le moteur doit aller.
On change la liaison pilotée du mécanisme afin de pouvoir configurer une entrée particulière pour le moteur.

```python
sys.pilot(r3)

sys.compile()

sys.change_signs({'1 RRR':1}}
```

On affiche le couple moteur selon les différentes entrées.

[Code + courbe]

[Commentaires]
On obtient ensuite l'énergie consommé durant l'ouverture en réalisant l'intégrale du produit vitesse de rotation et couple.

[Code + courbe puissance]

[Ccl]
