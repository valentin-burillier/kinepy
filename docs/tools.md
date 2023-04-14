# Géométrie

- `distance(p1, p2)` : Retourne la distance entre un point `p1` et `p2` deux tableau de forme (2,...)
- `norm(v)` : Retourne la norme du vecteur `v` correspondant à un tableau de forme (2,...)

# Derivées

- `get_speed(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n).
- `get_acceleration(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n).
- `get_angular_velocity(a, t)` :
- `get_angular_acceleration(a, t)` :

# Entrées cinématiques

- `direct_angular_input(a, b, t, n=101, v_max=None)` ou `direct_linear_input(a, b, t, n=101, v_max=None)` :
- `trapezoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None)` ou `trapezoidal_linear_input(a, b, t, n=101, v_max=None, a_max=None)` :
- `sinusoidal_angular_input(a, b, t, n=101, v_max=None, a_max=None)` ou `sinusoidal_linear_input(a, b, t, n=101, v_max=None, a_max=None)` :

# Calcul de masse/inertie

Les méthodes suivantes permettent de calculer la masse et l'inertie de solides simples. La liste de ces derniers est ci-dessous avec leur paramétrage correspondant. 

Pour calculer la masse d'un solide, on fait appel à la méthode ayant pour noms la forme que l'on veut suivit de `_mass` (par exemple : `round_rod_mass`). On renseigne en argumants les dimensions du solide et sa masse volumique.

Pour calculer l'inertie d'un solide selon l'axe schématisé, on fait appel à la méthode ayant pour noms la forme que l'on veut suivit de `_inertia` (par exemple : `parallepiped_inertia`). On renseigne en argumants les dimensions du solide et sa masse.

| <img width=100/> Noms <img width=100/> | <img width=100/> Représentation <img width=100/> |
|        :---:       | :---: |
|    `cylinder`      | ![cylinder](https://user-images.githubusercontent.com/93446869/232147341-17776847-6ea3-4ec4-89e9-185157684ca1.svg) |
|  `hollow_cylinder` | ![hollow cylinder](https://user-images.githubusercontent.com/93446869/232147311-2c8b0fee-4061-4c19-a8f9-8deeb27f6d97.svg) |
|    `round_rod`     | ![round rod](https://user-images.githubusercontent.com/93446869/232147255-b0685ef5-21f4-454b-86d2-70944f4c4903.svg) |
|    `round_pipe`    | ![round pipe](https://user-images.githubusercontent.com/93446869/232147235-1976ae77-7ae8-40a2-a44d-e1b8e91789d0.svg) |
|   `parallepiped`   | ![parallelepiped](https://user-images.githubusercontent.com/93446869/232147210-9f64bca7-fa55-4fd7-85f4-13264d450202.svg) |
| `rectangular_tube` | ![rectangular tube](https://user-images.githubusercontent.com/93446869/232146376-0b1a5482-2651-4c9a-b57c-989663e62d44.svg) |
|       `ball`       | ![ball](https://user-images.githubusercontent.com/93446869/232146496-b61f1f98-735b-408f-a59f-4759155a2c0b.svg) |

