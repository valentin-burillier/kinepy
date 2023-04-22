# Géométrie

- `distance(p1, p2)` : Retourne la distance entre un point `p1` et `p2`. Chaque correponde à un tableau de forme (2, n).
- `norm(v)` : Retourne la norme du vecteur `v` correspondant à un tableau de forme (2, n)

# Derivées

- `derivative(obj, t, phy='Length')` : Retourne la dérivée de l'argument `obj` sur la durée `t`. Le tableau retourné a le même format que le tableau `obj` en entrée. La dernière valeur du tableau est compléter par `nan`. L'argument `phy` qualifie la grandeur physique de `obj`. Par défaut, `phy` correspond à une longueur. On peut également le remplacer par `Angle`. Cette méthode permet de calculer le vecteur vitesse d'un point, la vitesse de glissement ou une vitesse de rotation...
- `second_derivative(obj, t, phy='Length')` : Retourne la dérivée seconde de l'argument `obj` sur la durée `t`. Le tableau retourné a le même format que le tableau `obj` en entrée. La première et la dernière valeur du tableau sont compléter par `nan`. L'argument `phy` qualifie la grandeur physique de `obj`. Par défaut, `phy` correspond à une longueur. On peut également le remplacer par `Angle`. Cette méthode permet de calculer le vecteur accélération d'un point, l'accélération de glissement ou une accélération de rotation...

# Entrées cinématiques

Les méthodes suivantes permettent la commande de liaisons pilotées. On y retrouve les entrées de type direct, trapèzoïdal et sinusoïdale.

- `direct_input(a, b, t, n=101, v_max=None, phy='Angle')` : Retourne un tableau de forme (n, ) correspondant à la liste de position d'une entrée de type direct allant de la valeur `a` à la valeur `b` en une durée `t` et avec `n` points d'échantillonage. Si la vitesse maximale `v_max` est dépassé la méthode le signalera et retournera `None`. Par défaut, il n'y a pas de vitesse maximale. L'argument `phy` qualifie la grandeur physique retournée. Par défaut, `phy` correspond à une angle. On peut également le remplacer par `Length`.
- `trapezoidal_input(a, b, t, n=101, v_max=None, a_max=None, phy='Angle')` : Retourne un tableau de forme (n, ) correspondant à la liste de position d'une entrée de type trapezoïdal allant de la valeur `a` à la valeur `b` en une durée `t` et avec `n` points d'échantillonage. La vitesse maximale de cette entrée est `v_max`. Par défaut, il n'y en a pas. Ainsi, l'entrée retournée correspond davantage à un triangle de vitesse. Si la vitesse maximale est trop faible ou si l'accélération maximale `a_max` est dépassé la méthode le signalera et retournera `None`. Par défaut, il n'y a pas d'accélération maximale. L'argument `phy` qualifie la grandeur physique retournée. Par défaut, `phy` correspond à une angle. On peut également le remplacer par `Length`.
- `sinusoidal_input(a, b, t, n=101, v_max=None, a_max=None, phy='Angle')` : Retourne un tableau de forme (n, ) correspondant à la liste de position d'une entrée de type sinusoïdale allant de la valeur `a` à la valeur `b` en une durée `t` et avec `n` points d'échantillonage. La vitesse maximale de cette entrée est `v_max`. Par défaut, il n'y en a pas. Ainsi, l'entrée retournée correspond davantage à sinus. Si la vitesse maximale est trop faible ou si l'accélération maximale `a_max` est dépassé la méthode le signalera et retournera `None`. Par défaut, il n'y a pas d'accélération maximale. L'argument `phy` qualifie la grandeur physique retournée. Par défaut, `phy` correspond à une angle. On peut également le remplacer par `Length`.

[mettre des im ages des entrées]

# Calcul de masse et d'inertie

Les méthodes suivantes permettent de calculer la masse et l'inertie de solides simples. La liste de ces derniers est ci-dessous avec leur paramétrage correspondant. 
- Pour calculer la masse d'un solide, on renseigne en argumants sa masse volumique `rho` et ses dimensions.
- Pour calculer l'inertie d'un solide selon l'axe schématisé, on renseigne en argumants sa masse `m` et ses dimensions.

| Noms | <img width=100/> Représentations <img width=100/> |
| :---: | :---: |
| `cylinder_mass(rho, d, h)` <br/> <br/> `cylinder_inertia(m, d)`    | ![cylinder](https://user-images.githubusercontent.com/93446869/232147341-17776847-6ea3-4ec4-89e9-185157684ca1.svg) |
|  `hollow_cylinder_mass(rho, d, h, t)` <br/> <br/> `hollow_cylinder_inertia(m, d, t)` | ![hollow cylinder](https://user-images.githubusercontent.com/93446869/232147311-2c8b0fee-4061-4c19-a8f9-8deeb27f6d97.svg) |
| `round_rod_mass(rho, d, l)` <br/> <br/> `round_rod_inertia(m, d, l)` | ![round rod](https://user-images.githubusercontent.com/93446869/232147255-b0685ef5-21f4-454b-86d2-70944f4c4903.svg) |
| `round_pipe_mass(rho, d, l, t)` <br/> <br/> `round_pipe_inertia(m, d, l, t)` | ![round pipe](https://user-images.githubusercontent.com/93446869/232147235-1976ae77-7ae8-40a2-a44d-e1b8e91789d0.svg) |
| `parallelepiped_mass(rho, l, w, h)` <br/> <br/> `parallelepiped_inertia(m, l, w)` | ![parallelepiped](https://user-images.githubusercontent.com/93446869/232147210-9f64bca7-fa55-4fd7-85f4-13264d450202.svg) |
| `rectangular_tube_mass(rho, l, w, h, t)` <br/> <br/> `rectangular_tube_inertia(m, l, w, h, t)` | ![rectangular tube](https://user-images.githubusercontent.com/93446869/232146376-0b1a5482-2651-4c9a-b57c-989663e62d44.svg) |
| `ball_mass(rho, d)` <br/> <br/> `ball_inertia(m, d)` | ![ball](https://user-images.githubusercontent.com/93446869/232146496-b61f1f98-735b-408f-a59f-4759155a2c0b.svg) |
