# Animation

- `animate(list_paths, list_vectors=None, anim_time=4, repeat=True, scale=1, vector_scale=0.1, magnitude_vector='proportionnal')` : Crée une animation matplotlib d'une durée `anim_time`. L'argument `list_paths` est une liste de liste de points chaque liste de point vont etre tracée. L'argument `list_vectors` est une liste de tuple composée de seux élément le point d'application du vecteur et le vecteur. `vector_scale` gère l'échelle des vecteur par rapport à l'echelle du graphique. `magnitude_vector` est une chaine de caractère parmi : 'proportionnal', 'log', 'unitary'. `scale` permet de gérer la taille de la fenêtre. `repeat` définit si l'animation est joué en boucle.

# Géométrie

- `distance(p1, p2)` : Retourne la distance entre un point `p1` et `p2` deux tableau de forme (2,...)
- `norm(v)` : Retourne la norme du vecteur `v` correspondant à un tableau de forme (2,...)
- `to_cartesian(r, a)` : Retourne les coordonnées cartésienne d'un point `(r, a)` en coordonnée polaire

# Derivée

- `get_speed(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n-1).
- `get_speed_accurate(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). Les valeurs retournées sont plus précises que la méthode `get_speed` mais la sortie est un tableau de forme (2, n-2).
- `get_acceleration(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n-2).

# Entrées cinématiques

- `direct_input(a, b, t, n=101, v_max=None)` :
- `trapezoidal_input(a, b, t, n=101, v_max=None, a_max=None)` :
- `sinusoidal_input(a, b, t, n=101, v_max=None, a_max=None)` :

# Calcul d'inetie

- `j_parallelepiped(L, l, e, m)` :
- `j_bar(L, r, m)` :
- `j_cylinder(r, e, m)` :
- `j_ball(r, m)` :
