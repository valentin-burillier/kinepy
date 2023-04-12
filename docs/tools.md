# Géométrie

- `distance(p1, p2)` : Retourne la distance entre un point `p1` et `p2` deux tableau de forme (2,...)
- `norm(v)` : Retourne la norme du vecteur `v` correspondant à un tableau de forme (2,...)

# Derivées

- `get_speed(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n-1).
- `get_acceleration(p, t)` : Retourne les vecteurs vitesses du point `p`. L'argument `p` correspond à un tableau de forme (2, n). La sortie est un tableau de forme (2, n-2).
- `get_velocity_vector(p, t)` :
- `get_acceleration_vector(p, t)` :
- `get_angular_velocity(a, t)` :
- `get_angular_acceleration(a, t)` :

# Entrées cinématiques

- `direct_input(a, b, t, n=101, v_max=None)` :
- `trapezoidal_input(a, b, t, n=101, v_max=None, a_max=None)` :
- `sinusoidal_input(a, b, t, n=101, v_max=None, a_max=None)` :

# Calcul de masse/inertie

[mettre tableau]
