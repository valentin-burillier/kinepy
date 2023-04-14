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

&nbsp;&nbsp;&nbsp;

| <img width=50/> Noms <img width=50/> | <img width=150/> Représentation <img width=150/> |
| :---: | --- |
| `cylinder` | ![Gear_dyn](https://user-images.githubusercontent.com/93446869/231987610-fa7bb8c6-f9e2-4df1-aa7f-3a058626da1b.svg) |
| `hollow_cylinder` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
| `round_rod` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
| `round_pipe` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
| `parallepiped` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
| `rectangular_tube` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
| `ball` | ![Gear_params](https://user-images.githubusercontent.com/93446869/231987677-8fe7490f-0140-4bc4-aa92-b39dc0b2d892.svg) |
