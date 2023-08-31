# Schéma cinématique

<p align="center" width="100%">
    <img width="80%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/a7faddd1-3816-4345-8965-eea9d503379c">
</p>

## Afficher un mécanisme

- `system(sys)` : Renseigne le system `sys` à afficher 
- `show()` : Montre l'animation

## Options d'animation

- `animation_time(t)` : Configure la durée `t` d'animation
- `frames_of_reference(value=True)` : Configure l'affichage des repères associés à chaque solide
- `grid(value=True)` : Configure l'affichage de la grille
- `graduation(value=True)` : Configure l'affichage des graduations
- `figure_size(shape)` : Configure la taille de la fenêtre
- `ligth_mode()` : Configure le mode clair
- `dark_mode()` : Configure le mode sombre
- `add_point(solid, point, trace=True, speed=False)` : Ajoute un point à tracer dans l'animation. L'argument `point` correspond aux coordonnées du point à afficher dans le repère de `solid`. L'argument `trace` permet d'afficher la trace du point et `speed` affiche le vecteur vitesse associer à ce point.

## Enregistrer

- `save(name)` : Enregistre l'animation sous le noms `name` au format gif
