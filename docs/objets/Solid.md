# Introduction

Chaque solide est associé à un unique repère orthonormé direct permettant de définir les positions relatives des différentes liaisons liant les solides entre eux. 
Ce système de coordonnées est représenté ci-dessous.

# Parameters
## Nomenclature

- `name`, str : Nom du solide. S'il n'est pas renseigné, un nom générique lui sera attribué.
- `rep`, int : Numéro de repérage du solide. Celui du bâti est forcément 0. `rep` incrémente de 1 à chaque ajout de solide dans un système.

## Paramètres physiques

- `m`, float : Masse du solide. Par défaut, 0. 
- `j`, float : L'inertie du solide. Par défaut, 0. 
- `g`, tuple : Coordonnées du centre d'inertie du solide confondu à celui de gravité. Par défaut, (0, 0).

En dehors de l'initialisation du solide, il est constamment possible de changer ces attributs.

# Kinematics

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181019881-8785a058-e1c5-452c-afc3-cc42e11b8e5d.svg">
</p>

Le repère orthonormé direct est qualifié par :

- `origin`, 2darray : Coordonnées successives de l'origine du repère associé au solide.
- `angle`, 1darray : Valeurs successives de l'angle du solide par rapport au système de coordonnées globales associées au bâti.

On peut également associer un point au solide pour obtenir les différentes positions de celui-ci lors d'une simulation.

- `get_point(p)` : Retourne les coordonnées successives du point `p` dans le système de coordonnées global associé au bâti. `p` ayant ses coordonnées exprimées dans le repère du solide et lié à celui-ci. 
Il peut soit être un tuple ou un tableau de forme (2,...) représentant une multitude de points.

# Actions mécaniques extérieurs

- `add_force(f, p)` : Ajoute une force `f` appliquée au solide au point `p`. `f` est exprimée dans le système de coordonnées globales alors que `p` est exprimé dans le repère local du solide. `f` peut soit être de type tuple/liste représentant un effort constant, soit un tableau (2darray) de valeur de la force à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés.
- `add_torque(t)` : Ajoute un couple `t` appliquée au solide. `t` peut soit être de type int/float représentant couple constant, soit un tableau (1darray) de valeur du couple à chaque instant de la simulation ou une fonction retournant l'un des types déjà décrits. Cette dernière a l'avantage de pouvoir dépendre de paramètres géométriques/cinématiques qui ne sont pas encore simulés.
