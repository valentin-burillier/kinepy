# Introduction

Chaque solide est associé à un unique repère orthonormé direct permettant de définir les positions relatives des différentes liaisons liant les solides entre eux. 
Ce système de coordonnées est représenté ci-dessous. Par défaut, toutes les longueurs/coordonnées sont exprimées en mm et les angles en radians. 

# Parameters
## Nomenclature

- `name`, str : Nom du solide. S'il n'est pas renseigné, un nom générique lui sera attribué.
- `rep`, int : Numéro de repérage du solide. Celui du bâti est forcément 0. `rep` incrémente de 1 à chaque ajout de solide dans un système.

## Paramètres physiques

- `m`, float : Masse du solide en kg. Par défaut, 0. 
- `j`, float : L'inertie du solide en kg.m². Par défaut, 0. 
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
