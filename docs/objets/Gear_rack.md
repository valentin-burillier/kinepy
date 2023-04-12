# Paramètres

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231422979-437ff5aa-2cd9-40e0-9eaf-da4df95c9750.svg">
</p>

Le paramétrage d'un engrenage s'effectue grâce à ces 5 attributs :
- `rev`, RevoluteJoint : Liaison pivot à accoupler
- `pri`, PrismaticJoint : Liaison glissière à accoupler
- `r`, float : Rayon de la roue dentée
- `v0`, float : Valeur du déphasage entre `rev` et `pri` quand l'angle de `rev` vaut zéro. Par défaut, `v0` vaut zéro.
- `pressure_angle`, float : Valeur de l'angle de pression entre les roues dentées. Par défaut, il est de 20° (ou pi/9 rad). (Voir figure ci-dessous)

Les liaisons `rev` et `pri` doivent avoir un solide en commun. Ici, cela correspond au solide jaune.

# Cinématique

La relation cinématique que décrit `Gear` s'écrit : `pri.sliding = rev.angle x r + v0`. Attention, aux solides de référence de chacune des liaisons mise en argument. En effet, cela change les signes de l'angle et du glissement de ces dernières. Il est donc important de bien vérifier le signe de `r` pour obtenir le mouvement souhaité. 

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231423172-743b630a-1d77-47ed-9c92-e7d102004338.svg">
</p>

- `contact_point`, 2darray : Coordonnées successives des points de contact exprimées dans le système de coordonnées global

# Actions mécaniques internes

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231423434-fcc25949-e16c-4add-ab65-e14c35a74776.svg">
</p>

- `contact_force`, 2darray : Forces successives exercées par la crémaillères sur la roue dentée au point de contact exprimées dans le système de coordonnées global
