# Paramètres

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231410339-ffbf9a8a-6f32-4105-9644-0a77480f5084.svg">
</p>

Le paramétrage d'un engrenage s'effectue grâce à ces 5 attributs :
- `rev1`, RevoluteJoint : Première pivots à accoupler
- `rev2`, RevoluteJoint : Deuxième pivots à accoupler
- `r`, float : Rapport de transmission. Cette valeur peu être négative.
- `v0`, float : Valeur du déphasage entre `rev1` et `rev2` quand l'angle de `rev1` vaut zéro. Par défaut, `v0` vaut zéro.
- `pressure_angle`, float : Valeur de l'angle de pression entre les roues dentées. Par défaut, il est de 20° (ou pi/9 rad). (Voir figure ci-dessous)

Les liaisons pivots `rev1` et `rev2` doivent avoir un solide en commun. Ici, cela correspond au solide jaune.

Les engrenages internes peuvent aussi être simulés. Le signe de `r` détermine quel type d'engrenage est simulé (voir le paragraphe suivant).

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/235699382-ee86aac3-a2bd-4676-a1df-5440793c688e.svg">
</p>

# Cinématique

La relation cinématique que décrit `Gear` s'écrit : `rev2.angle = rev1.angle x r + v0`. Attention, aux solides de référence de chacune des liaisons mise en argument. En effet, cela change les signes des angles de ces dernières. Il est donc important de bien vérifier le signe de `r` pour obtenir le mouvement souhaité. 

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231415588-9234c458-c906-400f-b8e6-3826bb23255e.svg">
</p>

- `contact_point`, 2darray : Coordonnées successives des points de contact exprimées dans le système de coordonnées global

# Actions mécaniques internes

<p align="center" width="100%">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/231416667-5da54427-1ad6-4e13-b63d-acb38bfea93a.svg">
</p>

- `contact_force`, 2darray : Forces successives exercées par la roue dentée 2 sur la roue dentée 1 au point de contact des roues exprimées dans le système de coordonnées global
