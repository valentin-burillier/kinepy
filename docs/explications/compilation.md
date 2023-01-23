
# Fonctionnement

Une fois un système paramétré, il va pouvoir être complié avant d'être résolu cinématiquement puis dynamiquement/statiquement.
La compilation consiste en plusieurs étapes:
 - Transformation du système en graphe
 - Identification des groupes Assur
 - Mise en ordre des instructions de résolution (cinématique ou dynamique)

Les instructions ainsi obtenues indiquent et ordonnent les fusions de groupes Assur et la propagation des relations.

## Transformation en graphe
La compilation utilise 2 graphes:

 - Un graphe qui a pour sommets les solides et pour arêtes les liaisons
 - Un graphe qui a pour sommets les liaisons et pour arêtes les relations

Remarque: _L'hypothèse d'isostatisme dicte que le graphe des relations est une forêt. (Mon arrogance me pousse à vous 
annoncer que la preuve est laissée au lecteur en EXERCICE)_

Concernant le graphe des liaisons, les liaisons à 2 degrés de liberté sont décomposées en pivots et glissières 
accompagnées d'un solide dit _fantôme_ (aucune inertie) afin de limiter l'inventaire des groupes Assur.
Elles sont ensuite rétablies après résolution

### Exemple

## Identification des groupes Assur

c.f. [Algorithme](https://doi.org/10.1016/j.dam.2018.02.018) (pas encore implémenté)


## Mise en ordre

La compilation fonctionne par opérations sur les graphes qui correspondront par la suite à des opérations de résolution cinématique/dynamique

On considère comme classe un solide ou un groupement de plusieurs solides. Elles repésentent à chaque étape de la 
résolution cinématique les classes d'équivalence des solides dont la position relative est résolue.\
Une fusion de classes transforme ces classes en une seule et a le même impact sur le graphe c.f. [dessin]

Maintenant que les groupe Assur peuvent être identifiés, la compilation procède comme suit:
Chaque solide forme une classe à part entière initialement.
Les liaisons pilotées ou bloquées engendrent la fusion des 2 classes qu'elles lient.
Ensuite on vérifie leur relation avec d'autres liaisons, si c'est le cas, les liaisons en relation sont
désignées comme racines de leur arbre et le parcours en largeur pour fusioner les liaisons sur la route.
Ensuite tant qu'il y a plus d'une classe, on identifie un groupe Assur, on fusionne les classes qui le composent et on 
vérifie les relations des liaisons qui le composent

### Cinématique

piloté = injection valeur
fusion ~ même ref
groupe = résout le groupe


### Dynamique/Statique

Inertie (si dynamique) + cinématique[::-1]





# Inventaire des groupes Assur identifiés par kinepy

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833440-20233a5a-d9b6-4e36-816e-dfa08279a892.svg">
</p>
<p align="center">Fig 1 - Schéma cinématique et graphe du groupe Assur RRR</p> 


<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833434-f354ef88-d5ca-4c4e-a12e-0da1a91e91ae.svg" >
</p>
<p align="center">Fig 2 - Schéma cinématique et graphe du groupe Assur RRP</p>


<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833426-806d4ee1-256f-4b03-8def-7981018277df.svg">
</p>
<p align="center">Fig 3 - Schéma cinématique et graphe du groupe Assur PPR</p>

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833424-f443d051-61bf-4aba-9e33-a4a340e1fa27.svg">
</p>
<p align="center">Fig 4 - Schéma cinématique et graphe du groupe Assur 3-RR</p>

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833448-fa52e8f7-c5cb-495f-bb86-4392d5af8cdb.svg">
</p>
<p align="center">Fig 5 - Schéma cinématique et graphe du groupe Assur 2-RR-PP</p> 

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833450-2be1f2df-3037-46e9-a1c0-d8fcfd465413.svg">
</p>
<p align="center">Fig 6 - Schéma cinématique et graphe du groupe Assur 3-PR</p> 

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833446-c6d99b87-787d-4313-97b2-bbd40be7359b.svg">
</p>
<p align="center">Fig 7 - Schéma cinématique et graphe du groupe Assur 2-PR-RR</p>

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833431-9784618b-0ffa-4aed-9f0e-5f9447fb4cbd.svg">
</p>
<p align="center">Fig 8 - Schéma cinématique et graphe du groupe Assur PP-PR-RR</p> 

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833449-1d2fb8a0-d805-4ecf-bba7-84e3311b281a.svg">
</p>
<p align="center">Fig 9 - Schéma cinématique et graphe du groupe Assur 2-RR-PR</p> 

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833429-bab1fbda-894a-4dfd-90a9-6084b9d8056a.svg">
</p>
<p align="center">Fig 10 - Schéma cinématique et graphe du groupe Assur PP-PR-RP</p> 

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833444-c9628f7d-71c6-442d-8824-2b1d80f321aa.svg">
</p>
<p align="center">Fig 11 - Schéma cinématique et graphe du groupe Assur 2-PR-PP</p>

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833436-cb702767-aa21-4c11-89fc-4f184b607a01.svg">
</p>
<p align="center">Fig 12 - Schéma cinématique et graphe du groupe Assur RR-PR-RP</p>


<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833442-844cb4f8-8128-46a3-a727-99088a9994b9.svg">
</p>
<p align="center">Fig 13 - Schéma cinématique et graphe du groupe Assur 2-PR-RP</p> 
