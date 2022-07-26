# kinepy

Cette librairie python est dédier à la simulation de mécanisme plan.

Elle permet de simuler rapidement et précisément un mécanisme complexe constituer de plusieurs boucle cinématique. Il est facile d'évaluer ses performances que se soit du point de vu de la position, de la vitesse des différents solides ou encore des efforts que subissent les pièces au niveau des liaisons. Ainsi, cet outil peut être utilisé pour réaliser le premier dimensionnement d'un mécanisme. Il fournit des modèles couplable à des algorithmes d'optimisation afin de réaliser une conception de mécanisme. Des études plus approfondies peuvent être nécessaire au vue des hypothèses que la bibliothèque suit :

- Le mecanisme est plan
- Les solides sont indéformables
- La masse des solides est constante
- Les liaisons sont parfaites
- Le schéma cinématique du mécanisme est minimal
- Le référentiel global associé au bâti est galiléen
- Le systeme est soumis un champs de force uniforme (ou nul)

Il est important d'aller voir ce [fichier](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md) pour savoir comment réaliser la simulation d'un mécanisme. La [documentation](https://github.com/valentin-burillier/kinepy/blob/main/docs) de la bibliothèque est aussi accessible. 

# Installation

Installez la bibliothèque "kinepy" comme cela :
```bash
pip install kinepy
```
# Exemples

Applications stylées à réaliser

# TODO

- Tester la dynamique/satatique
- Coder engrenages/RSG

# Links

- https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - Ce qui a inspiré ce répertoire
- http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - Simulateurs cinématiques existant
