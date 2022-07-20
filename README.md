# kinepy

Cette librairie python est dédier à la simulation de mécanisme plan.

Elle permet de simuler rapidement un mécanisme complexe constituer de plusieurs boucle cinématique. Il est facile d'évaluer ses performances que se soit du point de vu de la position, de la vitesse des différents solides ou encore des efforts que subissent les pièces. Ainsi, cet outil peut être utilisé pour réaliser le premier dimensionnement d'un mécanisme.

La bibliothèque suit les hypothèses suivantes :
- Le mecanisme est plan
- Les solides sont indéformables
- La masse des solides est constante
- Les liaisons sont parfaites
- Le shéma cinématique du mécanisme est minimal
- Le référentiel global associé au bâti est galiléen

Il est important d'aller voir ce [fichier](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md) pour savoir comment réaliser la simulation d'un mécanisme. La [documentation](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md) de la bibliothèque est aussi accessible. 

# Installation

Installez la bibliothèque "kinepy" comme cela :
```bash
pip install kinepy
```
# Exemples

applications stylées à réaliser

# TODO

- debuger compile
- tester cinématique
- coder les cas statique
- coder engrenages/RSG

# Links

- https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - ce qui a inspiré ce répertoire
- http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - simulateur existant
