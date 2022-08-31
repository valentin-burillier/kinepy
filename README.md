# kinepy

Not comfortable with French ? go [there](https://github.com/valentin-burillier/kinepy/blob/main/README_EN.md) for the English version. 

Cette librairie python est dédiée à la simulation de mécanisme plan.

Elle permet de simuler rapidement et précisément un mécanisme complexe constituer de plusieurs boucles cinématiques. Il est facile d'évaluer ses performances que ce soit du point de vue de la position, de la vitesse des différents solides ou encore des efforts que subissent les pièces au niveau des liaisons. Ainsi, cet outil peut être utilisé pour réaliser le premier dimensionnement d'un mécanisme. Il fournit des modèles couplables à des algorithmes d'optimisation afin de réaliser une conception de mécanisme. De plus, cela peut permettre de réaliser la cinématique inverse d'un mécanisme. Des études plus approfondies peuvent être nécessaires aux vues des hypothèses que la bibliothèque suit :

- Le mécanisme est plan
- Les solides sont indéformables
- La masse des solides est constante
- Les liaisons sont parfaites
- Le système est isostatique, le schéma cinématique doit être minimal
- Le système est composé uniquement de dyades (groupe Assur d'ordre 1)
- Le référentiel global associé au bâti est galiléen
- Le système est soumis un champ de forces uniformes (ou nul)

Il est important d'aller voir ce [fichier](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md) pour savoir comment réaliser la simulation d'un mécanisme. La [documentation](https://github.com/valentin-burillier/kinepy/blob/main/docs) de la bibliothèque est aussi accessible. 

# Installation

Installez la bibliothèque "kinepy" comme cela :
```bash
pip install kinepy
```
# Exemples

Applications stylées à réaliser

# TODO

- Tester les engrenages

# Liens

- https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - Ce qui a inspiré ce répertoire
- http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - Simulateurs cinématiques existants
