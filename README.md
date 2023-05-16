<p align="center" width="100%">
    <img width="70%" src="https://user-images.githubusercontent.com/93446869/234020322-1ef75f2b-1900-4b86-a83c-96becd308c8c.svg">
</p>

<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/a7faddd1-3816-4345-8965-eea9d503379c">
</p>

[![PyPI Downloads](https://img.shields.io/pypi/dm/kinepy.svg?label=PyPI%20downloads)](https://pypi.org/project/kinepy/)

KinePy est une librairie python dédiée à la simulation de mécanisme plan.

- KinePy permet de simuler rapidement et précisément un mécanisme complexe constituer de plusieurs boucles cinématiques. Il est facile d'évaluer les performances d'un mécanisme que ce soit du point de vue de la position, de la vitesse des différents solides ou encore des efforts que subissent les pièces au niveau des liaisons. Cet outil est donc destiné pour réaliser le premier dimensionnement d'un mécanisme.

- KinePy a été conçu pour pouvoir être utilisé avec des algorithme d'optimisation. En effet, après la création d'un mécanisme, l'utilisateur à la possibilité de modifier les dimensions d'un solide sans recréer un nouveau système. Cela fournit un modèle paramètrable où il est possible de créer une fonction coût. Cette dernière pourra être minimiser avec des algorithme d'optimisation fournit par d'autres bibliothèques : [scikit-learn](https://scikit-learn.org/stable/), [scipy](https://scipy.org/)...

- De plus, KinePy permet de réaliser la cinématique inverse d'un mécanisme. En ajoutant des liaisons pilotés ne faisant pas partie du mécanisme, on peut trouver le pilotage d'actionneurs.  

# Pré-requis

Des connaissances en science de l'ingénieur sont nécessaire pour utiliser KinePy notament dans les domaines de la modélisation, la cinématique et la dynamique. La documentation de KinePy seul ne permet pas de comprendre ces domaines.

# Documentation

Nous vous invitons à aller voir ce [fichier](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md) montrant comment simuler un mécanisme. Il renvoit sur la documentation des éléments utiliser.

La documentation des éléments permettant de créer un mécanisme est [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/System.md).

# Exemples

Vous pourrez trouver [ici](https://github.com/valentin-burillier/kinepy/tree/main/examples) des exemples d'application de KinePy sur des systèmes réels.

- Simulation cinématique du bras d'une pelleteuse :
<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/86fb79ad-6ebd-40fd-b010-c2c279667e6f">
</p>

# Hypothèses

La bibliothèque suit un certain nombre d'hypothèses. Il est nécessaire que le mécanisme que vous voulez simuler les respecte :
- Le mécanisme est plan
- Le système est isostatique, le schéma cinématique doit être minimal
- Les solides sont indéformables
- Les liaisons sont parfaites
- La masse des solides est constante
- Le référentiel global associé au bâti est galiléen
- Le système est soumis un champ de forces uniformes

Des études plus approfondit peuvent être utile après l'utilisation de KinePy.

# Installation

KinePy est installable avec pip :
```
pip install kinepy
```

*Notez que la version télécharger avec pip peut être bien antérieur au dépot le plus récent. Des fonctionnalités peuvent manquer.*

# Disclaimer

La bibliothèque est en développement. Il peut donc y avoir des bugs. Merci de les faire remonter aux développeurs afin de les corriger. Actuellement, l'interface graphique est en train d'être codé. La représentation de schéma cinématique animé n'étant pas une tâche triviale, veuillez signaler si vous rencontrez des problèmes.

De plus, les dévelopeurs seront très attentifs aux demandes d'intégration de nouvelles fonctionnalités.

Nous vous encourageons également à partager aux développeurs les applications que vous faites de KinePy.

# Liens

- https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - Ce qui a inspiré ce répertoire
- http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - Simulateurs cinématiques existants
