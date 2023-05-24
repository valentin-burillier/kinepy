<p align="center" width="100%">
    <img width="70%" src="https://user-images.githubusercontent.com/93446869/234020322-1ef75f2b-1900-4b86-a83c-96becd308c8c.svg">
</p>

<p align="center" width="100%">
    <img width="80%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/a7faddd1-3816-4345-8965-eea9d503379c">
</p>

[![PyPI Downloads](https://img.shields.io/pypi/dm/kinepy.svg?label=PyPI%20downloads)](https://pypi.org/project/kinepy/)

**KinePy est une librairie python dédiée à la simulation cinématique, dynamique et statique de mécanisme plan.** Elle est utilisée pour :

- `Evaluation de performences` : KinePy permet de simuler rapidement et précisément un mécanisme complexe constituer de plusieurs boucles cinématiques. Il est facile d'évaluer les performances d'un mécanisme que ce soit du point de vue de la position, de la vitesse des différents solides ou encore des efforts que subissent les pièces au niveau des liaisons.
- `Dimensionnement de composants` : En récupérant les informations de vitesse ou d'effort de certaines liaisons, il est possible de connaître les composants adéquates à utiliser. Par exemple, c'est utile dans le choix d'un moteur, d'un ressort, de roulements à bille, de paliers lisses...
- `Optimisation de mécanisme` : KinePy offre la possibilité de modifier les dimensions d'un solide sans recréer un nouveau système. Ainsi, un tel modèle peut être couplé à des algorithme d'optimisation afin de définir un mécanisme répondant à vos attentes. Il existe de nombreux algorithmes d'otimisation fournient par d'autres bibliothèques : [scikit-learn](https://scikit-learn.org/stable/), [scipy](https://scipy.org/)...
- `Cinématique inverse` : KinePy permet de réaliser la cinématique inverse d'un mécanisme. En ajoutant des liaisons pilotés ne faisant pas partie du mécanisme, on peut trouver le pilotage d'actionneurs. 

# Documentation et pré-requis

Nous vous invitons à aller voir le fichier [tutoriel](https://github.com/valentin-burillier/kinepy/blob/main/docs/TUTORIEL.md) montrant dans le détail comment simuler un mécanisme. Ce fichier renvoit sur la documentation des éléments utiliser.

La documentation compléte est accessible [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs).

*N.B. Des connaissances en science de l'ingénieur sont nécessaire pour utiliser KinePy notament dans les domaines de la modélisation, la cinématique et la dynamique. La documentation de KinePy seul ne permet pas de comprendre ces domaines.*

# Exemples

Vous pourrez trouver [ici](https://github.com/valentin-burillier/kinepy/tree/main/examples) des exemples d'application de KinePy sur des systèmes réels :

- Cinématique du bras d'une [pelleteuse](https://github.com/valentin-burillier/kinepy/blob/main/examples/pelleteuse.py) :
<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/86fb79ad-6ebd-40fd-b010-c2c279667e6f">
</p>

- Cinématique inverse d'une [machine à tracer](https://github.com/valentin-burillier/kinepy/blob/main/examples/plotter_machine.py) :
<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/9ba76909-b850-41ba-b97f-ce6decb20e2b">
</p>

- Dynamique d'un [moteur](https://github.com/valentin-burillier/kinepy/blob/main/examples/engine.py) :
<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/0959f164-5b58-4f4d-b8b0-86433319ea70">
</p>

- Quasi-statique d'un [camion benne](https://github.com/valentin-burillier/kinepy/blob/main/examples/Camion%20benne.py) :
<p align="center" width="100%">
    <img width="70%" src="https://github.com/valentin-burillier/kinepy/assets/93446869/4506a1f9-8d50-4b30-a14c-5c1d1b6a2323">
</p>

# Hypothèses

La bibliothèque suit un certains nombre d'hypothèses. Il est nécessaire que le mécanisme que vous voulez simuler les respecte :

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

La bibliothèque est en développement. Il peut donc y avoir des bugs. Merci de les faire remonter aux développeurs afin de les corriger. Actuellement, l'interface graphique permettant de réaliser les schémas cinématiques animés est en train d'être codé. Veuillez signaler si vous rencontrez des problèmes.

De plus, les dévelopeurs seront très attentifs aux demandes d'intégration de nouvelles fonctionnalités.

Nous vous encourageons également à partager les applications que vous faites de KinePy.

# Liens

- https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - Ce qui a inspiré ce répertoire
- http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - Simulateurs cinématiques existants
