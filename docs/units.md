Toute la bibliothèque s'articule autour d'un système d'unité pour l'expression des entrées et des sorties de chaque méthode. La bibliothèque gère automatiquement les convertions.

Au début de son programme, l'utilisateur choisit les unités avec lequel il va travailler. Si les unités déjà implémenté ne lui correspondent pas, il a la possibilité de créer ces propres unités.

Par default, le systéme d'unité est celui du système internationnal a 2 modifiaction près. Les longueurs sont exprimées en mm et la masse volumique en g/cm³.

```
Time                 : s
Length               : mm
Speed                : m/s
Acceleration         : m/s²
Angle                : rad
Angular velocity     : rad/s
Angular acceleration : rad/s²
Density              : g/cm³
Mass                 : kg
Force                : N
Torque               : Nm
SpringConstant       : N/m
Inertia              : kg.m²
dimensionless        : No Unit
```

Les fonction de unit fonctionne avec les arguments suivants :
- `phy` correspond à l'une des grandeur physique ci-dessus. C'est une chaîne de caractère comme par exemple : `'Time'` ou `'Torque'`. Pour éviter les erreurs dans le noms de ces chaînes de caractères et pour coder plus rapidement avec la complétion automatique, le module `unit` possède des variables du noms de la grandeur écrit en majuscule (`TIME`, `TORQUE`) et stockant la chaîne de caractère.
- `value` correspond à la valeur permettant la conversion d'une unité à l'unité du système internationnal. Par exemple, pour des millimètres, `value` vaut 0.001.
- `unit` correspond à l'abreviation du nom de l'unité. C'est une chaîne de caractère. Par exemple, pour des millimètres, `unit` vaut `'mm'`.

Le module `unit` fournit également des variables tel que `MILLINEWTON` où sont renseigné `value` et `unit` permettant le changement d'unité.

- `show_units()` : Affiche le système d'unité actuellement utilisé
- `set_unit_system(unit_system)` : Permet de changer le système d'unité au complet. On fait correspondre l'argument `unit_system` des systèmes d'unité disponibles : `DEFAULT_SYSTEM`, `SI`, `AMERICAN_SYSTEM`, `GOAT_SYSTEM`.
- `set_unit(phy, value, unit='Unnamed unit')` : Permet de changer l'unité d'une seule grandeur
- `get_unit(phy)` : Retourne l'unité de `phy` sous la forme d'une chaîne de caractère

# Le système d'unité chèvre

Prenez la plus belle chèvre que vous possédez. Mesurez sa hauteur, pesez-la, mesurez le temps de bêlement, mesurez la force qu'elle déploie pour vous foncer dessus, faite la tourner sur une broche de bbq pour calculer son moment d'inertie... Grâce à cela, vous aurez toutes les données nécessaire afin de créer un système d'unité. Attention toute fois à ne pas la blesser durant vos mesures. Les chèvres sont des animaux respectables donc il faut les respecter.

Quoi !? Vous ne disposez pas de chèvre et vous n'avez pas envie de faire subir ça à une chèvre. Mais comment allez vous faire pour dimensionner ce qui vous passe par la tête. Heuresement, on a fait tout cela pour vous. Ainsi, avec un tel système d'unité, vous n'aurez aucun mal à dimensionner vos mécanisme.

[finir les mesures + mettre système d'unité chèvre]
