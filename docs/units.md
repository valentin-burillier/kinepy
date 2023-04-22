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

- `show_units()` : Affiche le système d'unité actuellement utilisé
- `set_unit_system(unit_system)` : Permet de changer le système d'unité au complet. On fait correspondre l'argument `unit_system` des systèmes d'unité disponibles : `DEFAULT_SYSTEM`, `SI`, `AMERICAN_SYSTEM`, `GOAT_SYSTEM`.
- `set_unit(phy, value, unit='Unnamed unit')` : Permet de changer l'unité d'une seule grandeur parmi celle ci-dessus. [Bla³]
- `get_unit(phy)` : Retourne l'unité de `phy` sous la forme d'une chaîne de caractère.

# Le système d'unité chèvre

Prenez la plus belle chèvre que vous possédez. Mesurez sa hauteur, pesez-la, mesurez le temps qu'elle met pour manger une carotte, mesurez la force qu'elle déploie pour vous foncer dessus, faite la tourner pour calculer son moment d'inertie... Grâce à cela, vous aurez toutes les données nécessaire afin de créer un système d'unité. Attention toute fois à ne pas la blaisser durant vos mesures. Les chèvres sont des animaux respectables donc il faut les respecter.
