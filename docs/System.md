# Modélisation d'un mécanisme
## Ajout de solide

Le bâti est le seul solide déjà recensé dans le système.

- `add_solid(j=0., m=0., g=0., name='')` : Ajoute un solide au système et retourne l'objet `Solid` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Solid.md).

## Ajout de liaisons

- `add_revolute(s1, s2, p1=(0., 0.), p2=(0., 0.))` : Ajoute une liaison pivot entre `s1` et `s2` et retourne l'objet `RevoluteJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Revolute.md).
- `add_prismatic(s1, s2, a1=0., d1=0., a2=0., d2=0.)` : Ajoute une liaison glissière entre `s1` et `s2` et retourne l'objet `PrismaticJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Prismatic.md).
- `add_pin_slot(s1, s2, a1=0., d1=0., p2=(0., 0.))` : Ajoute une liaison sphère-plan entre `s1` et `s2` et retourne l'objet `PinSlotJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Pin_slot.md).

*On notera que `s1` est toujours le solide de référence pour l'expression des paramètres cinématiques et dynamiques.*

# Pilotage et blocage du mécanisme

- `pilot(joints)` : Permet de spécifier quelles liaisons sont pilotés : c'est-à-dire les liaisons où l'on impose une cinématique particulière. L'argument `joints` correspondant aux liaisons pilotées peut soit être une liaison (de type `Joint`) ou il peut correspondre à une liste/tuple de liaisons. Lors de la résolution, l'ordre des entrées doit correspondre à l'ordre du pilotage spécifié. Un message dans la console l'indiquera.
- `show_input()` : Montre l'ordre des entrées de pilotage.
- `block(joints)` : Une liaison est dite "bloquée" lorsqu'elle transmet des efforts sur ses degrés de liberté. Par défaut, les liaisons pilotées sont aussi les liaisons bloquées. Or, dans le cadre de la cinématique inverse d'un mécanisme il est utile de pouvoir dissocié les deux. Ainsi, la méthode `block` permet de définir les liaisons transmettant un effort. L'argument `joints` correspondant aux liaisons bloquées peut soit être une liaison (de type `Joint`) ou il peut correspondre à une liste/tuple de liaisons. Les valeurs des efforts transmis sont accessibles par des attributs de la liaison bloquée correspondante (voir la doc de chaque liaison pour plus d'informations).

# Actions mécaniques

- `add_spring(k, l0, s1, s2, p1=(0, 0), p2=(0, 0))` : Ajoute un ressort de raideur `k` et de longueur à vide `l0` fixé au point `p1` du solide `s1` et au point `p2` du solide `s2`. Par défaut, il se lie aux origines de chacun des solides.
- `add_gravity(g=(0, -9.81))` : Ajoute un champ gravitationnel constant s'appliquant à l'ensemble des solides de valeur `g`. Par défaut, `g` correspond au champ gravitationnel terrestre.
D'autres actions mécaniques propres aux solides et aux liaisons peuvent être imposées (voir leur doc respective).

# Compilation

- `compile()` : Lorsque le système est modélisé et que les liaisons sont déclarées pilotées/bloquées, cette méthode va permettre de définir les stratégies de résolution cinématiques, statiques et dynamiques à élaborer pour la résolution. Elle permet également de détecter et de signaler si le mécanisme n'est pas résoluble. Les différents cycles trouvés peuvent dépendre d'un "signe" : c'est-à-dire que le mécanisme peut avoir 2 configurations différentes pour un même paramétrage. C'est à vous de déterminer les bons signes correspondant à votre système. Cela peut être fait en visualisant le mécanisme ou en vérifiant la cohérence de certaines sorties cinématiques. Des signes par défaut sont choisis par la méthode. On les change en renseignant l'attribut `signs` du système. Il est toujours possible de modifier le paramétrage des liaisons et d'ajouter des actions mécaniques après cette étape. 

[mettre un ex]

# Résolution

Dans les trois modes de résolution suivants, `inputs` aux entrées de liaisons pilotées. Si une seule liaison est pilotée, `inputs` prend la forme d'un tableau (1darray) de valeur de l'attribut correspondant : angle pour les pivots,... Si plusieurs liaisons sont pilotées, `inputs` va correspondre à une liste/tuple/array de valeurs des attributs des liaisons correspondantes. L'ordre des entrées pour la résolution étant indiqué par le système, par la méthode `show_input()` et correspond à l'ordre à laquelle les liaisons ont été déclaré pilotées.

## Cinématique

- `solve_kinematics(inputs=None)` : Réalise la résolution cinématique du mécanisme à partir des entrées `inputs` des liaisons pilotées. Cela va permettre de trouver l'angle/l'origine des solides et les valeurs des attributs des liaisons.

## Dynamique

- `solve_dynamics(t, inputs=None, compute_kine=True)` : Réalise la résolution dynamique du mécanisme à partir des entrées `inputs` des liaisons pilotées. Cela correspond à réaliser une simulation cinématique mais aussi de déterminer les efforts transmis par les liaisons lors de la simulation. Les valeurs des efforts transmis sont accessibles par les attributs de la liaison correspondanteb(voir la doc de chaque liaison pour plus d'informations). L'argument `t` correspond à la durée totale de la simulation. L'argument `compute_kine=True` signifie que la résolution cinématique est réalisée par défaut. Mais si le paramétrage géométrique ne change pas entre deux simulations, il est possible de ne la réaliser qu'une fois auparavant grâce à `solve_kinematics` permettant d'accélérer les temps de calculs. Les efforts au début et à la fin de la simulation ne sont pas défini puisque l'accélération ne l'est pas non plus.

## Statique/Quasi-statique

- `solve_statics(inputs=None, compute_kine=True)` : Réalise la résolution statique du mécanisme à partir des entrées `inputs` des liaisons pilotées. Par rapport à la dynamique, les effets d'inertie des solides sont négligés mais le but reste le même : déterminer les efforts transmis par les liaisons. Contrairement à la dynamique, les efforts sont bien définis tout au long de la simulation.
