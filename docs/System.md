# Modélisation d'un mécanisme
## Ajout de solide

Le bâti est déjà recensé dans le système.

- `add_solid(j=0., m=0., g=0., name='')` : Ajoute un solide au système et retourne l'objet `Solid` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Solid.md).

## Ajout de liaisons

- `add_revolute(s1, s2, p1=(0., 0.), p2=(0., 0.))` : Ajoute une liaison pivot entre `s1` et `s2` et retourne l'objet `RevoluteJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Revolute.md).
- `add_prismatic(s1, s2, a1=0., d1=0., a2=0., d2=0.)` : Ajoute une liaison glissière entre `s1` et `s2` et retourne l'objet `PrismaticJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Prismatic.md).
- `add_pin_slot(s1, s2, a1=0., d1=0., p2=(0., 0.))` : Ajoute une liaison sphère-plan entre `s1` et `s2` et retourne l'objet `PinSlotJoint` correspondant. Pour connaître le détail des arguments et des fonctionnalités de cette classe, veuillez vous référer [ici](https://github.com/valentin-burillier/kinepy/blob/main/docs/Pin_slot.md).

*On notera que `s1` est toujours le solide de référence pour l'expression des paramètres cinématiques et dynamiques.*

# Entrées du système

- `pilot(joints)` : Permet de spécifier quelles liaisons sont pilotés : c'est-à-dire les liaisons où l'on impose une cinématique particulière. L'argument `joints` peut soit être une liaison (de type `Joint`) ou il peut correspondre à une liste/tuple de liaisons. Lors de la résolution, l'ordre des entrées doit correspondre à l'ordre du pilotage spécifié. Un message dans la console l'indiquera.
- `show_input()` : Montre l'ordre des entrées de pilotage.
- `block(joints)` : 

# Actions mécaniques extérieures

- `add_spring(k, l0, s1, s2, p1=(0, 0), p2=(2, 0))` :
- `add_acceleration_field(g=(0, -9.81))` :

# Compilation

- `compile()` : Lorsque le système est modélisé et que les entrées sont définies, cette méthode va permettre de définir les stratégies de résolution cinématiques, statiques et dynamiques à élaborer. Elle permet également de détecter et de signaler si le mécanisme n'est pas résoluble. Les différents cycles trouvés peuvent dépendre d'un "signe" : c'est-à-dire que le mécanisme peut avoir 2 configurations différentes pour un même paramétrage. C'est à vous de déterminer les bons signes correspondant à votre système. Cela peut être fait en visualisant le mécanisme ou en vérifiant la cohérence de certaines sorties cinématiques. Des signes par défaut sont choisis par la méthode. On les change en renseignant l'attribut `signs` du system. Il est toujours possible de modifier le paramétrage des liaisons après cette étape. 

[mettre un ex]

# Résolution

- `solve_kinematic(inputs)` : Réalise la résolution complète du mécanisme à partir des entrées `inputs` des liaisons pilotées. Si une seule liaison est pilotée, `inputs` correspond à un 1darray de valeur de l'attribut correspondant. Si plusieurs liaisons sont pilotées, `inputs` va correspondre à une liste/tuple/array de valeurs des attributs des liaisons correspondantes, l'ordre des entrées pour la résolution étant indiqué par la méthode `show_input()` ou correspond à l'ordre à laquelle les liaisons ont été déclaré pilotées.
- `solve_statics(t, inputs=None, compute_kine=True)` : 
- `solve_dynamics(t, inputs=None, compute_kine=True)` : 

# Sauvegarde et chargement d'un système

- `save(file)` : 
- `load(file)` : 
