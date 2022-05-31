# Glissiere

# Paramétrage d'une glissière

![image](https://user-images.githubusercontent.com/93446869/171271772-efb7b502-cd72-466a-a873-c091fbf4da47.png)

```python
>>> # creation d'une glissiere d'axe suivant l'axe des abscisse du repere associé à 1 et distant de 1.5 de son origine et d'axe penché de 45° par rapport à l'axe des abscisses du repere associe a 2 et distant de -3 de son origine
>>> G = m.Glissiere(sol1=1, sol2=2, dist1=1.5, dist2=-3, alpha1=0, alpha2=np.pi/4)
>>> G
'Glissiere 2/1'
>>> G.dist1, G.alpha2 = 0.4, 0.0 # modification du paramétrage
>>> G.dist2
-3
>>> 
>>> l0, k = 0.2, 0.07
>>> F = lambda : -(G.value - l0)*k # le couple peut dependre de parametres géométrique pas encore simuler
>>> G.applie_force(F) # applique une force selon l'axe de glissement de sol2 sur sol1
```

# apres une simulation

```python
>>> # apres une simulation
>>> G.value # distance entre l'origine de sol2 par rapport a l'origine de sol1
array([
>>> G.force # force selon la normale de l'axe de glissement
array([
```
