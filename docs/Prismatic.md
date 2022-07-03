# Prismatic joint

## Parameters

![image](https://user-images.githubusercontent.com/93446869/171271772-efb7b502-cd72-466a-a873-c091fbf4da47.png)

## Initialisation

```pycon
>>> G = S.add_prismatic(sol1=1, sol2=2, alpha1=0., d1=1., alpha2=np.pi/4, d2=-2.)
>>> G
'Pri(2/1)'
>>> G.d1, G.alpha2 = 0.4, 0.0 # updating parameters
>>> G.d2
-2.

>>> l0, k = 0.2, 0.07
>>> F = lambda : -(G.value - l0)*k # le couple peut dependre de parametres géométrique pas encore simuler
>>> G.apply_force(F) # applique une force selon l'axe de glissement de sol2 sur sol1
```

## apres une simulation

```pycon
>>> # apres une simulation
>>> G.value # distance entre l'origine de sol2 par rapport a l'origine de sol1
array([
>>> G.force # force selon la normale de l'axe de glissement
array([
```
