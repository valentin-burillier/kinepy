# Pivot

## Parameters

![image](https://user-images.githubusercontent.com/93446869/171271518-9edfab6c-9135-4c7b-82ce-f299bb226b7d.png)

## Initialisation

```pycon
>>> P = S.add_revolute(sol1=0, p1=(1, 1), sol2=1, p2=(0, 0.5))
>>> P
'Rev(1/0)'


# ça, c'est la sauce pour le moment
>>> P.p1 = (1, 0.5) # modification du point 1
>>> P.p2
(0, 0.5)
>>> 
>>> C = lambda : -P.angle*0.1 # le couple peut dependre de parametres géométrique pas encore simuler
>>> P.applie_couple(C) # applique un couple torsion de sol2 sur sol1 au niveau de la liaison
```


## apres une simulation

```python
>>> # apres une simulation
>>> P.angle # angle oriente de sol2 par rapport à sol1
array([
>>> P.point # point de pivot exprimer dans le referentiel global 
array([
>>> P.force # force transmis par la pivot exprimer dans le referentiel global
array([
```
