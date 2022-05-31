# Pivot

# Paramétrage d'une pivot

![image](https://user-images.githubusercontent.com/93446869/171271518-9edfab6c-9135-4c7b-82ce-f299bb226b7d.png)


```python
>>> # creation d'une pivot au point (1, 1) dans le solide d'index 0 et au point (0, 0.5) dans le solide d'index 1
>>> P = m.Pivot(sol1=0, p1=(1, 1), sol2=1, p2=(0, 0.5))
>>> P
'Pivot 1/0'
>>> P.p1 = (1, 0.5) # modification du point 1
>>> P.p2
(0, 0.5)
>>> 
>>> C = lambda : -P.angle*0.1 # le couple peut dependre de parametres géométrique pas encore simuler
>>> P.applie_couple(C) # applique un couple torsion de sol2 sur sol1 au niveau de la liaison
```


# apres une simulation

```python
>>> # apres une simulation
>>> P.angle # angle oriente de sol2 par rapport à sol1
array([
>>> P.point # point de pivot exprimer dans le referentiel global 
array([
>>> P.force # force transmis par la pivot exprimer dans le referentiel global
array([
```
