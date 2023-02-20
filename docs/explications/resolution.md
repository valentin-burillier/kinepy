# Outils de résolutions

## Notations
Le problème est dans le plan $(O,\vec{x},\vec{y})$. Cependant on se permettra quelques facilités de l'espace à 3 dimensions, notamment le produit vectoriel avec le vecteur $\vec{z}$ pour caractériser les rotations et le déterminant

Matrice de rotation d'un angle $\theta$

$$ R(\theta) = 
\begin{pmatrix}
    \cos\theta & -\sin\theta\ \\
    \sin\theta & \cos \theta\
\end{pmatrix} 
$$

En particulier pour tout vecteur $\vec{v}$ :

$$
R(\frac{\pi}{2}) \cdot \vec{v} = \vec{z} \wedge \vec{v} = 
\begin{pmatrix}
    - v_y \\
    v_x
\end{pmatrix}
$$

On utlisera que

$$
R(\theta)\cdot\vec{v} = \cos\theta\cdot\vec{v} + \sin\theta\cdot\vec{z}\wedge\vec{v}
$$


L'application signe $\text{sign}$

$$ \text{sign}(x) = 
\begin{cases}
    \ 1& \text{ si } x \geq 0 \\
    -1& \text{ sinon}
\end{cases}
$$

Le déterminant $\det$

$$
\det(\vec{v}\,\vec{w}) = (\vec{v}\wedge\vec{w})\cdot\vec{z} = (\vec{z} \wedge \vec{v})\cdot \vec{w} = v_x\cdot w_y - v_y\cdot w_x
$$


## Structures des solides
Un solide est représenté par:
 - un vecteur $\vec{o}$ répresentant la position de son origine dans le référentiel de sa classe
 - un angle $a$ représentant l'orientation du solide dans le référentiel de sa classe

Déplacer l'origine d'une classe S0 sur un point $\vec{p}$ exprimé dans cette même classe revient à $\forall S \in$ S0

$$
\vec{o}_S \gets \vec{o}_S - \vec{p}
$$

Tourner une classe S0 d'un angle $\theta$ revient à $\forall S \in$ S0

$$
\begin{align}
\vec{o}_S &\gets R(\theta)\cdot \vec{o}_S \\
a_S &\gets a_S + \theta
\end{align}
$$

Placer l'origine d'une classe S0 sur un point $\vec{p}$ exprimé dans une classe S1 revient à $\forall S \in$ S0

$$
\vec{o}_S \gets \vec{0}_S + \vec{p}
$$

## Cinématique

### RR
En lui-même ce groupe est hyperstatique, cependant en tant qu'état intermédiare, il permet de séquencer la résolution cinématique  


<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/213764345-5df7d27e-c186-4968-9b56-f044e0624727.svg">
</p>
<p align="center">Fig 1 - 2 classes (S1 et S2) liées par 2 pivots</p>

On doit vérifier:

$$ \lVert\vec{V_1}\lVert\  = \lVert\vec{V_2}\lVert\ = A $$

Ensuite, on obtient par le produit scalaire

```math
\begin{align}
    \vec{V_1} \cdot \vec{V_2}\  &= A^2 \cdot \cos \alpha \\
    \Leftrightarrow \lvert \alpha \lvert &= \text{Arccos} \frac{\vec{V_1} \cdot \vec{V_2}}{A^2}
\end{align}
```
De plus on a

$$ \text{sign}\ \alpha = \text{sign} \ \text{det}(\vec{V_2} \, \vec{V_1}) $$

On peut donc unir les 2 classes de la manière suivante:
 - Déplacement l'origine de S2 sur $R_{0,2}$
 - Rotation de la classe S2 de l'angle $\alpha$ 
 - Placement de l'origine de S2 sur $R_{0,1}$


### PP

De même que RR, ce groupe est hyperstatique et permet de séquencer la résolution

### P-Chain

Une glissière détermine l'orientation relative de solides, on en déduit donc que les parties connexes en P ont leurs orientations relatives résolues:



## Dynamique

### Group tmd/trd

# RRR
<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833440-20233a5a-d9b6-4e36-816e-dfa08279a892.svg"></p>
<p align="center">Fig 1 - Schéma cinématique et graphe du groupe Assur RRR</p> 

## Cinématique

Al-Kashi\
PP

## Dynamique

# RRP
<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833434-f354ef88-d5ca-4c4e-a12e-0da1a91e91ae.svg">
</p>
<p align="center">Fig 2 - Schéma cinématique et graphe du groupe Assur RRP</p>

## Cinématique

Calcul précis\
RR

## Dynamique

# PPR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833426-806d4ee1-256f-4b03-8def-7981018277df.svg">
</p>
<p align="center">Fig 3 - Schéma cinématique et graphe du groupe Assur PPR</p>

## Cinématique
Après l'application de l'opération P-chain, S2 et S3 ont leur orientation relative résolue.
Donc R est résolu, on peut unir S2 et S3:
 - On déplace l'origine de S2 sur R dans S2
 - On place l'origine de S2 sur R dans S3

Pour ajouter S1, on résout PP entre S1 et S2 $\cup$ S3

## Dynamique

# 3-RR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833424-f443d051-61bf-4aba-9e33-a4a340e1fa27.svg">
</p>
<p align="center">Fig 4 - Schéma cinématique et graphe du groupe Assur 3-RR</p>

## Cinématique

Non Résolu

## Dynamique

# 2-RR-PP

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833448-fa52e8f7-c5cb-495f-bb86-4392d5af8cdb.svg">
</p>
<p align="center">Fig 5 - Schéma cinématique et graphe du groupe Assur 2-RR-PP</p>

## Cinématique

P-Chain\
Al_Kashi\
RR + PP

## Dynamique

# 3-PR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833450-2be1f2df-3037-46e9-a1c0-d8fcfd465413.svg">
</p>
<p align="center">Fig 6 - Schéma cinématique et graphe du groupe Assur 3-PR</p> 

## Cinématique

P-Chain\
Calcul précis\
R résolus\
PP\
P résolu

## Dynamique

# 2-PR-RR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833446-c6d99b87-787d-4313-97b2-bbd40be7359b.svg">
</p>
<p align="center">Fig 7 - Schéma cinématique et graphe du groupe Assur 2-PR-RR</p>

## Cinématique

Non Résolu

## Dynamique

# PP-PR-RR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833431-9784618b-0ffa-4aed-9f0e-5f9447fb4cbd.svg">
</p>
<p align="center">Fig 8 - Schéma cinématique et graphe du groupe Assur PP-PR-RR</p> 

## Cinématique

P-Chain\
R résolu\
RRP\
PP

## Dynamique

# 2-RR-PR

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833449-1d2fb8a0-d805-4ecf-bba7-84e3311b281a.svg">
</p>
<p align="center">Fig 9 - Schéma cinématique et graphe du groupe Assur 2-RR-PR</p> 


## Cinématique

Non Résolu

## Dynamique

# PP-PR-RP

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833429-bab1fbda-894a-4dfd-90a9-6084b9d8056a.svg">
</p>
<p align="center">Fig 10 - Schéma cinématique et graphe du groupe Assur PP-PR-RP</p> 


## Cinématique

P-chain\
R résolus

## Dynamique

# 2-PR-PP

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833444-c9628f7d-71c6-442d-8824-2b1d80f321aa.svg">
</p>
<p align="center">Fig 11 - Schéma cinématique et graphe du groupe Assur 2-PR-PP</p>

## Cinématique

P-chain\
R-résolus

## Dynamique

# PP-PR-RP

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833436-cb702767-aa21-4c11-89fc-4f184b607a01.svg">
</p>
<p align="center">Fig 12 - Schéma cinématique et graphe du groupe Assur RR-PR-RP</p>

## Cinématique
Non résolu

## Dynamique

# 2-PR-RP

<p align="center">
    <img width="80%" src="https://user-images.githubusercontent.com/89185062/212833442-844cb4f8-8128-46a3-a727-99088a9994b9.svg">
</p>
<p align="center">Fig 13 - Schéma cinématique et graphe du groupe Assur 2-PR-RP</p> 

## Cinématique

trick chelou

## Dynamique
