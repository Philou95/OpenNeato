# SLAM : optimisation exacte et essais de poses clés

Ces travaux prolongent les corrections de fusion du commit `c3803fa`, déployées
sur Home Assistant le 7 septembre. Le SLAM existant est conservé.

## Améliorations intégrées

Le solveur du graphe recalculait les rotations des mesures constantes pour chaque
contrainte et chaque itération. Il calculait aussi les deux blocs de dérivées
alors qu'un seul était utilisé pour mettre à jour un nœud.

Les rotations des mesures sont maintenant calculées une fois ; celles des poses
sont actualisées immédiatement après chaque mise à jour. Ce dernier point conserve
l'ordre Gauss-Seidel : un voisin déjà modifié dans l'itération doit être lu dans
son nouvel état. Seul le bloc de dérivées utile est construit.

Sur le graphe de 504 poses et 1 479 contraintes extrait du dernier cycle :

| Solveur | Temps médian, 3 essais |
|---|---:|
| Avant | 1,280 s |
| Après, diagnostics compris | 1,158 s |

**9,6 % de temps en moins sur le solveur**, avec égalité exacte de toutes les poses
calculées sur le même PC. Ce gain concerne le graphe, pas l'ensemble du cycle ;
il ne s'additionne pas directement aux 36 % mesurés précédemment pour la finalisation.

Un nouvel essai complet, trois exécutions alternées par version, compare l'ensemble
des modifications à `b654744` : médiane de 5,35 s avant contre 3,19 s après, soit
environ 40 % de temps en moins sur le PC. Les communications et écritures HA sont
exclues ; le gain sur Home Assistant reste à mesurer lors d'un prochain cycle.

Les diagnostics indiquent désormais le nombre d'itérations, la convergence selon
le critère existant, le dernier déplacement et les résidus avant/après. Les mètres
et degrés restent séparés : aucune nouvelle pondération n'est appliquée.

Sur ce graphe, le solveur atteint les 400 itérations autorisées sans satisfaire
son critère d'arrêt très strict. Le dernier déplacement maximal n'est toutefois
que de 0,078 mm, et la dernière rotation de 0,00081°. Le percentile 95 des résidus
de translation passe de 8,54 à 3,23 cm ; celui des résidus angulaires, de 3,91 à 1,39°.
Ce sont des mesures de cohérence interne du graphe, pas des erreurs mesurées sur
les murs du logement. Le critère d'arrêt et le nombre d'itérations ne sont pas modifiés.

31 tests Python passent. Les nouveaux tests vérifient les dérivées par différences
finies, la fraîcheur du cache à chaque mise à jour, les unités des diagnostics et
l'absence d'effet des diagnostics sur les poses. Le benchmark réel vérifie aussi
l'égalité exacte avec le solveur du commit `c3803fa`.

## Réduction des poses : prototype non retenu

Un prototype hors ligne conserve une pose sur 2, 4 ou 8, transporte les contraintes
vers ces poses et interpole les corrections pour les poses intermédiaires. Il
approxime la marginalisation des contraintes ; il n'en conserve pas les corrélations.
Les résultats sont comparés au graphe complet dans son repère, sans ajuster à
nouveau l'alignement de la carte pour masquer les écarts.

| Poses conservées | Temps solveur + interpolation | Écart médian | Écart maximal |
|---:|---:|---:|---:|
| 253 / 504 | 0,381 s | 3,24 cm | 8,85 cm |
| 127 / 504 | 0,295 s | 16,61 cm | 35,41 cm |
| 64 / 504 | 0,131 s | 12,64 cm | 46,47 cm |

L'essai le plus prudent présente aussi un écart angulaire maximal de 2,70°.
Tous dépassent les critères exploratoires de 2,5 cm et 0,2° maximum par rapport au
graphe complet. **Cette réduction n'est ni intégrée à l'algorithme ni déployée.**
Elle ne démontre pas que toute méthode de poses clés est mauvaise : cette méthode
de regroupement et d'interpolation est insuffisante sur ces données.

## Travail suivant

1. Exploiter les diagnostics sur plusieurs vrais cycles : répétition des grandes
   erreurs angulaires, contraintes incompatibles, impact des virages et passages étroits.
2. Pour réduire le graphe, préserver les contraintes et leurs corrélations par une
   marginalisation adaptée, et sélectionner les poses selon la géométrie plutôt
   qu'un simple intervalle. Garder le graphe complet comme référence de validation.
3. Évaluer des sous-cartes conservées entre cycles pour rendre les contributions
   réversibles. Dimensionner le stockage avant toute activation.
4. Tester la pondération des traversées d'espace libre et l'horodatage précis des
   rayons séparément, avec des mesures de murs et les alignements enregistrés.

Les scripts et résultats privés sont dans `C:\IA\audit-map-fusion-20260906` :
`slam_improvements.py`, `slam-improvements.json`. Aucun scan privé n'est ajouté au dépôt.
