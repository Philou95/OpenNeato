# Cartographie : corrections intégrées et prochaines améliorations

Branche : `fix/history-lidar-recovery`. Le SLAM existant est conservé :
alignement des scans, fermetures de boucle et optimisation du graphe des poses.
Ces changements concernent l'intégration Home Assistant, sans modification du firmware.

## Corrections intégrées

- Alignement accéléré par masques binaires de lignes, sans dépendance supplémentaire.
  Le cache ne retient qu'un angle et un décalage horizontal ; les grilles trop
  larges ou dispersées utilisent une recherche exacte par ensembles.
- Suppression du double calcul avec `angle_hint` lorsque les murs donnent déjà
  une mesure d'angle. Les deux recherches sont conservées pour les cartes trop fines.
- Recouvrement calculé sur des cellules uniques des deux côtés, borné à 100 %.
- Conservation des contributions qui tombent dans une même cellule après rotation,
  avec sommation stable indépendante de l'ordre des observations.
- Rotation des centres de cellules, cohérente avec les coordonnées continues du
  trajet. Les transformations sauvegardées ne sont ni migrées ni réinterprétées.
- Une orientation ambiguë produit un placement provisoire, sans ajouter ou effacer
  des murs et sans avancer le compteur d'échecs qui pourrait réinitialiser la carte.
- Qualité ICP évaluée à la pose effectivement retournée, après la dernière mise à jour.

La pondération des rayons d'espace libre et la compensation du mouvement restent
des expériences à valider : elles ne sont pas activées par ces corrections.

## Validation

28 tests Python réussis, dont 11 nouveaux tests de géométrie et de SLAM ; syntaxe,
lint de correction Ruff et `git diff --check` réussis. Les tests comparent le score
accéléré à une implémentation indépendante par ensembles, les cellules tournées
au trajet continu, et le résultat ICP à un calcul indépendant des distances.
Ils couvrent aussi les coordonnées négatives, les translations très éloignées,
le repli mémoire, les collisions et les formats d'alignement historiques.

Reconstruction hors ligne du dernier cycle réel à partir de 504 captures, avec
la carte sauvegardée avant le cycle. Trois exécutions par version, ordre alterné,
sur le même PC :

| Étape | Avant | Après |
|---|---:|---:|
| Construction finale des grilles | 3,08 s | 3,06 s |
| Alignement et fusion | 2,38 s | 0,35 s |
| Total avec sérialisation | 5,40 s | 3,44 s |

Soit environ **36 % de temps en moins** pour le calcul final. Les communications
et écritures HA ne sont pas comprises. Ce gain reste à mesurer sur l'appareil HA.
Le pic des allocations Python suivies pendant un alignement est d'environ 6,1 Mio,
ce qui ne représente pas la mémoire totale du processus.

Le dernier cycle conserve 976 fermetures de boucle, la même orientation de fusion
(quart 0, angle fin 0,25°), la même translation (-12, -2 cellules) et un recouvrement
arrondi à 81 %. Les changements de qualité ICP modifient légèrement les grilles :
la carte finale contient 5 403 cellules de murs contre 5 405 auparavant.
Les deux versions complètes ne sont donc pas strictement identiques ; l'accélérateur,
lui, donne exactement les mêmes résultats que le score corrigé calculé sans masques.

En comparant les deux grilles de session dans le repère de fusion enregistré,
leur intersection sur union vaut 99,93 %. Ce chiffre décrit la faible ampleur du
changement, pas une précision physique de 99,93 %. Les anciens alignements sont préservés.

Sept rotations artificielles appliquées à la grille réelle (5°, 13°, 37°, 45°, 61°,
82°, 128°) sont retrouvées à environ 0,1° près, sans atteindre les limites de recherche
et sans perte de poids des murs. Ce sont des tests de robustesse, pas une comparaison
avec les dimensions réelles du logement.

Les mesures détaillées et scripts utilisant les captures privées sont conservés
localement dans `C:\IA\audit-map-fusion-20260906`, hors du dépôt.

## Suite recommandée pour le SLAM existant

1. **Mesurer la convergence et la cohérence des boucles.** Le graphe emploie déjà
   une perte de Huber sur le résidu de translation. Ajouter des diagnostics du coût
   avant/après, des résidus angulaires et de la dispersion des contraintes ; examiner
   ensuite un poids fondé sur l'incertitude et un contrôle des boucles incompatibles.
   Évaluer la rotation dans une échelle cohérente avec la translation, plutôt que
   d'ajouter directement radians et mètres. Un mauvais rapprochement peut sinon
   déformer plusieurs pièces. Aucun nouveau seuil n'est choisi sans données.
2. **Privilégier les accélérations à résultat identique.** L'essai de poses clés a
   été rejeté. Les mesures Smooth/Raw ne justifient pas de déformer ou de découper
   une session : c'est Raw qui dérive. Conserver les scans et le traitement actuel.
3. **Conserver les contributions de sessions entières.** Pour rendre une fusion
   réversible, archiver chaque contribution avec sa transformation rigide enregistrée,
   sans réaligner séparément des morceaux du trajet. Prévoir stockage borné, versions
   et retour arrière avant d'activer cette fonction.
4. **Pondérer l'effacement comme l'observation.** Les impacts sont pondérés par le
   mouvement du robot, mais les traversées d'espace libre comptent chacune pour un.
   Tester une confiance cohérente avec mesures d'épaisseur des murs, trous et vitesse
   de disparition des meubles déplacés. Garder les alignements enregistrés pour comparer.
5. **Améliorer les données temporelles avant de compenser le mouvement.** Conserver
   séquence, temps précis, poses encadrantes et mouvements signés. Vérifier le temps
   réellement associé aux rayons LDS ; la durée du transfert UART n'est pas celle du
   balayage laser. Une correction par rayon ne doit être activée qu'après cette calibration.

La première mesure de profil plaçait l'optimisation du graphe autour de 1,33 s et
le calcul des rayons libres autour de 0,85 s sur le PC. Après accélération de la fusion,
ces étapes deviennent les prochaines cibles à mesurer, sans réduire arbitrairement
les itérations ni supprimer des observations.

## Comparaison avec un autre système

**SLAM Toolbox constitue un bon candidat de comparaison hors ligne**, car il prend
en charge la cartographie 2D, l'optimisation du graphe et la sauvegarde/reprise de ce
graphe. Son fonctionnement et ses entrées laser/odométrie sont décrits dans le
[dépôt officiel](https://github.com/SteveMacenski/slam_toolbox).

Pour OpenNeato, cela nécessiterait un adaptateur vers les messages ROS et les
transformations de repères, ainsi qu'un service ROS 2 séparé sur PC ou Raspberry Pi.
Ce serait un changement d'architecture et de dépendances, pas une bibliothèque à
ajouter au pont ESP32. Je recommande d'abord un export des mêmes captures vers un
banc d'essai, puis une comparaison à transformations enregistrées et mesures physiques
connues. Son adoption ne serait justifiée que par un gain constaté de fidélité ou de
robustesse qui compense cette complexité. Aucun remplacement du SLAM n'est effectué ici.
