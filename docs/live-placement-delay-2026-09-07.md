# Délai de placement pendant le nettoyage

Le rôle attendu de `frameOffset` est d'aider à orienter la zone nettoyée dans les
premières minutes. La lecture actuelle n'est déclenchée que lors d'une tentative
de placement : auparavant après 40 scans, puis toutes les cinq minutes si le
premier placement était refusé. Une première tentative n'était pas obligatoirement
retardée de cinq minutes ; ce délai concernait les réessais.

Le correctif abaisse le premier seuil à 25 scans et le délai de réessai à 30 secondes
tant que le placement n'est pas accepté. Une fois le placement établi, les mises à
jour de translation gardent la cadence de cinq minutes. Les seuils de confiance,
la carte accumulée et la fusion finale ne sont pas modifiés.

40 tests Python passent, dont trois tests couvrant le premier essai peu après le
démarrage de HA, le réessai après un résultat ambigu et la cadence après placement.
Sur les 25 premiers scans du cycle enregistré précédent, le quart de tour correspond
à celui sauvegardé, avec une marge de 0,263 et un rapport de 3,79 entre orientations.
Cela ne garantit pas qu'un autre cycle ait assez de murs dès le même nombre de scans.
Les tests ajoutés après la passation protègent aussi la conservation de la mesure
initiale malgré la dérive tardive de Raw, sa restauration, le rejet des valeurs
invalides, la libération du placement après annulation et les deux balayages.

Précision après lecture du document de passation `frameOffset-brief.md` : la réduction
modulo 90° est volontaire, pas un bug. L'indice oriente le balayage fin lorsque les
murs sont insuffisants ; les quatre quarts restent comparés. L'angle fin du capteur
n'est pas assez précis pour remplacer la recherche. Le balayage aveugle de secours
et la grille absolue de pas de 0,5° dans la fenêtre orientée sont conservés.

Seule la première mesure initiale valide est utilisée, y compris après restauration
du journal. La lecture HTTP ne relance jamais la sonde série. Les valeurs manquantes,
non finies ou hors plage ne deviennent pas un faux zéro et ne bloquent pas les essais
suivants. Le repère Smooth est traité comme stable : la dérive de Raw n'est pas une
raison de mesurer à nouveau ni de découper la session en morceaux à recaler.

Le cycle actif n'a pas été interrompu : aucun redémarrage ou déploiement pendant
le nettoyage pour ce correctif.
