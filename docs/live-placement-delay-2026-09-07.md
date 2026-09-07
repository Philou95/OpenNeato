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

34 tests Python passent, dont trois tests couvrant le premier essai peu après le
démarrage de HA, le réessai après un résultat ambigu et la cadence après placement.
Sur les 25 premiers scans du cycle enregistré précédent, le quart de tour correspond
à celui sauvegardé, avec une marge de 0,263 et un rapport de 3,79 entre orientations.
Cela ne garantit pas qu'un autre cycle ait assez de murs dès le même nombre de scans.

Limite distincte : `_frame_angle_hint()` réduit encore l'angle modulo 90°. Le quart
de tour dépend de la reconnaissance des murs et de sa confiance. Le présent correctif
retire l'attente excessive mais ne constitue pas encore un placement d'affichage
fondé directement sur l'orientation complète de `frameOffset`. Cette seconde étape
doit tenir compte du repère de la carte accumulée et rester séparée de la fusion.

Le cycle actif n'a pas été interrompu : aucun redémarrage ou déploiement pendant
le nettoyage pour ce correctif.
