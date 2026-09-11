# simulflow — simulateur d'écoulement 2D (Lattice-Boltzmann D2Q9)

Simulateur interactif de mécanique des fluides en 2D, temps réel, basé sur
la méthode de Boltzmann sur réseau (**Lattice Boltzmann Method**, LBM,
réseau **D2Q9**). On dessine des obstacles à la souris dans un écoulement
uniforme et on observe en direct le champ de vitesse, la vorticité, la
pression, ainsi que les efforts aérodynamiques (traînée/portance) exercés
sur l'obstacle.

Rendu et fenêtrage via [raylib](https://www.raylib.com/) ; C++23 ; build
CMake avec presets Clang/libc++.

---

## Sommaire

- [Utilisation](#utilisation)
  - [Prérequis](#prérequis)
  - [Compilation](#compilation)
  - [Lancement](#lancement)
  - [Contrôles](#contrôles)
  - [Le HUD](#le-hud)
  - [Champs affichés](#champs-affichés)
  - [Obstacles](#obstacles)
  - [Bords du domaine](#bords-du-domaine)
- [L'algorithme : Lattice-Boltzmann D2Q9](#lalgorithme--lattice-boltzmann-d2q9)
  - [Principe général](#principe-général)
  - [Le réseau D2Q9](#le-réseau-d2q9)
  - [Distribution d'équilibre](#distribution-déquilibre)
  - [Collision (BGK)](#collision-bgk)
  - [Advection (streaming)](#advection-streaming)
  - [Grandeurs macroscopiques](#grandeurs-macroscopiques)
  - [Conditions aux limites](#conditions-aux-limites)
  - [Unités réseau, viscosité, Reynolds](#unités-réseau-viscosité-reynolds)
  - [Efforts aérodynamiques](#efforts-aérodynamiques)
  - [Stabilité numérique](#stabilité-numérique)
- [Structure du projet](#structure-du-projet)
- [Paramètres ajustables](#paramètres-ajustables)

---

## Utilisation

### Prérequis

Le projet récupère et compile raylib depuis les sources via CMake
`FetchContent` (reproductible, sans `sudo`) — mais compiler raylib
lui-même sous Linux nécessite les headers de développement des backends
X11/OpenGL :

```bash
sudo apt install -y libgl1-mesa-dev libx11-dev libxrandr-dev \
    libxinerama-dev libxcursor-dev libxi-dev libxkbcommon-dev \
    wayland-protocols
```

Compilateur : Clang (le preset force `-stdlib=libc++`, nécessaire pour
`<print>` et les autres en-têtes C++23 modernes). CMake ≥ 3.20.

### Compilation

```bash
cmake --preset clang        # configure (1ère fois, ou après un changement de CMakeLists)
cmake --build --preset clang
```

Le binaire est produit dans `bin/`, nommé
`simulflow_v<version>_<hash-git>` (métadonnées injectées automatiquement à
la configuration — voir `include/version.hpp.in`).

Le type de build par défaut est **Release** (`-O3 -march=native`) : une
simulation LBM sans optimisation est de l'ordre de 10× plus lente. Pour
déboguer :

```bash
cmake --preset clang -DCMAKE_BUILD_TYPE=Debug
```

### Lancement

```bash
./bin/simulflow_v1.0.1_<hash>
```

ouvre une fenêtre redimensionnable (la grille de simulation suit la
taille de la fenêtre, à raison de 2 px/cellule).

Il existe aussi un mode **benchmark headless** (pas de fenêtre), utile
pour mesurer le débit du solveur et vérifier qu'il reste stable :

```bash
./bin/simulflow_v1.0.1_<hash> --bench [nb_iterations]   # défaut : 2000
```

Affiche le débit en MLUPS (millions de mises à jour de cellule par
seconde), la vitesse max atteinte (doit rester finie et de l'ordre de
`u_in`), et les coefficients `Cx`/`Cz` de l'obstacle par défaut.

### Contrôles

| Entrée | Effet |
|---|---|
| Clic gauche (maintenu) | Peindre un obstacle sous le curseur |
| Clic droit (maintenu) | Effacer un obstacle sous le curseur |
| Molette | Changer le rayon du pinceau |
| `Espace` | Pause / reprise |
| `R` | Réinitialiser l'écoulement (obstacles conservés) |
| `+` / `-` (ou pavé numérique) | Faire pivoter les obstacles (1°/appui, répétition = rotation continue) |
| Flèches directionnelles | Translater les obstacles (rognés s'ils sortent de la grille) |
| `V` | Cycler le champ affiché : Vitesse → Vorticité → Pression |
| `/` (ou `.` selon la disposition clavier) | Basculer les bords haut/bas : paroi fixe ↔ frontière libre |
| `Z` (ou `W` en AZERTY) | Afficher/masquer le graphe `Cz(t)` |
| `H` | Afficher/masquer tout le HUD |

> Note disposition clavier : raylib rapporte les touches par **position
> physique** (norme US), pas par caractère produit. Sur un clavier
> français, "Z" physique tape "W", et "/" (Shift + ".") est rapporté
> comme la touche "." — d'où les doublons `Z`/`W` et `/`/`.` ci-dessus.

### Le HUD

En haut de la fenêtre (bascule avec `H`) :

- ligne d'état : nom du moteur, champ affiché, angle des obstacles, état
  des bords (`parois`/`libres`), `[PAUSE]` le cas échéant ;
- `Cx`, `Cz`, `finesse` (`Cz/Cx`) : coefficients aérodynamiques instantanés
  (lissés, voir plus bas) ;
- ligne de rappel des raccourcis clavier ;
- une flèche jaune part du centre de l'obstacle et matérialise l'effort
  résultant (direction et intensité, longueur saturée au-delà d'un
  certain seuil pour rester dans le cadre) ;
- un graphe `Cz(t)` (bascule `Z`/`W`) trace l'historique de la portance —
  utile pour repérer le détachement tourbillonnaire (allée de von Kármán),
  qui se traduit par une oscillation périodique de `Cz`.

### Champs affichés

Touche `V`, trois grandeurs, chacune avec sa propre palette :

| Champ | Grandeur | Palette |
|---|---|---|
| **Vitesse** | norme \|u\| | séquentielle sombre → violet → rose → orange → **blanc/jaune pâle** pour les vitesses les plus hautes ; les basses vitesses sont bleu-nuit très sombre (quasi noir), pas franchement bleu |
| **Vorticité** | rotationnel `∂u_y/∂x − ∂u_x/∂y` | divergente : **bleu** = négatif (rotation horaire), **rouge** = positif (antihoraire), gris sombre neutre à 0 — c'est le champ qui révèle le mieux les tourbillons |
| **Pression** | coefficient `Cp = (p − p_inf) / (½ ρ U²)` | même palette divergente : bleu = dépression, rouge = surpression |

Les obstacles sont toujours rendus en gris (`{60,62,74}`), quel que soit
le champ affiché.

### Obstacles

Un profil d'aile **NACA 4 chiffres** cambré (≈ NACA 2412, portance non
nulle dès incidence nulle) est placé par défaut dans le premier tiers du
domaine. On peut :

- ajouter/effacer de la matière au pinceau (disque) ;
- faire pivoter **tout** le masque d'obstacles (`+`/`-`) ;
- le translater (flèches).

En interne, tout est stocké dans un masque de **référence** à l'angle 0 ;
le masque réellement simulé en est une copie tournée reconstruite à la
demande (rotation inverse par *gather*, sans trou). Rotation et
translation sont donc solidaires : dessiner, pivoter puis translater
donne le même résultat quel que soit l'ordre.

### Bords du domaine

- **Gauche (entrée)** : vitesse imposée `u_in` (condition de type
  Dirichlet, via la distribution d'équilibre).
- **Droite (sortie)** : gradient nul — recopie de l'avant-dernière colonne
  (condition de Neumann homogène, laisse sortir l'écoulement sans
  réflexion).
- **Haut / bas** : togglables avec `/` (voir [Contrôles](#contrôles)) :
  - **paroi fixe** (par défaut) : rebond complet, non-glissement — une
    onde de pression qui atteint le bord se réfléchit. Une **ligne de
    pixels jaunes** est dessinée en haut et en bas pour matérialiser
    cette paroi ;
  - **frontière libre** : même traitement gradient-nul que la sortie —
    les ondes/tourbillons qui atteignent le bord sortent du domaine au
    lieu de rebondir. La ligne jaune disparaît.

---

## L'algorithme : Lattice-Boltzmann D2Q9

### Principe général

Au lieu de discrétiser directement les équations de Navier-Stokes sur les
champs macroscopiques `(ρ, u, p)`, la LBM fait évoluer, en chaque cellule
de la grille, un petit jeu de **populations** `f_i(x, t)` — la fraction
(fictive) de "particules" se déplaçant à la vitesse discrète `e_i`. À
chaque pas de temps, deux étapes purement **locales et explicites**
s'enchaînent :

1. **Collision** : relaxation de chaque population vers son équilibre
   local (BGK) ;
2. **Advection (streaming)** : chaque population se décale d'une cellule
   dans la direction `e_i`.

Les grandeurs macroscopiques (densité, vitesse, pression) sont de simples
**moments** de ces populations, recalculés à la volée. C'est un schéma
très bien adapté au calcul parallèle (rien de global, tout est local à la
cellule et à ses 8 voisines), au prix d'une compressibilité artificielle
(la densité varie légèrement, alors que l'écoulement visé est
quasi-incompressible) — d'où la contrainte de rester à faible nombre de
Mach (voir [Stabilité numérique](#stabilité-numérique)).

### Le réseau D2Q9

"D2Q9" = 2 Dimensions, 9 vitesses discrètes `e_i` :

```
    6   2   5
      \ | /
    3 - 0 - 1
      / | \
    7   4   8
```

- `e_0 = (0,0)` (repos), poids `w_0 = 4/9` ;
- `e_1..e_4` : axes, poids `1/9` ;
- `e_5..e_8` : diagonales, poids `1/36`.

(`include/core/LbmEngine.hpp`, `src/core/LbmEngine.cpp` — tables `kCx`,
`kCy`, `kW`.) Ces poids garantissent l'isotropie du réseau jusqu'à l'ordre
nécessaire pour retrouver Navier-Stokes en limite continue.

### Distribution d'équilibre

Développement de Maxwell-Boltzmann tronqué à l'ordre 2 en vitesse :

```
f_i^eq(ρ, u) = w_i · ρ · [ 1 + 3(e_i·u) + (9/2)(e_i·u)² − (3/2)|u|² ]
```

(`feq_dir()`, `LbmEngine.cpp`). C'est la cible vers laquelle chaque
population relaxe à la collision, et c'est aussi ce qui sert à *imposer*
une condition aux limites (vitesse d'entrée, cf. plus bas) : on remplace
directement les `f_i` par leur valeur d'équilibre pour la `(ρ, u)` visée.

### Collision (BGK)

Approximation **BGK** (Bhatnagar-Gross-Krook), une seule constante de
temps de relaxation `τ` :

```
f_i* = f_i − ω (f_i − f_i^eq(ρ, u)),      ω = 1/τ
```

`ρ` et `u` sont les moments locaux courants (calculés juste avant, dans
la même passe). C'est l'étape qui injecte la viscosité dans le système —
voir [Unités réseau](#unités-réseau-viscosité-reynolds).

### Advection (streaming)

Chaque population post-collision se propage d'une cellule dans sa
direction :

```
f_i(x + e_i, t+1) = f_i*(x, t)
```

Le code fusionne collision + streaming en une seule passe
(`collide_and_stream()`), en écrivant directement dans un buffer "cible"
(`m_f_tmp`, schéma *ping-pong*, échangé avec `m_f` à la fin du pas via
`swap`) — pas de recopie intermédiaire.

### Grandeurs macroscopiques

Simples moments des populations, recalculés partout après chaque pas
(`compute_macros()`, utilisé uniquement pour le rendu) :

```
ρ(x)   = Σ_i f_i(x)
ρ u(x) = Σ_i f_i(x) · e_i
```

La pression suit de l'équation d'état du gaz sur réseau :
`p = c_s² ρ`, avec `c_s² = 1/3` (vitesse du son du réseau).

### Conditions aux limites

| Bord | Traitement | Type physique |
|---|---|---|
| Gauche (entrée) | `f_i` remplacé par `f_i^eq(1, u_in, 0)` | Dirichlet (vitesse imposée) |
| Droite (sortie) | `f_i` recopié depuis la colonne voisine | Neumann homogène (gradient nul) |
| Haut/bas, mode paroi | rebond complet sur tout lien qui sortirait du domaine | Dirichlet (non-glissement, `u=0`) |
| Haut/bas, mode libre | `f_i` recopié depuis la ligne voisine, pas de rebond | Neumann homogène (gradient nul) |
| Obstacles | rebond complet ("halfway bounce-back") | Dirichlet (non-glissement) |

Le **rebond complet** (*bounce-back*) renvoie une population dans la
direction opposée : `f_opp(i)(x, t+1) = f_i*(x, t)` — implémenté en
écrivant `fpost` à l'index `kOpp[i]` (`kOpp` = permutation involutive des
9 directions) au lieu de l'index `i`. C'est la condition de non-glissement
la plus simple à mettre en œuvre en LBM, appliquée uniformément aux
parois fixes du domaine *et* aux obstacles peints par l'utilisateur (même
branche de code, `collide_and_stream()`).

Détail d'implémentation notable : la sortie (droite) et — quand elle est
activée — la frontière libre (haut/bas) sont réécrites en gradient nul
**avant** l'étape collision+streaming de chaque sous-pas
(`apply_boundaries()`, appelé en tête de boucle dans `step()`). Le rebond
générique appliqué ensuite dans `collide_and_stream()` à *tout* lien
sortant du domaine — y compris sur ces bords — est donc sans effet sur la
gauche/droite (immédiatement écrasé au sous-pas suivant), mais bien réel
sur le haut/bas quand aucune frontière libre n'a été demandée : c'est
littéralement la seule chose qui empêche l'écoulement de "fuir" par le
haut ou le bas.

### Unités réseau, viscosité, Reynolds

Tout est exprimé en **unités réseau** (`Δx = Δt = 1`, `ρ_inf = 1`). La
viscosité cinématique se déduit du temps de relaxation :

```
ν = c_s² (τ − 1/2) = (τ − 1/2) / 3
```

(`set_relaxation_time()`). Le nombre de Reynolds, pour une échelle
caractéristique `L` (typiquement la corde de l'obstacle) :

```
Re = u_in · L / ν
```

(`reynolds()`), affiché au démarrage dans la console.

### Efforts aérodynamiques

Calculés par la méthode d'**échange de quantité de mouvement**
(*momentum-exchange method*) : à chaque rebond sur un obstacle, la
population arrive avec une quantité de mouvement `e_i · f*`, repart avec
`−e_i · f*` ; l'obstacle encaisse donc `2 e_i · f*` — accumulé sur tous
les liens de rebond, tous les sous-pas, moyenné puis lissé par une
moyenne mobile exponentielle (EMA, facteur `0.95`) car le signal instantané
oscille au rythme du détachement tourbillonnaire :

```
Cx = F_x / (½ ρ_inf U_inf² c)      (traînée)
Cz = −F_y / (½ ρ_inf U_inf² c)     (portance, "vers le haut" écran)
```

`c` = corde de **référence** de l'obstacle (étendue en x du masque non
tourné), pour que le coefficient ne varie pas artificiellement avec
l'angle d'incidence choisi.

### Stabilité numérique

- `τ > 1/2` est une condition **impérative** (sinon `ν < 0`) — imposé par
  `set_relaxation_time()` (`τ ≥ 0.5001`).
- Nombre de Mach réseau `Ma = |u| / c_s ≲ 0.3` : condition empirique pour
  que le développement tronqué de `f_i^eq` reste une bonne approximation
  et que le schéma ne diverge pas.
- Des garde-fous anti-NaN sont présents à plusieurs endroits : `reset()`
  remet à zéro les accumulateurs EMA des efforts (sinon un seul NaN issu
  d'une divergence passée les contamine indéfiniment,
  `kEma·NaN + (1−kEma)·x = NaN`) ; les fonctions de colormap
  (`colormap_speed`, `colormap_diverging`) et le tracé `Cz(t)`
  détectent les valeurs non finies pour saturer proprement l'affichage
  plutôt que de planter sur un cast UB ou un tracé corrompu.

---

## Structure du projet

```
include/core/ISimulationEngine.hpp  interface abstraite (moteur de simulation)
include/core/LbmEngine.hpp          déclaration du moteur LBM D2Q9
include/core/FluidEngine.hpp        squelette alternatif (Stable Fluids), non branché
include/version.hpp.in              template de métadonnées (version, hash git, compilo)
src/core/LbmEngine.cpp              implémentation du solveur LBM (ce fichier)
src/core/FluidEngine.cpp
src/main.cpp                        fenêtre raylib, boucle principale, entrées, HUD
CMakeLists.txt                      build (FetchContent de raylib, presets Clang/libc++)
```

## Paramètres ajustables

Constantes en tête de `src/main.cpp` (namespace anonyme) :

| Constante | Rôle |
|---|---|
| `kGridWidth/Height` | résolution initiale de la grille (cellules) |
| `kSubSteps` | itérations LBM par image affichée (débit ↔ coût) |
| `kTau` | temps de relaxation BGK → pilote la viscosité/Reynolds |
| `kInletVelocity` | vitesse d'entrée en unités réseau (0.02–0.1 typiquement) |
| `kBrushMin/Max/Init` | bornes et valeur initiale du pinceau |
| `kRotStepDeg`, `kTransStepCells` | pas de rotation/translation des obstacles |
