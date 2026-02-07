# Manuel de test – Carte STM32 + MCSDK 
## Pilotage moteur via UART (Arduino Serial Monitor – mode ASCII)

## **Objectif du test**

Ce test permet de :

* vérifier que la communication UART entre le PC et la carte STM32 fonctionne,

* commander le moteur brushless (start, stop, vitesse, position),

* lire l’état interne de la machine d’état MCSDK,

* lire les défauts (faults) générés par le SDK,

* valider le bon fonctionnement de la logique de sécurité (fault latched).

Le test se fait sans interface graphique, uniquement via le Serial Monitor de l’Arduino IDE en mode texte (ASCII).

## **Matériel et logiciels**

### Matériel :

* Carte STM32 (Nucleo / carte de commande)

* Carte puissance + moteur BLDC / PMSM

* Câble USB (ST-Link / UART virtuel)

* Alimentation adaptée à la carte puissance

### Logiciel

* STM32CubeIDE (projet généré avec MCSDK)

* Arduino IDE (uniquement pour le Serial Monitor)

## **Connexion et configuration UART**

### Sur la carte STM32 :

* UART configuré à :

** Baudrate : 115200

** 8 bits, no parity, 1 stop bit

* Réception UART par interruption

* Messages envoyés en ASCII

### Sur le PC (Arduino IDE) :

* Ouvrir Arduino IDE

* Aller dans Tools → Serial Monitor

* Régler :

** Baudrate : 115200

** Line ending : Newline (ou Both NL & CR)

* Sélectionner le bon port COM

## **Principe de fonctionnement logiciel**

* Les commandes sont envoyées sous forme de texte (ASCII)

* Chaque ligne reçue est analysée

* Une commande valide déclenche une action MCSDK

* La STM32 répond toujours par un message texte

Exemple :
```
start
→ STATE=RUN
```

## **Liste des commandes disponibles**

### Commandes de base

* `start` : Démarre le moteur
* `stop` : Arrête le moteur
* `reset` : Arrêt + remise à zéro de la vitesse
* `ack` : Acquitte un défaut MCSDK

### Commande vitesse

```
v200
```

* Définit la consigne de vitesse

* Valeur bornée dans le code (ex : 50 à 500)

* La vitesse est appliquée au prochain start ou immédiatement si déjà en RUN

Réponse typique :
```
VITESSE SET = 200
```

### Commande position

```
pos90
```

* Commande une position mécanique (en degrés)

* Utilise MCI_ExecPositionCommand

* Exemple : rotation de 90° en 2 secondes

Réponse typique :
```
POSITION CMD = 90 deg
```

### Commandes d’information

```
status
```
Renvoie l’état de la machine d’état MCSDK :
```
status
→ Etat moteur : 6
```

Correspondances courantes :

* `O` : IDLE
* `4` : START
* `6` : RUN
* `8` : STOP
* `10` : FAULT_NOW
* `11` : FAULT_OVER

```
fault
```
Renvoie les défauts du MCSDK :
```
fault
→ FAULT = 0004
```

```
help
```
Affiche toutes les commandes disponibles :
```
help
→ Commandes:
→ help
→ status
→ fault
→ start
→ stop
→ reset
→ vX
→ posY
```

## **Gestion des défauts (Fault Management)**

### Principe

* Le MCSDK gère automatiquement les défauts matériels :

** surcourant

** surtension

** surchauffe

* En cas de défaut :

** la machine d’état passe à FAULT_NOW

** le moteur est arrêté automatiquement

### Logique ajoutée dans l’application

* Un fault_latched bloque toute commande moteur

* Seules les commandes ack ou reset sont acceptées

* Le moteur ne peut pas redémarrer tout seul

Exemple :
```
ERREUR
Etat = FAULT_NOW
Attente ACK ou RESET
````

## **Séquence de test recommandée**

* Brancher la carte et alimenter

* Ouvrir le Serial Monitor

* Vérifier message de démarrage :

```
COM UART OK!
```

* Tester :

```
status
```

* Régler une vitesse :

```
v200
```

* Démarrer :

```
start
```

* Arrêter :

```
stop
```

* Tester une position :

```
pos90
```

## **Points importants à retenir**

* Les courants moteur et la FOC tournent beaucoup plus vite que la boucle main

* Le SysTick sert uniquement à :

** lecture capteurs

** communication

** supervision

* La commande moteur réelle est temps réel (interruptions MCSDK)

* Le UART est volontairement lent et lisible

## **Conclusion**

Ce mode de test :

* est robuste

* est pédagogique

* permet de comprendre le MCSDK sans IHM complexe

* est parfaitement adapté à une phase de validation et de débogage
