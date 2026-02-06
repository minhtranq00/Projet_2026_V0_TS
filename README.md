# Ce document décrit le code "main" utile pour le microcontrôleur

## **Informations générales**

* Pour reconfigurer les broches du microcontrôleur, STMicroelectronics utilise un système de balise dans le fichier main pour conserver les modifications ajoutées.
Il faut donc ajouter les lignes de code dans les /* USER CODE ... */

* Il est fortement conseillé de lire le rapport de stage de Pierrick Lienard, notamment la partie sur la programmation du microcontrôleur.

## **Description du code**

### **/* USER CODE BEGIN Includes */**
Contient les #include utiles au projet

### **/* USER CODE BEGIN PD */**
Chaque #define permet d'activer ou désactiver une partie de code.
Cela est utile pour le développement et pour tester une partie de code.
Les if associées aux define (dans le while(1)) sont imbriquées, donc une seule partie peut être active à la fois.
NB : les parties de code désactivées, ne sont pas compilées.

### **/* USER CODE BEGIN 0 */**
Déclaration des variables :
* 6 variables identifiant pour la communication UART TX (voir rapport de stage pour la description du protocole UART)
* 4 variables identifiant pour la communication UART RX
* 5 variables utiles pour stocker la mesure réalisée
* 2 variables pour les ordres, la vitesse (valeur) et le type d'ordre (pas encore utilisés dans le code (le type peut être : 0-> vitesse, 1-> position, 2-> couple)) 
* 2 variables pour la communication UART -> buffer de réception et la définition du nombre d'octets reçus par trame (défini à 3 voir rapport de stage)
* 2 variables pour les erreurs, "buffer_error" ->buffer pour envoyer un message d'erreur, "faults" -> stocke le message d'erreur de la machine d'état
* 4 variables utiles pour la commande en boucle ouverte
* 1 variable (new_order) qui est utile pour indiquer qu'une trame UART a été reçue. 
* 2 struct utilisés pour créer des flags qui s'activent toutes les X ms.  
* 1 fonction créée pour transmettre un message (caractères) 

### **Début du main**
Le code initialise les différentes parties (code généré par les logiciels MCWorkbench et STM32CubeMX)

### **/* USER CODE BEGIN 2 */**
* HAL UART Receive IT -> lance la réception des trames UART en interruption (avec l'adresse de l'UART concerné, le buffer de stockage et le nombre d'octets par réception) 

* Phase de démarrage, pour vérifier que le code se lance bien deux indicateurs sont définis :
 * Visuel LED verte : allume la LED verte 1 seconde puis l'éteint.
 * COM UART : transmet le message "COM UART OK!\n" à l'ordinateur de bord.
 * Attend une seconde avant de continuer le programme.

* 3 variables utiles pour la commande de courant. NB : les variables ne sont pas connues si elles sont déclarées avant la phase d'initialisation.

* 3 variables utiles pour la commande de boucle ouverte. Même NB que pour les 3 variables d'avant.

### **/* USER CODE BEGIN 4 */**
* void HAL UART RxCpltCallback(UART HandleTypeDef *huart) -> c'est une fonction par défaut qui est appelée à chaque fois que le nombre d'octets reçus est égal à la valeur définie dans /* USER CODE BEGIN 0 */.
Dans cette fonction on active le flag et relance l'interruption.
* void HAL SYSTICK Callback(void) -> permet d'incrémenter les flags de durée (il ne faut rien modifier ici)

### **/* USER CODE BEGIN 3 */**
Dans cette partie, nous sommes dans la boucle infinie.
#### **#if ON_OFF**
Active et désactive le moteur avec les fonctions de l'API avec une période de 20s.

#### **#if Vitesse** 
programme qui modifie la vitesse avec une période de 10s.

#### **#if cmd_rasb** 
Ce programme est le plus développé à la fin du stage. 
Deux schémas blocs de principe sont disponibles dans le rapport de stage. 
Comme on peut le voir sur le schéma bloc du programme principal, nous allons nous assurer qu'il n'y a pas d'erreurs dans la machine d'état et que la température est inférieure à la limite définie. 
Pour ce faire on a deux if imbriqués :

* premier if : on vérifie que l'état n'est pas une erreur. -> Si c'est le cas, on envoie un message avec l'état de la machine d'état et un message avec le type d'erreur. Les valeurs sont formatées dans une chaine de caractères avant de l'envoyer en UART. 
Ensuite un while bloquant est ajouté qui attend un acquittement pour retourner dans le programme principal. Malheureusement l'acquittement ne marche pas et l'état reste en erreur.

* Deuxième if : on vérifie la valeur de la température. -> Si c'est le cas on envoie l'identifiant de température trop élevée (en UART). 

* Si les deux if sont négatifs, alors le système est en bonne condition. 
On peut donc passer aux fonctions utiles aux mouvements du moteur. 
	* if(AppFlags.read_sensors) -> Dans ce if on vérifie si le flag des capteurs est actif. 
Cela permet d'avoir une fréquence de lecture des capteurs régulière et définie. 
Dans notre cas il est défini à 100ms.
Dans le if on commence par reset le flag à 0.
Ensuite on va récupérer les valeurs de tension, courant, tension, etc, en utilisant des fonctions générées par MCWorkbench. Ensuite on communique les valeurs à l'ordinateur de bord. Pour ce faire on commence par transmettre l'identifiant puis la valeur associée.
Par la suite on vérifie si l'état est RUN. 
Si c'est le cas, alors on envoie l'ordre reçu au moteur. Dans cette version du code, il y a aussi un message RUN transmis à l'ordinateur de bord. Il est utile juste pour savoir quand on est dans l'état RUN, il peut être supprimé.


	* if(new_order) -> Si le if est vrai alors nous avons reçu un nouvel ordre et nous allons le traiter.
On commence donc par reset le flag, ensuite on va vérifier le premier octet dans le buffer de réception.
Ce premier octet contient l'identifiant de l'ordre.
En fonction de l'identifiant, différentes actions vont être réalisées.
Si c'est un ordre de start, stop ou reset, on active/désactive le moteur et pour le reset on définit la vitesse à 500 (3000tr/min). 
Si c'est un ordre de vitesse, alors on lit les deux octets qui suivent. 
Ils contiennent la valeur de consigne sur 16bits.
Ensuite on vérifie que la valeur n'est pas trop basse (si c'est le cas on définit la consigne à vitesse minimale), pareil pour une vitesse trop haute. Si on ne dépasse aucune des deux limites, alors on récupère toute la consigne envoyée.
