**Résolveur de Labyrinthe utilisant PyAmaze/Algorithme de Recherche en Profondeur (DFS)**

Ce script Python utilise la bibliothèque PyAmaze pour créer et résoudre des labyrinthes, pour la première solution le labyrinthe est représenté sous forme de grille, et un agent le parcourt pour trouver une solution.Pour la deuxième solution utilise la bibliothèque PyAmaze pour créer un labyrinthe et résoudre ce labyrinthe en utilisant l'algorithme de recherche en profondeur (DFS) Voici un README simple décrivant comment utiliser le code.

**Prérequis :**
- Python 3.x
- Bibliothèque PyAmaze (`pip install pyamaze`)
- ROS 

**Utilisation :**
1. **Installation :**
   - Assurez-vous d'avoir Python installé sur votre système.
   - Installez la bibliothèque PyAmaze en exécutant `pip install pyamaze` dans votre terminal ou votre invite de commande.
   - Assurez-vous d'avoir ROS installé sur votre système.

2. **Exécution du Script :**
   - Enregistrez le code fourni dans un fichier Python (par exemple, `maze_solver.py`).
   - Exécutez le script à l'aide d'un interpréteur Python : `python maze_solver.py`.

3. **Compréhension du Code :**
   laby2.py
   - Le script importe des classes de la bibliothèque PyAmaze pour créer un labyrinthe et un agent.
   - Il définit une structure de labyrinthe et un chemin prédéfini pour que l'agent le suive.
   - L'agent traverse le labyrinthe en suivant le chemin prédéfini, laissant des empreintes.
   - Le résolveur de labyrinthe visualise ensuite le chemin de l'agent à travers le labyrinthe.
   laby.py
   - Le script importe des classes de la bibliothèque PyAmaze pour créer un labyrinthe, un agent et exécuter l'algorithme DFS.
   - Il définit une fonction `DFS` qui prend le labyrinthe et la cellule de départ en argument et retourne le chemin exploré par l'algorithme DFS.
   - Ensuite, il crée un labyrinthe et utilise la méthode `CreateMaze` pour générer le labyrinthe ou charger un labyrinthe à partir d'un fichier CSV.
   - L'algorithme DFS est utilisé pour trouver un chemin à travers le labyrinthe, en marquant les cellules qui ont plusieurs chemins possibles.
   - Un agent est placé dans le labyrinthe à la position de départ, et le chemin trouvé par DFS est tracé à l'aide de la méthode `tracePath`.
   - Enfin, le labyrinthe est affiché avec le chemin exploré et le chemin tracé par l'agent.
   robot_control.py
   - Le script importe des classes des bibliothèques geometry_msgs.msg et sensor_msgs.msg pour récuperer les valeurs du laser et déplacer le robot.
   - Il définit les méthodes pour que le robot se déplace dans le labyrinthe.
   - Il initialise la node, le publisher et le subscriber.
   - Il crée un objet pour le contrôler.
   project.py
   - Le script importe la classe RobotControl du fichier robot_control.py.
   - Il définit toute la logique pour résoudre le labyrinthe en utilisant les méthodes de la RobotControl.

5. **Personnalisation :**
   - Vous pouvez personnaliser la taille du labyrinthe, la position de départ de l'agent et le chemin prédéfini selon vos besoins.
   - Explorez la documentation de la bibliothèque PyAmaze pour des options de personnalisation supplémentaires et des fonctionnalités.
   - Vous pouvez ajuster les valeurs récupérées par le lidar ou les vitesses en fonction de vos besoin.


Ce README fournit un aperçu de base de l'utilisation du script de résolution de labyrinthe. Pour des instructions et des explications plus détaillées, référez-vous aux commentaires dans le code et à la documentation de la bibliothèque PyAmaze.







