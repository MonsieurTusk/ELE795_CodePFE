Module de capture d'ecran automatique 

Librairie necessaire : PyAutoGUI
https://pyautogui.readthedocs.io/en/latest/

Installation 
	pip3 install pyautogui
	sudo apt-get install scrot
	sudo apt-get install python3-tk
	sudo apt-get install python3-dev

Les dernieres versions (21,04 +) de Ubuntu fonctionnent maintenant sur le serveur Wayland. Une particularite de ce serveur est qu'il protege les utilisateurs des applications potentiellement dangereuses. Les applications ne peuvent donc plus voir ce qui se trouve a l'ecran, ce qui fait en sorte que la fonction screenshot de PyAutoGUI retourne une image noir.

La solution a ce probleme est de retourner au serveur Xorg qui lui permet la capture d'ecran automatique. Il est possible de changer de serveur dans les parametres au moment du demarrage, il faut choisir Ubuntu on Xorg.
