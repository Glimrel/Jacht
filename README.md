# Jacht

Projekt magisterski autonomicznej łodzi żaglowej <br/>
Autorzy: Tomasz Ujazdowski, Damian Kułaga<br/>

Projest wykorzystuje ROS, Node-RED.<br/>

Folder Algorytmy Wymiany Danych zawiera węzły wymiany informacji pomiędzy systemem decyzyjnym, a kontrolno pomiarowym.
Folder Basic Symulation zawiera pliky wykonawcze dla symulacji utrzymywania kursu.
Folder Fuzzy zawiera aktualne algorytmy rozmyte utrzymywania kursu, ustawienia żagli i kontroli przechyłu.
Folder SLOW zawiera węzły grafu systemu decyzyjnego wykonywane z obniżoną częstotliwością.
Folder Symulacje zawiera pliky wykonawcze i węzły dla symulacji algorytmów.
Folder msg zawiera aktualne typy msg dla pakietu ROS.
Folder src zawiera aktualne pliki źródłowe projektu w pakiecie ROS.
Astar.py to plik zawierający algorytm A* wraz z metodą potencjalów.
Dashboard.json to plik zawierający kod Node-RED dla panelu kontrolnego.
Node-RED.json to plik zawierający kod Node-RED dla systemu decyzyjnego.
TransmitHub.json to plik zawierający kod Node-RED dla wymiany informacnji.
