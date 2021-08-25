# Jacht

Projekt magisterski autonomicznej łodzi żaglowej <br/>
Autorzy: Tomasz Ujazdowski, Damian Kułaga<br/>

Projest wykorzystuje ROS, Node-RED.<br/>

Folder Algorytmy Wymiany Danych zawiera węzły obsługujące UART pomiędzy systemem decyzyjnym, a kontrolno-pomiarowym.<br/>
Folder Basic Symulation zawiera pliky wykonawcze dla symulacji utrzymywania kursu.<br/>
Folder Fuzzy zawiera aktualne algorytmy rozmyte utrzymywania kursu, ustawienia żagli i kontroli przechyłu.<br/>
Folder SLOW zawiera węzły grafu systemu decyzyjnego wykonywane z obniżoną częstotliwością.<br/>
Folder Symulacje zawiera pliky wykonawcze i węzły dla symulacji algorytmów.<br/>
Folder msg zawiera aktualne typy msg dla pakietu ROS.<br/>
Folder src zawiera aktualne pliki źródłowe projektu w pakiecie ROS.<br/>
Astar.py to plik zawierający algorytm A* wraz z metodą potencjalów.<br/>
Dashboard.json to plik zawierający kod Node-RED dla panelu kontrolnego.<br/>
Node-RED.json to plik zawierający kod Node-RED dla systemu decyzyjnego.<br/>
TransmitHub.json to plik zawierający kod Node-RED dla wymiany informacnji.<br/>
