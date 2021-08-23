"""# Potencjały + A*"""

"""Mapa i funckje do mapy"""

import geopandas as gpd
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry.multipolygon import MultiPolygon
import shapely.geometry

def explode(data):
#Explode MultiPolygon geometry into individual Polygon geometries in a shapefile using GeoPandas and Shapely 
#by mhweber <script src="https://gist.github.com/mhweber/cf36bb4e09df9deee5eb54dc6be74d26.js"></script>

    indf = gpd.GeoDataFrame.from_file(data)
    outdf = gpd.GeoDataFrame(columns=indf.columns)
    for idx, row in indf.iterrows():
        if type(row.geometry) == Polygon:
            outdf = outdf.append(row,ignore_index=True)
        if type(row.geometry) == MultiPolygon:
            multdf = gpd.GeoDataFrame(columns=indf.columns)
            recs = len(row.geometry)
            multdf = multdf.append([row]*recs,ignore_index=True)
            for geom in range(recs):
                multdf.loc[geom,'geometry'] = row.geometry[geom]
            outdf = outdf.append(multdf,ignore_index=True)
    return outdf

def printPoly(data):
  gdf = data
  gdf['color'] = ['b', 'g', 'g']
  gdf.plot(color=gdf['color'], figsize=(10, 10))

#Sprawdza czy punk jest jeziorem: Zwraca True jeśli tak
def isWater(geoMap,point):
  x = geoMap.loc[0].geometry.contains(point)
  #print('Status przynaleznosci punktu:',x)
  return x

geoTransform = explode("JezioroRadunskieGorne.gpkg")

"""A*"""

class Node:
    # Initialize the class
    def __init__(self, position:(), parent:()):
        self.position = position
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))


def astar_search(start, end, geodata, bounds, n_cells, buffer, shape_number):
    xmin, ymin, xmax, ymax = bounds #geodata.total_bounds
    #n_cells=100
    cell_size = (xmax-xmin)/n_cells
    #buffer_radius = 0.0003
    #shape_number = geodata.geometry.size
    #buffer = geodata.geometry.boundary.buffer(buffer_radius)
    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)
    max_iter = 0
    # Loop until the open list is empty
    while len(open) > 0 or max_iter == 100:
        max_iter = max_iter + 1
        check_land = 0
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            #path.append(start) 
            # Return reversed path
            return path[::-1]
        # Unzip the current node position
        (x, y) = current_node.position
        # Get neighbors
        #diag = cell_size*math.sqrt(2)/2
        #neighbors = [(x-cell_size, y), (x+cell_size, y), (x, y-cell_size), (x, y+cell_size)]
        neighbors = [(x-cell_size, y), (x+cell_size, y), (x, y-cell_size), (x, y+cell_size), (x - (cell_size), y+(cell_size)), (x - (cell_size), y-(cell_size)), (x + (cell_size), y+(cell_size)), (x + (cell_size), y-(cell_size))]
        # Loop neighbors
        for next in neighbors:
            # Get value from map
            position = Point(next[0], next[1])
            # Check if the node is a wall
            for b in range(0, shape_number):
                if buffer.loc[b].contains(position):
                    check_land = 1
                    
            if geodata.distance(position)[0] > 0:
                check_land = 1

            if(check_land == 1):
                continue
            # Create a neighbor node
            neighbor = Node(next, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Generate heuristics (Manhattan distance)
            #neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            #neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
            neighbor.g = math.sqrt(abs(neighbor.position[0] - start_node.position[0]))**2 + math.sqrt(abs(neighbor.position[1] - start_node.position[1]))**2
            neighbor.h = math.sqrt(abs(neighbor.position[0] - goal_node.position[0]))**2 + math.sqrt(abs(neighbor.position[1] - goal_node.position[1]))**2
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True

"""Siatka do mapy"""

xmin, ymin, xmax, ymax = geoTransform.total_bounds
n_cells=100
cell_size = (xmax-xmin)/n_cells
# projection of the grid
crs = "+proj=sinu +lon_0=0 +x_0=0 +y_0=0 +a=6371007.181 +b=6371007.181 +units=m +no_defs"
# create the cells in a loop
grid_cells = []
grid_points = []
for x0 in np.arange(xmin, xmax+cell_size, cell_size ):
    for y0 in np.arange(ymin, ymax+cell_size, cell_size):
        # bounds
        x1 = x0-cell_size
        y1 = y0+cell_size
        #grid_cells.append( shapely.geometry.box(x0, y0, x1, y1)  )
        grid_points.append([x0, y0])
#cell = gpd.GeoDataFrame(grid_cells, columns=['geometry'], crs=crs)

"""Potencjały"""

import math 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import linalg as LA
import numpy as np
from scipy.optimize import minimize
from numpy.random import rand
from matplotlib import cm
import sys
import pandas as pd

#Odległość Euklidesowa
def dist(P, Q):
  dist = math.sqrt((Q.x-P.x)**2 + (Q.y-P.y)**2)
  return dist

def getDistanceInMeters(pointA, pointB):
  earthRadius = 6371000
  latA = pointA.y * math.pi/180
  latB = pointB.y * math.pi/180
  diffLat = (pointA.y - pointB.y) * math.pi/180
  diffLng = (pointA.x - pointB.x) * math.pi/180
  a = math.sin(diffLat / 2) * math.sin(diffLat / 2) + math.cos(latB) * math.cos(latA) * math.sin(diffLng / 2) * math.sin(diffLng / 2)
  #b = 2 * math.asin(math.sqrt(a))
  b = 2 * math.atan2( math.sqrt(a), math.sqrt(1-a) )
  distance = round((earthRadius * b), 2)
  return distance

#Potencjały
class Potentials:
  def __init__(self, Gg=3, k=100, Gup=10, Gdown=5, Gac=0.0000004):
    self.Gg = Gg
    self.k = k
    self.Gup = Gup
    self.Gdown = Gdown
    self.Gac = Gac

  #Potencjał względem celu
  def PotentialGoal(self, position, goal):
      Pg = self.Gg * dist(position, goal)
      return Pg
  
  #Potencjał względem przeszkody
  def PotentialObstacle(self, position, obstacle):
    if dist(position, obstacle) != 0:
      Po = self.k/dist(position, obstacle)
    else:
      Po = 200
    return Po

  #Potencjał pod wiatr
  def PotentialUpwind(self, boat, position, windDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    #fi = windDirection - angle
    fi = angle - windDirection

    if fi < -180:
      fi = 360 + fi
    if fi > 180:
      fi = 360 - fi

    if 0 <= abs(fi)< 30:
      Pup = self.Gup*dist(boat, position)
    else:
      Pup = 0
    return Pup

  #Potencjał z wiatrem
  def PotentialDownwind(self, boat, position, windDirection):
    angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
    #fi = windDirection - angle
    fi = angle - windDirection

    if fi < -180:
      fi = 360 + fi
    if fi > 180:
      fi = 360 - fi

    if 0<= abs(abs(fi)-180) < 30:
      Pdown = self.Gdown * dist(boat, position)
    else:
      Pdown = 0
    return Pdown    

  #Potencjał - kara za zmianę kierunku
  def PotentialAngleChange(self, boat, position, windDirection, previousDirection):
      angle = math.degrees(math.atan2(position.y - boat.y, position.x - boat.x))
      if abs(previousDirection - angle) > 180:
        Pac = self.Gac * (360-abs(previousDirection - angle))
      else:
        Pac = self.Gac * abs(previousDirection - angle)
      return Pac

  #Suma Potencjałów 
  def PotentialSum(self, boat, position, goal, obstacle, windDirection, previousDirection):
      GoalP = Potentials.PotentialGoal(position, goal)
      ObstacleP = Potentials.PotentialObstacle(position, obstacle)
      UpwindP = Potentials.PotentialUpwind(boat, position, windDirection)
      DownwindP = Potentials.PotentialDownwind(boat, position, windDirection)
      AngleChangeP = Potentials.PotentialAngleChange(boat, position, windDirection, previousDirection)
      Psum = GoalP + ObstacleP + UpwindP + DownwindP + AngleChangeP
      return Psum

DaneX=np.zeros(1800)       #tablica na kolejne położenia łodzi x
DaneY=np.zeros(1800)       #tablica na kolejne położenia łodzi y
window = np.zeros(180)    #tablica na wartość potencjału na okręgu wokół łodzi
X = np.zeros(180)         #tablica na x na okręgu wokół łodzi
Y = np.zeros(180)         #tablica na y na okręgu wokół łodzi
Potentials = Potentials() #można podać parametry w kontruktorze


#szukanie minimum w pętli
iter_zew = 0
def find_min(boat, goal, obstacle, windDirection, iter_zew):
  max_iter = 100
  iter = 0
  previousDirection = 135
  radius = 0.00005  #rozmiar okręgu na którym szukamy punktów 3,96m
  radius2 = 0.0003
  shape_number = geoTransform.geometry.size
  xD = geoTransform.geometry.boundary.buffer(radius2)


  while iter < max_iter and getDistanceInMeters(boat, goal) > 5: 
    radius = 0.00005
    for c in range(0, shape_number):
      if xD.loc[c].contains(boat):
        radius = 0.0009
    index = 0
    #print(radius)
    DaneX[iter_zew] = boat.x
    DaneY[iter_zew] = boat.y
    #liczenie potencjału na okręgu dla punktów co 2 stopnie
    for angle in range(0, 360, 2):
      x = boat.x + radius*math.cos(math.radians(angle))
      y = boat.y + radius*math.sin(math.radians(angle))
      position = Point(x, y)
      check_land = 0
      for b in range(0, shape_number):
        if xD.loc[b].contains(position):
          check_land = 1
      if geoTransform.distance(position)[0] > 0:
        check_land = 1
      if check_land == 1:
        Potential = 100000
      else:
        Potential = Potentials.PotentialSum(boat, position, goal, obstacle, windDirection, previousDirection)
      window[index] = Potential
      X[index] = x
      Y[index] = y
      index = index + 1
    minIndex = np.argmin(window) #indeks dla minimum na okręgu
    previousDirection = math.degrees(math.atan2(Y[minIndex] - boat.y, X[minIndex] - boat.x))
    boat = Point(X[minIndex], Y[minIndex])   #zmiana pozycji statku na wyznaczone minimum
    iter = iter + 1
    iter_zew = iter_zew + 1
  return boat, iter_zew

# zostawienie punktów siatki na lądzie
bounds = geoTransform.total_bounds
buffer_radius = 0.0003
buffer = geoTransform.geometry.boundary.buffer(buffer_radius)
shape_number = geoTransform.geometry.size

start = None
end = None
data = grid_points.copy()
indeks_delete = []
for k in range(0,len(data),1):
  check_land = 0
  start = (data[k][0], data[k][1])
  position = Point(start)
              # Check if the node is a wall
  for b in range(0, shape_number):
    if buffer.loc[b].contains(position):
      check_land = 1
                      
  if geoTransform.distance(position)[0] > 0:
    check_land = 1
  #print("iteracja")
  #print(k)
  #print("check_land")
  #print(check_land)
  if check_land == 1:
    indeks_delete.append(k)

indeks_delete.sort(reverse=True)
print(indeks_delete)
data_pop = data.copy()
for v in range(len(indeks_delete)):
  data_pop.pop(indeks_delete[v])

#ustawienie startu i końca na siatce
#wklęsłóść
start_n = (17.954248, 54.214303)       #start    
end_n = (17.945373, 54.219224)         #koniec
#jedna wyspa
#start_n = (17.975696, 54.231898)       #start    
#end_n = (17.975696, 54.241756)         #koniec
#dwie wyspy
#start_n = (17.974556, 54.234057)       #start    
#end_n = (17.986122, 54.251899)         #koniec
#prosta
#start_n = (17.955683, 54.223704)       #start    
#end_n = (17.968805, 54.231189)         #koniec
bliskosc_gridS = []
for u in range(len(data_pop)):
  bliskosc_gridS.append(abs(start_n[0] - data_pop[u][0])+abs(start_n[1] - data_pop[u][1]))
minimum_gridS = np.argmin(bliskosc_gridS)

bliskosc_gridE = []
for t in range(len(data_pop)):
  bliskosc_gridE.append(abs(end_n[0] - data_pop[t][0])+abs(end_n[1] - data_pop[t][1]))
minimum_gridE = np.argmin(bliskosc_gridE)

start = (data_pop[minimum_gridS][0], data_pop[minimum_gridS][1])
end = (data_pop[minimum_gridE][0], data_pop[minimum_gridE][1])

#A* wywołanie
path = astar_search(start, end, geoTransform, bounds, n_cells, buffer, shape_number)
path = path[1::4]
#goal = Point(17.949000,54.223725)           #Punkt docelowy

goal = Point(end) 
obstacle = Point(60.959218,60.220071)       #Położenie przeszkody
#boat = Point(17.951814,54.215700)           #tu failuje
boat = Point(start)           #Pozycja startowa statku
#Kierunek wiatru
windDirection = 90

#wywołanie pętli z potencjałami na punktach z A*
for w in range(len(path)-1):
  boat = Point(path[w])
  goal = Point(path[w+1]) 
  wynik, iter_zew = find_min(boat, goal, obstacle, windDirection, iter_zew)

print('Wynik:', wynik.x, wynik.y)

#plotowanie trasy na pustym wykresie
fig = plt.figure(figsize=(12,6), dpi=100)
ax = fig.add_subplot(111)
ax.azim = 270
ax.dist = 10
ax.elev = 90
plt.xlim([17.955683, 17.968805])
plt.ylim([54.223704, 54.231189])
plt.xlabel("X axis label")
plt.ylabel("Y axis label")
ax.scatter(DaneX, DaneY, s=0.5, c='red')
ax.scatter(obstacle.x, obstacle.y, c='red')
#ax.scatter(goal.x, goal.y, c='green')
circleGoal = plt.Circle((goal.x, goal.y), 0.00005, color='green', fill=False)
ax.add_patch(circleGoal)
plt.show()

#przypisanie wynikó kolejnych iteracji do formatu GeoDataFrame
kappa_licznik = 0
for x in range(0, 1800):
  if DaneX[x] != 0:
    kappa_licznik = kappa_licznik + 1
df = pd.DataFrame(
    {'Latitude': DaneY[0:kappa_licznik],
     'Longitude': DaneX[0:kappa_licznik]})
gdf = gpd.GeoDataFrame(
    df, geometry=gpd.points_from_xy(df.Longitude, df.Latitude))
#ustawienie mapy jeziora jako tła
gdf1 = geoTransform
gdf1['color'] = ['b', 'g', 'g']
base = gdf1.plot(color=gdf1['color'], figsize=(15, 15))

#kappa
pathX = np.zeros(1800)
pathY = np.zeros(1800)
for p in range(len(path)):
  (pathX[p], pathY[p]) = path[p]

kappa_licznik = 0
for x in range(0, 1800):
  if pathX[x] != 0:
    kappa_licznik = kappa_licznik + 1
df = pd.DataFrame(
    {'Latitude': pathY[0:kappa_licznik],
     'Longitude': pathX[0:kappa_licznik]})
gdf2 = gpd.GeoDataFrame(
    df, geometry=gpd.points_from_xy(df.Longitude, df.Latitude))
base2 = gdf.plot(ax=base, color='black', markersize=1)
#plotowanie wyników symulacji na jeziorze
gdf.plot(ax=base2, color='red', markersize=1)