from collections import defaultdict
import numpy as np

def input_matrix():
    size= int(input("tamaño de la matriz de adyacencia: "))
    a=np.empty((size,size))  #crea una matriz de adyacencia

    for f in range(size):   ##el usuario introduce valores en la matriz
        for c in range(size):
            a[f][c] = int(input("Elemento %d,%d : " % (f,c) ))
    print("matriz A: ")
    print(a)
    return (a)

def resize_input(a):
    
    size=np.shape(a)#tomamos el tamaño de a
    size=size[0]#como es cuadrada solo tomamos el primer elemento
    for f in range(size):
        for c in range(size):
            if(f<c):
                a[f][c]=0#la hacemos triangular inferior
    arraysillo=[]
    for f in range(size):
        for c in range(size):
            if (a[f][c]==1):
                arraysillo=np.append(arraysillo,(f,c))#creamos un arreglo con los pares del grafo

    return arraysillo


#usamos la lista de adyacencia para manejar el grafo
class Eulerian: 
	def __init__(self,vertices): 
		self.V= vertices # de vertices 
		self.graph = defaultdict(list) # creamos un diccionario para guardar el grafo 
		self.Time = 0

	
	def addEdge(self,u,v): 
		self.graph[u].append(v) 
		self.graph[v].append(u) 

	# remueve vertices del grafo si es necesario	 
	def rmvEdge(self, u, v): 
		for index, key in enumerate(self.graph[u]): 
			if key == v: 
				self.graph[u].pop(index) 
		for index, key in enumerate(self.graph[v]): 
			if key == u: 
				self.graph[v].pop(index) 

	#usamos DFS para buscar vertices alcanzables al vertice actual.
	def DFSCount(self, v, visited): 
		count = 1
		visited[v] = True
		for i in self.graph[v]: 
			if visited[i] == False: 
				count = count + self.DFSCount(i, visited)		 
		return count 

	# checa si el siguiente par de vertices son validos 
	def isValidNextEdge(self, u, v): 
		#el vertice u-v es valido en 2 casos
		# si v es el unico adyacente a u 
		if len(self.graph[u]) == 1: 
			return True
		else: 

 
			#si hay multiplices adyacentes, y u-v no es puente, si es puente hacer lo siguiente...
			#cuenta los vertices alcanzables por u	
			visited =[False]*(self.V) ##crea un vector para corroborar que nodos han sido visitados
			count1 = self.DFSCount(u, visited) 

				#elimina la arista u,v y despues cuenta los vertices alcanzable de u
				#usando DFS
			self.rmvEdge(u, v) 
			visited =[False]*(self.V) 
			count2 = self.DFSCount(u, visited) 

			#2.c) agrega de nuevo el vertice al arreglo 
			self.addEdge(u,v) 

			# 2.d)si el contador 1 es mayor que el contador 2 es un puente 
			return False if count1 > count2 else True


	#imprime el camino euleriano iniciando desde u
	def printEulerUtil(self, u): 
		#recorre todos los vertices adyacentes
		for v in self.graph[u]: 
			#si la arista entre u y v no a sido removida es una salida valida
			if self.isValidNextEdge(u, v): 
				print("%d-%d " %(u,v)), 
				self.rmvEdge(u, v) 
				self.printEulerUtil(v) 


	#como base esta funcion primero encuentra si hay vertices con grado impar, si si hay entonces llama 
	#directamente a printEulerUtil() para imprimir camino si no imprime el ciclo
	def printEulerTour(self): 
		#grado impar?? 
		u = 0
		for i in range(self.V): 
			if len(self.graph[i]) %2 != 0 : ##si el numero de vertices en el grafo es impar:
				u = i 
				break
		# imprime el recorrido.... 
		self.printEulerUtil(u) 

#############################################################################INICIO DE HAMILTON##############################
class Hamiltonian(): 
	def __init__(self, vertices): 
		self.graph = [[0 for column in range(vertices)] 
							for row in range(vertices)] 
		self.V = vertices 

    # mira si el vertice es adyacente al anterior y si no esta en Path
	def isSafe(self, v, pos, path): 
		# corrobora el vertice actual y el ultimo son adyacentes
		if self.graph[ path[pos-1] ][v] == 0: 
			return False

		# Checa si no esta en Path 
		for vertex in path: 
			if vertex == v: 
				return False

		return True

	def hamCycleUtil(self, path, pos): 
		if pos == self.V: 
#el ultimo vertice debe de ser adyacente al primero para que se haga ciclo""" 
			if self.graph[ path[pos-1] ][ path[0] ] == 1: 
				return True
			else: 
				return False


        #siguiendo el algoritmo, probamos vertices para ver si son
        #candidatos, si si lo son agregamos al path (no probamos el 0)
		for v in range(1,self.V): 

			if self.isSafe(v, pos, path) == True: 

				path[pos] = v 
 
				if self.hamCycleUtil(path, pos+1) == True: 
					return True
                #si el vertice no nos sirve lo quitamos poniendole -1

		return False

	def hamCycle(self): 
		path = [-1] * self.V ##inicializamos el vector del camino con un -1 para distinguir los no visitados

			#el vertice 0 es el primero en el camino
            #si hay un ciclo hamiltoniano eso no importa,
            #va inciiar de donde sea
		path[0] = 0

		if self.hamCycleUtil(path,1) == False: 
			print ("NO HAY SOLUCION SEGUN EL ALGORITMO...\n") 
			return False

		self.printSolution(path) 
		return True

	def printSolution(self, path): 
		print ("CICLO DE HAMILTON:") 
		for vertex in path: 
			print (vertex, end = "-") 
		print (path[0], "\n") 

print("\n\n")
print("\033[1;33m"+"Hamilton"+'\033[0;m')
print("\t\t\tNota: \nPara el caso de caminos/ciclos hamiltonianos no existe algoritmo optimo para la toma de desiciones, todos los algoritmos tienen errores y pueden equivocarse...")
input("presione enter...")
#a = [ [0,1,0,1,1], [1,0,1,1,1],[0,1,0,1,0],[1,1,1,0,1],[1,1,0,1,0]  ] 
#a=[ [0, 1, 0, 1, 0], [1, 0, 1, 1, 1],[0, 1, 0, 0, 1,],[1, 1, 0, 0, 1],[0, 1, 1, 1, 0]]
a=input_matrix()##INTRODUCE LA MATRIZ DE ADYACENCIA FORZADA A SER CUADRADA
g2=Hamiltonian(len(a))
g2.graph=a
g2.hamCycle()

print("\n\n")
print("\033[1;33m"+"Euler"+'\033[0;m')
pairs_array=resize_input(a)##CONVIERTE A PARES PARA INTRODUCIRLO AL ALGORITMO EULERIANO
len_pairs=int(len(pairs_array)/2)##NOS DA EL NUMERO DE PARES
pairs_array=np.reshape(pairs_array,(len_pairs,2))##CREA UN ARREGLO CON LOS PARES DEL GRAFO
g1 = Eulerian(len_pairs) #ENVIA COMO PARAMETRO EL NUMERO DE PARES AL ALGORITMO
for v in range(len_pairs):
    g1.addEdge(int(pairs_array[v,0]),int(pairs_array[v,1]))##VA AGREGANDO PAR POR PAR A EULERIAN
g1.printEulerTour() ##IMPRIME EL RESULTADO DE EULER
print("\n\n\n")

from collections import defaultdict 
  ##############################################################algoritmo de pesos###########################################################
class pesos: 
  

	#encuentra el vertice con menos valor de distancia de los que estan en la cola
    def minDistance(self,dist,queue): 
        # Initialize min value and min_index as -1 
        minimum = float("Inf") 
        min_index = -1
          
		#toma elementos del arreglo de distancias, de los cuales estan en la cola
        for i in range(len(dist)): 
            if dist[i] < minimum and i in queue: 
                minimum = dist[i] 
                min_index = i 
        return min_index 
  
  

	#imprime el camino mas corto desde 0 hasta j
    def printPath(self, parent, j): 
          
        #si j es 0 entonces....
        if parent[j] == -1 :  
            print (j) 
            return
        self.printPath(parent , parent[j]) 
        print (j)
          

	#imprime el arreglo de las distancias
    def printSolution(self, dist, parent): 
        src = 0
        print("Vertex \t\tDistance from Source\tPath") 
        for i in range(1, len(dist)): 
            print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])), 
            self.printPath(parent,i) 
  
  

	#funcion que usa el algoritmo de dijkstra con la matriz de adyacencia
    def dijkstra(self, graph, src): 
  
        row = len(graph) 
        col = len(graph[0]) 
  
		# inicializamos todos los vertices como infinito, la salida va imprimir el camino mas corto desde 0 a n 
        dist = [float("Inf")] * row 

		#aqui se almacenan el camino mas corto del arbol
        parent = [-1] * row 

		#sabemos que la distancia del vertice 0 consigo mismo es 0
        dist[src] = 0
      
        # agrega todos los vertices al stack 
        queue = [] 
        for i in range(row): 
            queue.append(i) 
              
        #encuentra el camino mas corto... 
        while queue: 
  
            # selecciona al vertice de menos peso que todavia estan en la cola 
            u = self.minDistance(dist,queue)  
  
            # quita el elemento innecesario      
            queue.remove(u) 
  
			# modifica el valor de dist y del padre de su vertice adyacente respecto al vertice actual, considerando solo los que estan en la cola
            for i in range(col): 

				#modifica a dist si esta en la cola, hay un vertice entre u y i y el peso total del camino desde el inicio y i pasando por u es mas pequeño que el valor de  dist
                if graph[u][i] and i in queue: 
                    if dist[u] + graph[u][i] < dist[i]: 
                        dist[i] = dist[u] + graph[u][i] 
                        parent[i] = u 
  
  
        # imprime solucion
        self.printSolution(dist,parent) 
input("matriz con pesos...")
print("\033[1;33m"+"Iniciando algoritmo de matriz con pesos..."+'\033[0;m')
g= pesos() 

graph=input_matrix() 
#graph= [[0,3,7,8,14],[3,0,9,5,11],[7,9,0,4,8],[8,5,4,0,6],[14,11,8,6,0]]
#graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],[4, 0, 8, 0, 0, 0, 0, 11, 0],[0, 8, 0, 7, 0, 4, 0, 0, 2],[0, 0, 7, 0, 9, 14, 0, 0, 0],[0, 0, 0, 9, 0, 10, 0, 0, 0],[0, 0, 4, 14, 10, 0, 2, 0, 0],[0, 0, 0, 0, 0, 2, 0, 1, 6], [8, 11, 0, 0, 0, 0, 1, 0, 7], [0, 0, 2, 0, 0, 0, 6, 7, 0] ] 
# Print the solution 
g.dijkstra(graph,0)
input("...")
  

