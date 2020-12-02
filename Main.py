import numpy as np
matriz=[] ###matriz vacia
Deg = []
contador=0

##pedimos al usuario el tamaño de la matriz 
size= int(input("tamaño de la matriz de adyacencia: "))
issym=1 #inicialmente suponemos que la matriz es simetrica osea grafo no dirigido, si no se cumple se cambiara la bandera

for i in range(size):
	 matriz.append([0]*size)  ##inicializa la matriz de adyacencia con 0
	 Deg.append([0])
for f in range(size):   ##el usuario introduce valores en la matriz
	for c in range(size):
		matriz[f][c] = int(input("Elemento %d,%d : " % (f,c) ))



#------------------------------------------------------------#
for f in range(size):   #corrobora si la matriz es un grafo dirigido o no y calcula el grado de cada vertice 
	for c in range(size):

		if (matriz[f][c]==1): #contador de grados del vertice
			contador=contador+1
			print (contador)
			
		if (matriz[f][c]==matriz[c][f]):		
			"""print("ok %d,%d " %(f,c) )"""
		else:
			issym=0  #bandera de simetria que indica si el grafo es dirigido o no
			"""print("false %d,%d " %(f,c))"""
	Deg[f]=[contador] #añade al vector el numero de grados de cada vertice
	contador=0

if (issym==1):
	print('Grafo es no dirigido')
elif (issym==0):
	print('grafo es dirigido')
#------------------------------------------------------#

"""2 vertices de grado impar  hacen un camino euleriano
	todos los vertices de grado par hacen ciclo euleriano
""" 


print('grados de cada vertice:')
print(Deg)
print ('')
print('Matriz de adyacencia')
print (matriz)
input("presiona enter para salir...")
