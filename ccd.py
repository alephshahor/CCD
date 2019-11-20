#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - Curso 2014/2015
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure(1)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
 # raw_input()
  plt.clf()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)

  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    print("TEMP: " + str(tmp))
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# Por como está escrito el script el número de articulaciones es siempre fijo,
# en este caso 4 articulaciones.
# Valores articulares arbitrarios para la cinemática directa inicial
th=[0.,0.,0.]
a =[5.,5.,5.]
L = sum(a) # variable para representación gráfica
EPSILON = .01

#plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]

# O es una matriz de pares que contienen la posición de las cuatro
# articulaciones que componen el brazo.
# Al inicializarse del modo en que se inicializa, lo que tenemos es un vector
# de tamaño cuatro tal que O = [ 1, 2, 3, 4]. Al parecer costaba mucho decir
# O = [0] * (len(th) + 1)
O=range(len(th)+1) # Reservamos estructura en memoria

O[0]=cin_dir(th,a) # Calculamos la posicion inicial

print "- Posicion inicial:"
muestra_origenes(O[0])

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  j = len(th) - 1

  for i in range(len(th)):

    print(O[i][j])

    pos_artic_actual = O[i][j]
    pos_effec_final = O[i][len(th)]

    '''
    La lógica que estamos usando es la siguiente:

    Queremos obtener en primer lugar para cada articulación un ángulo theta
    que permita acercar el efector final al punto objetivo.

    Para ello debemos de calcular dos vectores:

    * actual_a_effect --> Es el vector desde la articulación actual en la que
    estamos iterando al efector final.

    * actual_a_objet  --> Es el vector desde la articulación actual en la que
    estamos iterando al punto objetivo.

    El ángulo theta es calculado a partir del arcocoseno del mismo, que se obtendría
    a partir de los dos vectores del siguiente modo:

    cos_theta = (ae / |ae|) * (ao / |ao|)

    Donde * simboliza el producto punto.

    La dirección de giro del ángulo theta se calcular a partir del producto vectorial
    de ambos vectores, este nos da un vector perpendicular al plano a partir del cual
    siguiendo la regla de la mano derecha determinamos la dirección.

    direccion = (ae / |ae|) x (ao / |ao|)

    Donde x simboliza el producto vectorial.

    '''

    # Declaramos el vector desde la articulacion actual al efector final.
    actual_a_effec = [0] * 2

    actual_a_effec[0] = pos_effec_final[0] - pos_artic_actual[0]
    actual_a_effec[1] = pos_effec_final[1] - pos_artic_actual[1]
    actual_a_effec_mag = sqrt(pow(actual_a_effec[0],2) + pow(actual_a_effec[1],2))

    # Declaramos el vector desde la articulación actual al punto objetivo.
    actual_a_objet = [0] * 2

    actual_a_objet[0] = objetivo[0] - pos_artic_actual[0]
    actual_a_objet[1] = objetivo[1] - pos_artic_actual[1]
    actual_a_objet_mag = sqrt(pow(actual_a_objet[0],2) + pow(actual_a_objet[1],2))

    #effector_a_objet_mag = actual_a_effec_mag * actual_a_objet_mag

    actual_a_effec = [element / actual_a_effec_mag for element in actual_a_effec]
    actual_a_objet = [element / actual_a_objet_mag for element in actual_a_objet]

    cos_theta = np.dot(actual_a_effec, actual_a_objet)
    direction_theta = np.cross(actual_a_effec, actual_a_objet)

    theta = acos(cos_theta)

    if(direction_theta < 0.0):
        theta = -theta

    print("Theta: " + str(theta))

    th[j] += theta
    O[i+1] = cin_dir(th,a)
    j -= 1


  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print "\n- Iteracion " + str(iteracion) + ':'
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print "Distancia al objetivo = " + str(round(dist,5))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print "\n" + str(iteracion) + " iteraciones para converger."
else:
  print "\nNo hay convergencia tras " + str(iteracion) + " iteraciones."
print "- Umbral de convergencia epsilon: " + str(EPSILON)
print "- Distancia al objetivo:          " + str(round(dist,5))
print "- Valores finales de las articulaciones:"
for i in range(len(th)):
  print "  theta" + str(i+1) + " = " + str(round(th[i],3))
for i in range(len(th)):
  print "  L" + str(i+1) + "     = " + str(round(a[i],3))
