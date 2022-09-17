import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import json
import math

a1, a2, a3, a4=np.loadtxt("/home/jonathan/dev_ws/src/parallel_robot/results/Posiciones.txt",unpack=True)
r1, r2, r3, r4=np.loadtxt("/home/jonathan/dev_ws/src/parallel_robot/results/referencias.txt",unpack=True)
t1, t2, t3, t4=np.loadtxt("/home/jonathan/dev_ws/src/parallel_robot/results/Fuerzas.txt",unpack=True)

#posiciones=[a2,a3,a4,a1]
#referencias=[r1,r2,r3,r4]

muestras=np.size(a1)
tiempo = np.linspace(0, muestras*0.01, muestras)

fig= plt.figure(figsize=(15,15))
fig.tight_layout()
#colores = ['blue','green','red','cyan','magenta','yellow','black','white']

#for i in range(1,5):
ax=plt.subplot(2,2,1) 
ax.plot(tiempo,a1,'b')
ax.plot(tiempo,r1,'r')
#ax.plot(tiempo,t1,'y')
ax.set_xlabel('Posicion lineal prismatica q')
ax.set_ylabel('Tiempo')
ax.set_title('Posicion q vs t: Actuador 1')

ax=plt.subplot(2,2,2) 
ax.plot(tiempo,a2,'y')
ax.plot(tiempo,r2,'r')
#ax.plot(tiempo,t2,'g')
ax.set_xlabel('Posicion lineal prismatica q')
ax.set_ylabel('Tiempo')
ax.set_title('Posicion q vs t: Actuador 2')


ax=plt.subplot(2,2,3) 
ax.plot(tiempo,a3,'g')
ax.plot(tiempo,r3,'r')
ax.set_xlabel('Posicion lineal prismatica q')
ax.set_ylabel('Tiempo')
ax.set_title('Posicion q vs t: Actuador 3')

ax=plt.subplot(2,2,4) 
ax.plot(tiempo,a4,'m')
ax.plot(tiempo,r4,'r')
ax.set_xlabel('Posicion lineal prismatica q')
ax.set_ylabel('Tiempo')
ax.set_title('Posicion q vs t: Actuador 4')
plt.show()











