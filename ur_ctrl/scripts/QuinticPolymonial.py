#! usr/bin/python
# filename: QuinticPolymonial.py

import numpy
from numpy.linalg import *
from pychart import *

T = 10
BC0 = numpy.array([[0], [0], [0]])
BC1 = numpy.array([[100], [0], [0]])
Tm01 = numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 2]])
#C0 = Tm01.I * BC0
C0 = numpy.dot(inv(Tm01), BC0)

Tm11 = numpy.array([[1, T, T*T], [0, 1, 2*T], [0, 0, 2]]);
Tm12 = [[T*T*T, T*T*T*T, T*T*T*T*T], [3*T*T, 4*T*T*T, 5*T*T*T*T], [6*T, 12*T*T, 20*T*T*T]]
C1 = numpy.dot(inv(Tm12), (BC1 - numpy.dot(Tm11, C0))) #Tm12^-1 * (BC1 - Tm11 * C0)

time_step = 0.1

tm = numpy.empty([6, T/time_step])
for index1 in range(tm.shape[1]):
	for index0 in range(tm.shape[0]):
		tm[index0, index1] = numpy.power(time_step * index1, index0)

C = numpy.array([[C0[0,0], C0[1,0], C0[2,0], C1[0,0], C1[1,0], C1[2,0]], 
				 [C0[1,0], 2*C0[2,0], 3*C1[0,0], 4*C1[1,0], 5*C1[2,0], 0],
				 [2*C0[2,0], 6*C1[0,0], 12*C1[1,0], 20*C1[2,0], 0, 0]])
QP = numpy.dot(C, tm)


#Draw the plots of QP[0,:]
theme.get_options()
data = []

for i in range(QP.shape[1]):
	data.append((i, QP[0,i]))

xaxis = axis.X(format = '/hL%d', label = 'time/s')
yaxis = axis.Y(label = 's/m')

ar =area.T(x_axis=xaxis, y_axis=yaxis, y_range=(0, None), size=(120,110))
plot = line_plot.T(label ='sss', data = data, ycol = 1, tick_mark=tick_mark.star)
ar.add_plot(plot)

ar.draw()
