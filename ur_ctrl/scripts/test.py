#!/usr/bin/python

import random
import sys

# Not using the cgi module at the moment.
#import cgi
#import cgitb; cgitb.enable()

from pychart import *

print "Content-type: image/png"
print

sys.argv.append( '--format=png' )

theme.get_options()
theme.scale_factor = 2
theme.reinitialize()

data = []
for i in range(10):
    data.append( (i, random.random()* 3.0) )

xaxis = axis.X( format="/hL%d",  label="time" )
yaxis = axis.Y(  label="synaptic activity" )

ar = area.T( x_axis=xaxis, y_axis=yaxis, y_range=(0,None), size=(120,110) )
plot = line_plot.T( label="cortex data", data=data, ycol=1, tick_mark=tick_mark.square )
ar.add_plot( plot )

ar.draw()
