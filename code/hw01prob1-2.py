#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 16:51:48 2019

@author: Sandra Bustamante
"""

import numpy as np
import matplotlib.pyplot as plt
from astropy.io import fits


hdulist = fits.open('/home/sandra/ASTRO732/cosmos_noise_example.fits')
hdu=hdulist['PRIMARY']

#number of pixels equivalent to 20 arcminutes
nPix=20*60/3
#determine the position of the central pixel
nTotalPixX=hdu.header['NAXIS1']
nTotalPixY=hdu.header['NAXIS2']
centerPixX=nTotalPixX/2
centerPixY=nTotalPixY/2


#create x and y values where the center is zero
x=np.arange(0,nTotalPixX,1)-centerPixX 
y=np.arange(0,nTotalPixY,1)-centerPixY
X,Y=np.meshgrid(x,y)
#create a circular mask of pixel equivalent to 20 arcminutes
interior = np.sqrt(X**2 + Y**2) > nPix

#Info for use later when plotting
intx,inty=np.where(interior==False)

#creates a masked array
data = np.ma.array(hdu.data) 
#applies mask to data
data[interior] = np.ma.masked
#update data so the histogram doesn't count all of the zeros on the borders
data=data.data[intx.min():intx.max(),inty.min():inty.max()]


#initialize plot
#width=3.39 in for single column, 6.9 for double
#height=2.41 in for single column, 4.26 for double
figWidth=6.9
figHeight=4.26
col=1
row=2

fig1 = plt.figure(figsize=(figWidth*col,
                           figHeight*row)) 
ax1 = fig1.add_subplot(211)
histData=ax1.hist(data.flatten(), bins=100)
ax1.set(
       xlim=[data.min()*1.1,data.max()*1.1],
       xlabel='Jy / beam',
       ylim=[0,histData[0].max()*1.1],
       ylabel='Counts',
       title='Linear Form'    
       )

ax2 = fig1.add_subplot(212)
histLogData=ax2.hist(data.flatten(), bins=100)
ax2.set(
       xlim=[data.min()*1.1,data.max()*1.1],
       xlabel='Jy/beam',
       ylabel=r'$log_{10} \left( Counts \right)$',
       yscale='log',
       title='SemiLog Form'    
       )
fig1.tight_layout()
plt.savefig('hw01prob1-2fig1.png', dpi=600)
plt.show()
