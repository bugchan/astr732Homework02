#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 21:17:35 2019

@author: Sandra Bustamante
"""
import numpy as np
import matplotlib.pyplot as plt
from astropy.io import fits
from astropy.wcs import WCS

#w = WCS(naxis=2)



hdulist = fits.open('/home/sandra/ASTRO732/signal_wfilt_sn.fits')
hdu=hdulist['PRIMARY']

#used to see if coordinates are in sky coordinates
#hdu=hdulist['PRIMARY']
vmin=hdu.data.min()
vmax=hdu.data.max()
wcs = WCS(hdu.header)

#number of pixels equivalent to 20 arcminutes
nPix=20*60/3
#determine the position of the central pixel
nTotalPixX=hdu.header['NAXIS1']
nTotalPixY=hdu.header['NAXIS2']
centerPixX=nTotalPixX/2
centerPixY=nTotalPixY/2

#create a circular mask
x=np.arange(0,nTotalPixX,1)-centerPixX
y=np.arange(0,nTotalPixY,1)-centerPixY
X,Y=np.meshgrid(x,y)
interior = np.sqrt(X**2 + Y**2) > nPix

#Info for use later when plotting
intx,inty=np.where(interior==False)

#creates a masked array
data = np.ma.array(hdu.data) 
#applies mask to data
data[interior] = np.ma.masked


#initialize plot
#width=3.39 in for single column, 6.9 for double
#height=2.41 in for single column, 4.26 for double 
fig1 = plt.figure(figsize=(6.9,4.26))
ax1 = fig1.add_subplot(111, projection=wcs)

#plot data
dataPlotted=ax1.imshow(data,aspect='equal')

#adds circles around sources with SN>4.5
ysources,xsources=np.where(data>4.5) #position in pixels

ax1.scatter(xsources,ysources,
            s=100,
            edgecolor='white',
            facecolor='none')

#customize plot
fig1.colorbar(dataPlotted, label='$ S / N $')
ax1.set(
       xlim=[intx.min()-10,intx.max()+10], #10 just to leave white space
       xlabel='RA',
       ylim=[inty.min()-10,inty.max()+10],
       ylabel='DEC',
       title='COSMOS field'    
       )

fig1.tight_layout()
plt.savefig('hw01prob1-3fig1.png', dpi=600)