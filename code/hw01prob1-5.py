#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Sep  8 01:28:06 2019

@author: Sandra Bustamante
"""

import numpy as np
import matplotlib.pyplot as plt

def nextPhi(n,aPhi):
    """
    n is used to determine the actual phi
    aPhi is the array of Phi
    """
    lastPhi=aPhi[n-1]
    actualPhi=aPhi[n]
    nextPhi=lastPhi-actualPhi
    aPhi[n+1]=nextPhi
    return aPhi   

GoldenMean32=np.float32((np.sqrt(5)-1)/2)
GoldenMean64=np.float64((np.sqrt(5)-1)/2)

nMax=30

aPhi32=np.zeros(nMax,dtype=np.float32)
aPhi64=np.zeros(nMax,dtype=np.float64)

#Initialize arrays since we know that when n=0, phi_N=1 
#and if n=1, phi_n=GoldenMean
aPhi32[0]=np.float32(1)
aPhi32[1]=GoldenMean32

aPhi64[0]=np.float64(1)
aPhi64[1]=GoldenMean64

for i in range(1,nMax-1):
    
    aPhi32=nextPhi(i,aPhi32)
    aPhi64=nextPhi(i,aPhi64)

#initialize plot
#width=3.39 in for double columns, 6.9 for single
#height=2.41 in for double columns, 4.26 for single
figWidth=6.9
figHeight=4.26
col=1
row=2

fig1 = plt.figure(figsize=(figWidth*col,
                           figHeight*row))

ax1 = fig1.add_subplot(211)
ax1.plot(aPhi32,'-o', label='Float32')
ax1.plot(aPhi64,'--',label='Float64')
ax1.set(
       xlim=[-1,31],
       xlabel='n',
       ylim=[aPhi32.min()*30,1.1],
       ylabel=r'$\phi^n$',
       title='Unstability of the integer powers of the Golden Mean'    
       )
ax1.legend()

ax2 = fig1.add_subplot(212)
x=np.arange(10,nMax)
ax2.plot(x,aPhi32[10:],'-o', label='Float32')
ax2.plot(x,aPhi64[10:],'--',label='Float64')
ax2.set(
       xlim=[-1,31],
       xlabel='n',
       ylim=[aPhi32.min()*1.3,aPhi32[10:].max()*1.1],
       ylabel=r'$\phi^n$'    
       )
ax2.legend()

fig1.tight_layout()
plt.savefig('hw01prob1-5fig1.png', dpi=600)
plt.show()