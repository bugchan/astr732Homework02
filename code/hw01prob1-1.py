#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 11:15:22 2019
@author: Sandra Bustamante
Homework 1 Due September 10am at 9:00am

"""

import numpy as np
import matplotlib.pyplot as plt

#Problem 1.1 Reproduce plot of blackbody radiation
def cons (x,SI= True):
    """ 
    Returns the value of the constant you want.
        string x is the constant whose value you want.
        bool SI is either true or false.
    Values are taken from 2018 CODATA NIST SP959 June 2019
    """    
    if SI:
        if x=='h': return 6.62607015e-34     #J Hz^-1
        if x=='hbar': return 1.054571817e-34 #J s
        if x=='c': return 299792458                 # m s^-1
        if x=='k': return 1.380649e-23       #J K^-1
    
    else : print ('update function')   
        
    
def plancksLaw(nu,T):
    """
    Calculates the blackbody radiation given the frequency 
    and temperature.
    nu is the frequency in Hz
    T is the temperature of the blackbody object in Kelvin
    """
    #define constants
    h=cons('h')
    c=cons('c')
    k=cons('k')
    #Calculate blackbody radiation
    A = (2*h* nu**3)/(c**2)
    B = np.exp((h*nu) / (k*T)) - 1
    Bnu = A / B                     # W m^-2 sr^-1 Hz^-1
    return Bnu

def wiensDisplacementLaw(T,mode='freq'):
    if mode=='freq':
        nu = 5.879e10 * T #Hz
        return nu
    if mode == 'wavelength':
        Lambda = 2.89777195e-3 / T
        return Lambda
    
#define constants or known variables
points=60
freqsStart=9 # 1e9 Hz
freqsEnd=13 # 1e13 Hz
freqsBase=10
Temp1=20 #kelvin
Temp2=10 #kelvin


#calculate the blackbody radiaton
freqs=np.logspace(freqsStart,freqsEnd,base=freqsBase,num=points)
Bnu1=plancksLaw(freqs,Temp1)
Bnu2=plancksLaw(freqs,Temp2)
freqMax1=wiensDisplacementLaw(Temp1)
freqMax2=wiensDisplacementLaw(Temp2)

#create plot
#width=3.39 in for single column, 6.9 for double
#height=2.41 in for single column, 4.26 for double  
fig1 = plt.figure(figsize=(6.9,4.26))
ax = fig1.add_subplot(111)

#plot Blackbody Radiation
ax.loglog(freqs,Bnu1,'-',label='20K',color='black')
ax.loglog(freqs,Bnu2,'-',label='10K',color='red')

#plot vertical lines at the peak frequency
ax.axvline(x=freqMax1,
           ls='--',
           label='Max of 20K Blackbody',
           color= 'black',
           linewidth=2
           )
ax.axvline(x=freqMax2,
           ls='--',
           label='Max of 10K Blackbody',
           color='red',
           linewidth=2)

#add labels 
ax.text(15e9,6e-17,'20K Blackbody',
        color='black',
        fontsize=12,
        fontweight='bold')
ax.text(30e9,1e-18,'10K Blackbody',
        color='red',
        fontsize=12,
        fontweight='bold')

#customize plot
ax.set_title('Blackbody Radiation',
             fontsize=12,
             fontweight='bold')

ax.set_xlabel('Frequency [GHz]',fontsize=12)
ax.set_xticklabels(('','10','100','1000'),fontsize=12)
ax.set_xlim(1e10,1e13)

ax.set_ylabel(r'$B_\nu \left( T \right)$',fontsize=12)
ax.set_ylim(1e-19,1e-14)


fig1.tight_layout()
plt.savefig('hw01prob1-1.png', dpi=600)
plt.show()
