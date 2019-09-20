#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Sep  8 20:49:07 2019

@author: root
"""

import numpy as np
#import matplotlib.pyplot as plt

def func(n,eps,x):
    one=valueType(x,1)
    oneE16=valueType(x,1e16)
    
    if n=='a':
       return (one + eps > one)
    if n=='b':
       return (one - eps < one)
    if n=='c':
       return (oneE16 + eps > oneE16)
    if n=='d':
       return (oneE16 - eps < oneE16)

def valueType(x,value):
    if x == 32:
        return np.float32(value)
    if x == 64:
        return np.float64(value)


exercise='d'
x=64
h=valueType(x,1e16)#for problems a & b, h=1, c&d h=1e16

eps0=valueType(x,0)
new_eps=eps0
old_eps=eps0
repeat=0
continueWhile=True
i=0

verbose=False
debug=False



while continueWhile:
    i+=1
    old_h=h
    if i==1:
        new_state=func(exercise,new_eps,x)
        new_eps=valueType(x,(old_eps+h))
        if debug:
            print('%s Function is %s'%(i, new_state))
    else:
        prev_state=new_state
        new_state=func(exercise,new_eps,x)
        if new_state:
            min_eps=new_eps
            min_oldeps=old_eps
            iTrue=i-1
        if debug:
            print('%s Function is %s'%(i, new_state))
        if new_state!=prev_state:
            h=valueType(x,((old_eps-new_eps)/2))
            if debug:
                print('%s changing h to %s'%(i,h))
        old_eps=valueType(x,(new_eps))
        new_eps=valueType(x,(old_eps+h))
    if verbose:
        print('%s new_eps=%s, old_eps=%s, h=%s'%(i,new_eps,old_eps,h))
    if new_eps == old_eps:
        if old_h== h:
            repeat+=1
        if repeat>3:
            continueWhile=False
            print("""The minimum value of epsilon using float%s so that problem 1.4.%s was True, is between %s and %s. It was obtained in iteration %s."""% (x,exercise,min_eps,min_oldeps,iTrue))
            
            
