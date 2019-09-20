#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 10:11:45 2019
@author: Sandra Bustamante

The Monkey and The Hunter

Uses Runge Kutta 4 order to solve for the velocities and positions of a bullet and a monkey in a system.
3 conditions of shooting:
    1) Only gravity is acting on bullet.
    2) Gravity and air resistance are acting on bullet.
    3) Gravity, air and random wind are acting on bullet. 
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from scipy import linalg as la
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

def cons (x,SI= True):
    """ 
    Returns the value of the constant you want.
        string x is the constant whose value you want.
        bool SI is either true or false.
    Values are taken from 2018 CODATA NIST SP959 June 2019
    """    
    if SI:
        if x=='h': return 6.62607015e-34    #J Hz^-1
        if x=='hbar': return 1.054571817e-34#J s
        if x=='c': return 299792458         #m s^-1
        if x=='k': return 1.380649e-23      #J K^-1
        if x=='g': return 9.80665           #m s-2
    else : print ('update function')   
    
def gravityArray():
    ''' Creates the aceleration array for both the monkey 
    and the bullet. 
    '''
    g=-cons('g') #m/s^2
    a=np.array([0,g])
    return a

def Fgravity(t,u,v,vWind):
    #mbullet=0.007 #kg
    a=gravityArray()
    dvdt=a
    return dvdt

def Fair(t,u,v,vWind):
    c=0.04
    rho=1.204 #kg/m^3
    r=7.2e-3 # m (7.2mm)
    # Cross section of sphere is pi*(2r)^2 / 4
    A=np.pi*r**2
    k1=-(c*rho*A)/2.
    k2=v/np.sqrt((v**2).sum())
    dvdt=k1 * v**2 *k2
    return dvdt

def FwithAir(t,u,v,vWind):
    dvdt=Fgravity(t,u,v,vWind)+Fair(t,u,v,vWind)
    return dvdt

def FwithWind(t,u,v,vWind):
    c=0.04
    rho=1.204 #kg/m^3
    r=7.2e-3 # m (7.2mm)
    # Cross section of sphere is pi*(2r)^2 / 4
    A=np.pi*r**2
    dvdt=Fgravity(t,u,v,vWind)+Fair(t,u,v,vWind) + (c*rho*A*v_wind**2)/2 
    return dvdt

def calcVWind():
    """ 
    Random oriented wind with random speed. 
    User have the option to fire again with same wind conditions.
    Speed is uniform distribution on range 0 - 20 m/s.
    Direction random distribution on the uniform range [0,pi] where O is hunter to monkey, 
        pi/2 is vertical pointing to the ground, and pi is monkey to hunter.
    Parameters: None
    Returns: v = (vx,vy)
    """
    speed=np.random.uniform(0,20)
    #Making my range on the direction go from 0 to -np.pi so 
    #that the angle of direction can be consistent with angle of the shooting. 
    direction=np.random.uniform(0,-np.pi) #radians
    v=np.array([speed*np.cos(direction),
                speed*np.sin(direction)])
    return v,speed,direction    



def rk4(func,told,uold,vold,vWind,h):
    midt=told+h/2
    tnew=told+h
    k1v=h*func(told,uold,vold,vWind)
    k1u=h*vold
    k2v=h*func(midt,uold,vold+k1v/2,vWind)
    k2u=h*(vold+k1v/2)
    k3v=h*func(midt,uold,vold+k2v/2,vWind)
    k3u=h*(vold+k2v/2)
    k4v=h*func(tnew,uold,vold+k3v,vWind)
    k4u=h*(vold+k3v)
    vnew=vold+k1v/6+k2v/3+k3v/3+k4v/6
    unew=uold+k1u/6+k2u/3+k3u/3+k4u/6
    return tnew,unew,vnew


#User inputs
#=============================================================
print(''' A monkey has been stealing your bananas everyday for a month now. 
      Suddenly, when you are leaving your house to golf practice, you see 
      something moving on a tree. It's HIM. On closer inspection, he looks like 
      he is waiting for you to leave your house and steal your fresh bananas.
      'No way!' you think and decide to scare the monkey. You prepare to shoot
      him with your golf ball. You estimate he is at about 2km away at a 
      height of, probably, 60m. ''')
time.sleep(3)

theta=np.float(input('At what angle (deg) would you aim to shoot the monkey?'))
theta=theta*np.pi/180. #rad

#Default mode is gravity only
airDrag=input('Do you want to add air resistance? [y,n] ')
if airDrag == 'y':
    airDrag = True
else:
    airDrag = False

wind=input('Do you want to include wind? [y,n] ')
if wind == 'y':
    wind = True
else:
    wind = False

print('Prepare...')
time.sleep(.5)
print('Set...')
time.sleep(.5)
print('SHOOT!')


#General variables needed in the code
#=============================================================
h=.0001 #s, step size in time
tf=int(2/h) #trial and error. The outmost it will take to arrive to the other side.
seeReal=True #Shows plot of real values
debug=True #shows last 3 items of the final x array from bullet and monkey
doPlot=True

#initial conditions for bullet
v0bullet=1100  #m/s

vBullet=np.zeros((tf,2),dtype=np.float32)
vBullet[0]=np.array([v0bullet*np.cos(theta),v0bullet*np.sin(theta)])

xBullet=np.zeros((tf,2),dtype=np.float32)
xBullet[0]=np.array([0.,0.])
xBullet_new=np.array([0,0])


#initial conditions for monkey
v0monkey=0. #m/s

vMonkey=np.zeros((tf,2),dtype=np.float32)
vMonkey[0]=np.array([0.,v0monkey])

if debug:
    x0=2000.
    y0=60.
else:
    x0random=np.random.uniform(0,100)
    x0=2000. + x0random
    y0random=np.random.uniform(0,20)
    y0=60. + y0random

xMonkey=np.zeros((tf,2),dtype=np.float32)
xMonkey[0]=np.array([x0,y0])
xMonkey_new=np.array([0,0])

#Initialize time array
t=np.zeros(tf,dtype=np.float32)
t[0]=0.

#clear number of iterations
n=0

#choose which func are going to be used
if airDrag==False and wind==False:
    FUNC=Fgravity
    v_wind=0
    if debug:
        print('The only force acting on bullet is gravity')
if airDrag==True and wind==False:
    FUNC=FwithAir
    v_wind=0
    if debug:
        print('The forcces acting on bullet is gravity and air resistance')
if airDrag==True and wind==True:
    FUNC=FwithWind
    v_wind,speed,direction=calcVWind()
    direction=direction*180/np.pi #converts it into degrees
    if debug:
        print('The forces acting on bullet is gravity, air resistance and wind.')
    print('The wind is blowing at %s m/s with direction %s deg'%(speed,direction))

#solving the equations of motion of monkey and bullet
#while loop will continue until x position of the bullet is bigger than 
#the x position of the monkey.
while xBullet_new[0]<=(xMonkey[0][0]+.1):
    n+=1
    #RK4 for Bullet
    t_new,xBullet_new,vBullet_new=rk4(FUNC,
                                      t[n-1],
                                      xBullet[n-1],
                                      vBullet[n-1],
                                      v_wind,
                                      h)
    t[n]=t_new
    vBullet[n]=vBullet_new
    xBullet[n]=xBullet_new

    #Rk4 for Monkey who is only affected by gravity
    t_newM,xMonkey_new,vMonkey_new=rk4(Fgravity,
                                      t[n-1],
                                      xMonkey[n-1],
                                      vMonkey[n-1],
                                      v_wind,
                                      h)
    t[n]=t_new
    vMonkey[n]=vMonkey_new
    xMonkey[n]=xMonkey_new


if debug:
    print ('Bullet')
    print (xBullet[n-2:n+1])
    print ('Monkey')
    print (xMonkey[n-2:n+1])



#Plot only some points of the position array
#just plot points number of points to decrease computing time
points=10
xBulletPlot=xBullet[0:n+1:int(n/points)]
xMonkeyPlot=xMonkey[0:n+1:int(n/points)]
tPlot=t[0:n+1:int(n/points)]
    
if doPlot:
    fig1 = plt.figure(figsize=(6.9,4.9))
    bg = plt.imread('background.jpg')
    monkey = plt.imread('monkey.png')
    
    if seeReal:
        ax1 = fig1.add_subplot(111)
    else:
        ax1 = fig1.add_subplot(111)
        
    ax1.imshow(bg, extent=[0,2100,0,80], aspect='auto')
#    ax1.imshow(monkey, extent=[xMonkey[n,0]-50,
#                               xMonkey[n,0]+50,
#                               xMonkey[n,1]-50,
#                               xMonkey[n,1]+50],aspect='equal')
    ax1.plot(xBulletPlot[:,0],xBulletPlot[:,1],'-o')
    ax1.plot(xMonkeyPlot[:,0],xMonkeyPlot[:,1],'-*')
    ax1.set_xlim(0,2100)
    ax1.set_ylim(0,80)
    ax1.grid()


##Find the position using the formula: v=at+v0; x=x0+v0t+at^2/2 using the
# the time array generated from RK4. 
# I create new arrays so it won't overwrite the previous ones

if seeReal:
    a=gravityArray()
    #v=np.zeros((tPlot.shape[0],2),dtype=np.float32)
    v0=vBullet[0]
    x0=xBullet[0]
    x=np.zeros((tPlot.shape[0],2),dtype=np.float32)

    for i in np.arange(1,tPlot.shape[0]):
        #v[i]=a*tPlot[i]+v[0]
        x[i]=v0*tPlot[i]+x0+(1/2.)*a*tPlot[i]**2
        
    #calculate x,y at last t
    xfinalBullet=xBullet[0]+vBullet[0]*t[n]+(1/2.)*a*t[n]**2
    xfinalMonkey=xMonkey[0]+vMonkey[0]*t[n]+(1/2.)*a*t[n]**2
    
    #create plot
    if doPlot:
        #ax2 = fig1.add_subplot(122)
        ax1.plot(x[:,0],x[:,1],'-o')
        ax1.set_xlim(0,2100)
        ax1.set_ylim(0,80)
        ax1.grid()
    
    #calculates residuals
    if debug:
        res=xBullet[0:n+1:int(n/points)]-x
        print ('Residuals')
        print (res)
if doPlot:
    fig1.tight_layout()
    plt.show()

#Determining if it was a Hit or a Missed 
#use only if step size is smaller than .0001
majorSizeMonkey=xMonkey[n-1]+.1
minorSizeMonkey=xMonkey[n-1]-.1

if (xBullet[n-1]<majorSizeMonkey).all() and (xBullet[n-1]>minorSizeMonkey).all() :
    print ('Hit the Monkey')
else:
    print ('Missed the Monkey')

##do svd to fit a parabola to find use last five values
#a0=np.ones((5,1))
#a1,y=np.split(xBullet[n-3:n+1],2,axis=1)
#a2=a1**2
#A=np.concatenate((a0,a1,a2),1)
#Ainv=la.pinv(A)
#coeff=np.dot(Ainv,y)
#
#ybullet=coeff[0]+coeff[1]*xMonkey[0,0]+coeff[2]*xMonkey[0,0]**2


