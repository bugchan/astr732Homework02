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
#from scipy i.mport linalg as la
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
    m=.007 #kg
    k1=-(c*rho*A)/(2.*m)
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

def rk4loop(FUNC,t,xBullet,vBullet,vWind,xMonkey,vMonkey,h,debug=False):
    ''' Solves the equations of motion of monkey and bullet while x position of the bullet is smaller than the x position of the monkey.
    '''
    xBullet_new=np.array([0,0])
    #clear number of iterations
    n=0
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
        vMonkey[n]=vMonkey_new
        xMonkey[n]=xMonkey_new
        if debug and n==20000-1:
            print ('Last xBullet', xBullet[n-1])
            print ('Last time', t[n-1])
                
    return xBullet, xMonkey, n

def doPlot(mode,xBullet,xMonkey,xCompare,t,tCompare,debug=False):
    if mode == 'story':
        fig1 = plt.figure(figsize=(6.9,4.9))
        ax1 = fig1.add_subplot(111)
        #Set background image
        bg = plt.imread('background.jpg')
        ax1.imshow(bg, extent=[0,2100,0,80], aspect='auto')
        #Set monkey on last point
        monkey = plt.imread('monkey.png')
        monkeyim = OffsetImage(monkey, zoom=.2)
        aMonkey = AnnotationBbox(monkeyim, xMonkey[-1], xycoords='data', frameon=False)
        ax1.add_artist(aMonkey)
        #Set golfball on last point
        golfball = plt.imread('golfball.png')
        golfballim = OffsetImage(golfball, zoom=.3)
        aGolfball = AnnotationBbox(golfballim, xBullet[-1], xycoords='data', frameon=False)
        ax1.add_artist(aGolfball)
        #Plot trajectory monkey and golfball
        ax1.plot(xBullet[:,0],xBullet[:,1],'-o')
        ax1.plot(xMonkey[:,0],xMonkey[:,1],'-*')
        ax1.set_xlim(0,2100)
        ax1.set_xlabel('Distance [m]')
        ax1.set_ylim(0,80)
        ax1.set_ylabel('Height [m]')
        ax1.grid()
        
        ax2=[]
        res=[]
        
    elif mode == 'gravity':
        #do plot gravity vs real
        fig1 = plt.figure(figsize=(6.9,4.9))
        grid = plt.GridSpec(3,1, wspace=0.4, hspace=0.3)
        ax1 = fig1.add_subplot(grid[:2,0])
        ax1.plot(xBullet[:,0],xBullet[:,1],'-', label='Gravity Only')
        ax1.plot(xCompare[:,0], xCompare[:,1],'-', label='Gravity and Air Resistance')
        ax1.plot(xMonkey[:,0],xMonkey[:,1],'-*')
        ax1.set_xlim(0,2100)
        ax1.set_xlabel('Distance [m]')
        ax1.set_ylim(0,80)
        ax1.set_ylabel('Height [m]')
        ax1.grid()
        ax1.legend()

        ax2=fig1.add_subplot(grid[2,0])
        res=xPlot-xBullet
        if debug:
            print ('Residuals Shape', res.shape)
            print ('time shape: ',t.shape)
        ax2.plot(t,res[:,0],'o',label='Residuals in x direction')
        ax2.plot(t,res[:,1],'o',label='Residuals in y direction')
        ax2.set_xlim(0,t.max()*1.1)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylim(res.min()*1.1,res.max()*1.1)
        ax2.set_ylabel('Residuals [m]')
        ax2.grid()
        ax2.legend()
        
        
    elif mode == 'airdrag':
        #do plot air drag, gravity and real
        fig1 = plt.figure(figsize=(6.9,7.2))
        grid = plt.GridSpec(3,1, wspace=0.4, hspace=0.4)
        ax1 = fig1.add_subplot(grid[0,0])
        ax1.plot(xBullet[:,0],xBullet[:,1],'-o', label='Gravity + Air')
        ax1.plot(xCompare[:11,0], xCompare[:11,1],'-o', label='Fgravity')
        ax1.plot(xMonkey[:,0],xMonkey[:,1],'-*')
        ax1.set_xlim(0,2100)
        ax1.set_xlabel('Distance [m]')
        ax1.set_ylim(0,xBullet[:,1].max()*1.2)
        ax1.set_ylabel('Height [m]')
        ax1.grid()
        ax1.legend()

        ax2=fig1.add_subplot(grid[1,0])
        ax2.plot(t,xBullet[:,1],'-o',label='With Air')
        ax2.plot(tCompare[:11],xCompare[:11,1],'-o',label='Gravity only')
        #ax2.set_xlim(0,t.max()*1.1)
        #ax2.set_xlabel('Time [s]')
        ax2.set_ylim(0,xBullet[:,1].max()*1.2)
        ax2.set_ylabel('Y Position [m]')
        ax2.grid()
        ax2.legend()
        
        ax3=fig1.add_subplot(grid[2,0],sharex=ax2)
        ax3.plot(t,xBullet[:,0],'-o',label='With Air')
        ax3.plot(tCompare[:11],xCompare[:11,0],'-o',label='Gravity only')
        ax3.set_xlim(0,t.max()*1.1)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylim(0,xBullet[:,0].max()*1.1)
        ax3.set_ylabel('X Position [m]')
        ax3.grid()
        ax3.legend()
        
    elif mode == 'wind':
        #do plot wind vs airdrag vs gravity vs real
        fig1 = plt.figure(figsize=(6.9,4.9))
        ax1 = fig1.add_subplot(111)
        ax1.plot(xBulletPlot[:,0],xBulletPlot[:,1],'-o')
        ax1.plot(xMonkeyPlot[:,0],xMonkeyPlot[:,1],'-*')
        ax1.set_xlim(0,2100)
        ax1.set_ylim(0,80)
        ax1.grid()
        
    else:
        print("There's no plot for you (u.u)")
    return fig1,ax1,ax2

def calcReal(points,xBulletPlot,tPlot,v0):     
    a=gravityArray()
    x0=xBullet[0]
    xPlot=np.zeros((tPlot.shape[0],2),dtype=np.float32)
    
    for i in np.arange(0,tPlot.shape[0]):
        xPlot[i]=v0*tPlot[i]+x0+(1/2.)*a*tPlot[i]**2
    
    return  xPlot

def hitMiss(h,xMonkey,xBullet):
        
    if h<=.0001:
        majorSizeMonkey=xMonkey[n-1]+.1
        minorSizeMonkey=xMonkey[n-1]-.1
        
        if (xBullet[n-1]<=majorSizeMonkey).all() and (xBullet[n-1]>minorSizeMonkey).all() :
            print ('Hit the Monkey')
            Miss=False
        else:
            print ('Missed the Monkey')
            Miss=True        
    if Miss:
        repeat = input('Want to try again? [y,n] ')
        if repeat == 'y':
            repeat=True
            x0Monkey = xMonkey[0]
        elif repeat == 'n':
            repeat=False
        else:
            print('not one of the option but still will end game. Thanks for playing.')
            time.sleep(2)
            repeat=False
    else:
        print('You scare the monkey away. Thanks for playing. Bye')
        repeat=False
    return repeat, x0Monkey

def initialize(x0bullet,v0bullet,x0monkey,v0monkey,tf):    
    #Initializing arrays
    vBullet=np.zeros((tf,2),dtype=np.float32)
    vBullet[0]=np.array([v0bullet*np.cos(theta),v0bullet*np.sin(theta)])
    
    xBullet=np.zeros((tf,2),dtype=np.float32)
    xBullet[0]=x0bullet
    
    vMonkey=np.zeros((tf,2),dtype=np.float32)
    vMonkey[0]=v0monkey
    
    xMonkey=np.zeros((tf,2),dtype=np.float32)
    xMonkey[0]=x0monkey
    
    t=np.zeros(tf,dtype=np.float32)
    t[0]=0.
    
    return xBullet,vBullet,xMonkey,vMonkey,t

#Programmer Interaction
#=============================================================
mode='airdrag' #'story', 'gravity', 'airdrag', 'wind'
showPlot=True
h=.0001 #s, step size in time
tf=int(4/h) #trial and error. The outmost it will take to arrive to the other side.
debug=True #Prints several values along the code to help in debugging

#=============================================================
if mode == 'story':
    print('A monkey has been stealing your bananas everyday for a month now. \n')
    time.sleep(1)
    print('As you are leaving your house to golf practice, you notice something moving on a tree. \n')
    time.sleep(1)
    print("It's HIM! On closer inspection, he looks like he is waiting for you to leave your house and steal your fresh bananas. \n")
    time.sleep(1)
    print('No way! Suddenly, you have an idea: you can scare him away.\n')
    time.sleep(1)
    print('You prepare to shoot him with your golf ball. You estimate he is at about 2km away and at a height of, probably, 60m. \n')
    time.sleep(1)
    
#User inputs
#=============================================================
    airDrag=input('Is there air resistance? [y,n] ')
    if airDrag == 'y':
        airDrag = True
        wind=input('Is there wind? [y,n] ')
        if wind == 'y':
            wind = True
        else:
            wind = False
    else:
        airDrag = False
        wind = False
    
    ####################################################################
    #Selecting the differential equation to solve by RK4
    ####################################################################
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
        print('Oh no! The wind is blowing at %s m/s with direction %s deg /n'%(speed,direction))

    ####################################################################
    #Initializes the vectors with initial positions
    ####################################################################
    
    v0bullet=1100  #m/s
    x0bullet=np.array([0.,0.])
    v0monkey=np.array([0.,0.]) #m/s
    x0random=np.random.uniform(0,100)
    x0=2000. + x0random
    y0random=np.random.uniform(0,20)
    y0=60. + y0random
    x0monkey=np.array([x0,y0])
    
    repeat=True
    v_wind=0
    
    while repeat:
        theta = np.float32(input('Select your shooting angle in degrees. '))
        theta = theta*np.pi/180.
    
        print('Prepare...')
        time.sleep(.5)
        print('Set...')
        time.sleep(.5)
        print('SHOOT! \n')
    
        xBullet,vBullet,xMonkey,vMonkey,t=initialize(x0bullet,v0bullet,x0monkey,v0monkey,tf)
        
        ################################################################
        #Solves for the position of the monkey and hunter using RK4
        ################################################################
        xBullet,xMonkey,n=rk4loop(FUNC,t,xBullet,vBullet,v_wind,xMonkey,vMonkey,h)
        
        if debug:
            print ('Bullet')
            print (xBullet[n-2:n+1])
            print ('Monkey')
            print (xMonkey[n-2:n+1])
        
    
        ################################################################
        # Plot
        # To reduce computing time just calculate points points between start and stop.
        ################################################################
        points=20
        xBulletPlot=xBullet[0:n:int(n/points)]
        xMonkeyPlot=xMonkey[0:n:int(n/points)]
        tPlot=t[0:n:int(n/points)]
        #Add last point calculated by RK4
        np.append(xBulletPlot,xBullet[n].reshape(1,2),axis=0)
        np.append(xMonkeyPlot,xMonkey[n].reshape(1,2),axis=0)
        np.append(tPlot,t[n])
        
        fig1,ax1,ax2= doPlot(mode,xBulletPlot,xMonkeyPlot,0,tPlot)    
        fig1.tight_layout()
        plt.show()
        
        ################################################################
        # Determining if it was a Hit or a Missed 
        # use only if step size is smaller than .0001
        ################################################################
        repeat,x0monkey=hitMiss(h,xMonkey,xBullet)

    
elif mode == 'gravity':
    if debug:
        print('The only force acting on bullet is gravity')
    
    
    v0bullet=1100  #m/s
    x0bullet=np.array([0.,0.])
    v0monkey=np.array([0.,0.]) #m/s
    x0random=np.random.uniform(0,100)
    x0=2000. + x0random
    y0random=np.random.uniform(0,20)
    y0=60. + y0random
    x0monkey=np.array([x0,y0])
    repeat=True
    v_wind=0
    
    while repeat:
        
        theta = np.float32(input('Select your shooting angle in degrees. '))
        theta = theta*np.pi/180.
        
        xBullet,vBullet,xMonkey,vMonkey,t=initialize(x0bullet,v0bullet,x0monkey,v0monkey,tf)
        if debug:
            print('x0Monkey',xMonkey[0])
            
        xBullet,xMonkey,n=rk4loop(Fgravity,t,xBullet,vBullet,v_wind,xMonkey,vMonkey,h)
        
        points=20
        xBulletPlot=xBullet[0:n:int(n/points)]
        xMonkeyPlot=xMonkey[0:n:int(n/points)]
        tPlot=t[0:n:int(n/points)]
        #Add last point calculated by RK4
        np.append(xBulletPlot,xBullet[n].reshape(1,2),axis=0)
        np.append(xMonkeyPlot,xMonkey[n].reshape(1,2),axis=0)
        np.append(tPlot,t[n])
        
        v0=vBullet[0]    
        xPlot=calcReal(points,xBulletPlot,tPlot,v0)
        
        fig1,ax1,ax2= doPlot(mode,xBulletPlot,xMonkeyPlot,xPlot,tPlot)
        plt.show()
        
        repeat,x0monkey=hitMiss(h,xMonkey,xBullet)
    

elif mode == 'airdrag':

    if debug:
        print('The forces acting on bullet is gravity and air resistance')
        
    v0bullet=1100  #m/s
    x0bullet=np.array([0.,0.])
    v0monkey=np.array([0.,0.]) #m/s
    x0random=np.random.uniform(0,100)
    x0=2000. + x0random
    y0random=np.random.uniform(0,20)
    y0=60. + y0random
    x0monkey=np.array([x0,y0])
    repeat=True
    v_wind=0
    
    while repeat:
        
        theta = np.float32(input('Select your shooting angle in degrees. '))
        theta = theta*np.pi/180.
        
        xBulletG,vBulletG,xMonkeyG,vMonkeyG,tG=initialize(x0bullet,v0bullet,x0monkey,v0monkey,tf)        
        xGravity,xMonkey,nG=rk4loop(Fgravity,tG,xBulletG,vBulletG,v_wind,xMonkeyG,vMonkeyG,h,debug)
        
        xBullet,vBullet,xMonkey,vMonkey,t=initialize(x0bullet,v0bullet,x0monkey,v0monkey,tf)
        xBullet,xMonkey,n=rk4loop(FwithAir,t,xBullet,vBullet,v_wind,xMonkey,vMonkey,h,debug)

        
        points=20
        xBulletPlot=xBullet[0:n:int(n/points)]
        xMonkeyPlot=xMonkey[0:n:int(n/points)]
        tPlot=t[0:n:int(n/points)]
        xGravityPlot=xGravity[0:n:int(n/points)]
        tPlotG=tG[0:n:int(n/points)]
        #Add last point calculated by RK4
        np.append(xBulletPlot,xBullet[n].reshape(1,2),axis=0)
        np.append(xMonkeyPlot,xMonkey[n].reshape(1,2),axis=0)
        np.append(xGravityPlot,xGravity[n].reshape(1,2),axis=0)
        np.append(tPlot,t[n])
        np.append(tPlotG,tG[n])
        
        
        fig1,ax1,ax2= doPlot(mode,
                             xBulletPlot,
                             xMonkeyPlot,
                             xGravityPlot,
                             tPlot,
                             tPlotG,
                             debug)
        plt.show()
        
        repeat,x0monkey=hitMiss(h,xMonkey,xBullet)
        
elif mode == 'wind':
    FUNC=FwithWind
    v_wind,speed,direction=calcVWind()
    if debug:
        print('The forces acting on bullet is gravity, air resistance and wind.')

else:
    print('No mode selected')
