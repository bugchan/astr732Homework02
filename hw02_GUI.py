#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 15 09:17:11 2019

@author: root
"""

import Tkinter as Tk

master=Tk.Tk()

master.title('The Monkey and The Hunter') 

col0Width=20

AirResistance = Tk.IntVar() 
Tk.Checkbutton(master,
            text='Air Resistance',
            variable=AirResistance).grid(row=0,
                                  sticky=Tk.W) 

Wind = Tk.IntVar() 
Tk.Checkbutton(master,
            text='Wind',
            variable=Wind).grid(row=1,
                         sticky=Tk.W)

comm = Tk.Label(master,
             text='Will you be able to hit the Monkey? Choose your pointing angle',
             width=20,
             wraplength=150,
             #anchor=Tk.W,
             justify=Tk.LEFT,
             relief=Tk.SUNKEN,
             ) 
comm.grid(row=5) 


btnShoot = Tk.Button(master,
                   text='Shoot',
                   width=20,
                   command=3+1)
btnShoot.grid(row=6,sticky=Tk.W)

#button.pack() #not needed if specify grid

c = Tk.Canvas(master, width=200, height=100)
c.grid(column=2)
#canvas.pack() 
canvas_height=100
canvas_width=200
y = int(canvas_height / 2) 
c.create_line(0, y, canvas_width, y)




master.mainloop() 
