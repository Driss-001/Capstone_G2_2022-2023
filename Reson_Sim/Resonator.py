import numpy as np
import math as mt
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt  
import time
import os 

#Constants
PI=mt.pi
nm=1E-9
pm=1E-12

start = time.time()
r1=0.9   #properties of the ring coupling into the waveguide 
r2=0.9   #properties of the ring coupling out of the waveguide
a=0.9       # round trip loss coefficient of the ring resonator
n_eff=2.31        # effective index of the waveguide in the ring
n_g=4.34         # group index of the waveguide in the ring, affects how closely spaced the ring resonaces are


cwd = os.getcwd() #Get current workign directory
basepath = cwd+ '\\src\\'
#plt.show()

class Resonator:
    def __init__(self,Temp = np.arange(0,10,1) ,wave_start=1400E-9,wave_finish=1590E-9) -> None:
        #Initialisation
        self.start = time.time()
        self.wavelength=np.arange(wave_start, wave_finish,10E-12)    
        pass