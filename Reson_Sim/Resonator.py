import numpy as np
import math as mt
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt  
import time
import os 
from pathlib2 import Path
import urllib3

#Constants
PI=mt.pi
nm=1E-9
pm=1E-12

start = time.time()
r1=0.9   #properties of the ring coupling into the waveguide 
r2=0.9   #properties of the ring coupling out of the waveguide
a=0.9       # round trip loss coefficient of the ring resonator     
ring_length=1.223E-3


cwd = os.getcwd() #Get current working directory
basepath = cwd+ '\\Reson_Sim\\src\\'

#pathfinding to required external iles
path_neff = Path(basepath+'neff_TE_h_220nm_w_450nm.txt') # effective index of the waveguide in the ring
path_ng = Path(basepath+'ng_TE_h_220nm_w_450nm.txt') # group index of the waveguide in the ring, affects how closely spaced the ring resonaces are
n_eff=np.loadtxt(path_neff)
n_g=np.loadtxt(path_ng)

n_eff_interp = InterpolatedUnivariateSpline(n_eff[:,0], n_eff[:,1], k=1)
n_g_interp = InterpolatedUnivariateSpline(n_g[:,0], n_g[:,1], k=1)

#plt.show()

class Resonator:
    def __init__(self,Temp = np.arange(0,10,1) ,wave_start=1575E-9,wave_finish=1585E-9, ring_length = ring_length, coupling_in = r1,coupling_out = r2,rt_losscoef = a) -> None:
        #Initialisation

        self.start = time.time()
        self.temperature = Temp
        self.wavelength=np.arange(wave_start, wave_finish,10E-12)    
        self.number_of_wavelengths=len(self.wavelength)
        self.n_eff = n_eff_interp(self.wavelength)
        self.n_g = n_g_interp(self.wavelength)
        self.ring_length = ring_length
        self.r1 = coupling_in #properties of the ring coupling into the waveguide 
        self.r2 = coupling_out #properties of the ring coupling out of the waveguide
        self.a = rt_losscoef # round trip loss coefficient of the ring resonator
        self.produce_spectrum()


    #This function calculates the spectral shift as a funciton of chip temperature
    def temperature_wavelength_shift(self, temp:float)->np.array:
        alpha_w_si=2.5e-6  #coefficient of thermal expansion of Si
        alpha_w_sio2=1e-5  #coefficient of thermal expansion of SiO2 (not used)
        sigma_t=1.97e-4  #rate of change of effective index with temp (thermoooptic effect)
        thermal_expansion=alpha_w_si*self.n_eff*self.wavelength*temp/self.n_g; #thermal expansion effect (negligible)
        thermooptic_effect=sigma_t*self.wavelength*temp/self.n_g;      #thermooptic effect, change in index with temperature
        wave_shift = thermal_expansion + thermooptic_effect         #add the two effects together
        return wave_shift

    def produce_spectrum(self)->None:
        
        self.through_port_spectrum = []
        prop_constant=2*PI*self.n_eff/self.wavelength
        theta=prop_constant*self.ring_length #no shift
        self.drop_port_spectrum=((1-self.r1**2)*(1-self.r2**2)*self.a)/(1-2*self.r1*self.r2*self.a*np.cos(theta)+(self.r1*self.r2*self.a)**2)  #How the drop port spectrum is calculated
        for i in self.temperature:
            wave_shift=self.temperature_wavelength_shift(i)  #The wavelength shift for the spectrum with temeprature
            #theta is the phase of the resonance cavity (ring resonator)
            theta2=(2*PI*self.n_eff/(self.wavelength-wave_shift))*self.ring_length #with shift
            
            through_port_spectrum=((self.r2**2)*(self.a**2)-2*self.r1*self.r2*self.a*np.cos(theta2)+self.r1**2)/(1-2*self.r1*self.r2*a*np.cos(theta2)+(self.r1*self.r2*self.a)**2) #calculation of through port spectrum
            
            self.through_port_spectrum+=[through_port_spectrum]

    
    def save_spectrum(self)->None:
        pass    


    def display_spectrum(self,range:range)->None:
        plt.figure(dpi=600)
        plt.plot(self.wavelength,self.drop_port_spectrum)
        legend = ["drop port spectrum"]
        for i in range:
            plt.plot(self.wavelength,self.through_port_spectrum[i])
            legend+=[f"through port @ T°={self.temperature[i]}°C"]    

        plt.legend(legend)
        plt.xlabel("Wavelength(m)")
        plt.ylabel("Normalised Power")
        plt.show()

    def transmit_spectrum(self)->None:
        pass


            