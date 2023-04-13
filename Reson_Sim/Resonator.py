import numpy as np
import math as mt
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt  
import time
import os 
from pathlib2 import Path
import urllib3
from astropy.io import fits 
import astropy.modeling.functional_models as astromodels
from scipy.integrate import simpson as sps   
import pickle as pkl

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

#Broadening parameters
#===========================
lorentzian_fwhm=20 #in pm
gaussian_fwhm=20   #in pm
#===========================

#Noise parameter
#=====================
noise=0.01   #in percent of normalized background signal .  It's the standard deviation of the normally-disttributed noise contribution


#Set the exoplanet contribution absorption depths  (in %)
#===========================
exoplanet_abs_depth=100  #in percent


#print(gas_names)
#plt.show()

class Resonator:
    def __init__(self,Temp = np.arange(0,10,1) ,wave_start=1575E-9,wave_finish=1590E-9, ring_length = ring_length, coupling_in = r1,coupling_out = r2,rt_losscoef = a, concentration = 100,save_dir =cwd+ '\\Reson_Sim\\Training_db\\') -> None:
        #Initialisation

        self.start = time.time()
        self.concentration = concentration
        self.gas_init()
        self.temperature = Temp
        self.wavelength=np.arange(wave_start, wave_finish,(wave_finish-wave_start)/(2**(round(mt.log2((wave_finish-wave_start)/10E-12-1)))+1))
        self.concentration = concentration   
        self.number_of_wavelengths=len(self.wavelength)
        self.n_eff = n_eff_interp(self.wavelength)
        self.n_g = n_g_interp(self.wavelength)
        self.ring_length = ring_length
        self.r1 = coupling_in #properties of the ring coupling into the waveguide 
        self.r2 = coupling_out #properties of the ring coupling out of the waveguide
        self.a = rt_losscoef # round trip loss coefficient of the ring resonator
        self.noise_profile = np.random.normal(1, noise/100, size=len(self.wavelength))
        self.produce_spectrum()


    def gas_init(self):
                #Set the gas saturation fraction  (in %)
        #===========================
        C2H2=0
        CH3=0
        CH4=0
        CO=0
        CO2= self.concentration
        H2O=0
        HCN=0
        NH3=0
        NO=0
        O2=0
        O3=0
        OH=0
        self.absorption_depths=[C2H2,CH3,CH4,CO,CO2,H2O,HCN,NH3,NO,O2,O3,OH] #Put the absorption depths into an array
        #===========================


        #Initalize arrays

        self.gas_names=[]
        self.gas_names_ext=[]
        self.absorption_array=[]
        self.gas_names_present='Gases: '
        # find all the files in the Lines directory
        i=0
        for entry in os.listdir(basepath+"\\Lines\\"):
            if os.path.isfile(os.path.join(basepath+"\\Lines\\", entry)):   #check if there is a file in the directory
                self.gas_names.append(entry[:-4])    #remove the file extension
                self.gas_names_ext.append(entry)     #build an array of the file names with the extensions
            i=i+1


    #This function calculates the spectral shift as a function of chip temperature
    def temperature_wavelength_shift(self, temp:float)->np.array:
        alpha_w_si=2.5e-6  #coefficient of thermal expansion of Si
        alpha_w_sio2=1e-5  #coefficient of thermal expansion of SiO2 (not used)
        sigma_t=1.97e-4  #rate of change of effective index with temp (thermoooptic effect)
        thermal_expansion=alpha_w_si*self.n_eff*self.wavelength*temp/self.n_g; #thermal expansion effect (negligible)
        thermooptic_effect=sigma_t*self.wavelength*temp/self.n_g;      #thermooptic effect, change in index with temperature
        wave_shift = thermal_expansion + thermooptic_effect         #add the two effects together
        return wave_shift

    def produce_spectrum(self)->None:

        y=np.zeros((len(self.wavelength),1))
        self.gas_names_present='Gases: '
        gas_count=0  #index of the gas.
        gas_present_count=0
        for i in self.gas_names_ext:   #Iterate over the files that were loaded (including file extension)

            if self.absorption_depths[gas_count] != 0:  #Why bother doing calculations if we don't want the gas in the bandpass? Let's skip it
                self.absorption_array=np.loadtxt(basepath+"\\Lines\\"+self.gas_names_ext[gas_count]) #Load the HITRAN data into an array from text file
                line_count=0  #Start spectral line count for one gas
                print(self.gas_names[gas_count] + " at abs depth of " + str(self.absorption_depths[gas_count])+ '%')   #Print the gas line absorption depth and gas name
                self.gas_names_present+=str(self.gas_names[gas_count]) + ', '
                self.absorption_array[:,1]=self.absorption_array[:,1]/max(self.absorption_array[:,1])  #Normalize the absorption cross-section

                for i in self.absorption_array[:,1]:
                    v1 = astromodels.Voigt1D(x_0=self.absorption_array[line_count,0]*nm, amplitude_L=1.3*self.absorption_depths[gas_count]*self.absorption_array[line_count,1]/100, fwhm_L=lorentzian_fwhm*pm, fwhm_G=gaussian_fwhm*pm) #Calculate voigt profiles for one line at a time
                    y[:,0]=y[:,0]+v1(self.wavelength)        #Add voigt profiles to the spectra
                    line_count=line_count+1 #Count iteration for each spectral line in the wavleength region

                gas_present_count=gas_present_count+1    
            gas_count=gas_count+1   #Count iteration for next gas
        print(self.gas_names_present)
        abs_spectrum=1-y        #Subtract from normalized flat background (should add a star background eventually...one day....*sigh*)
        self.abs_spectrum= abs_spectrum.clip(min=0)      #Clip the data since you can't have negative light intensity
        #abs_spectrum=1-abs_spectrum                 #Flip the data to prep for the exoplanet abs fraction contribution
        #abs_spectrum=(exoplanet_abs_depth/100)*abs_spectrum         #Scale by the exoplanet abs fraction contribution 
        #self.abs_spectrum=1-abs_spectrum                         #Flip the data back
        noisy_abs_spectrum=np.multiply(self.noise_profile,np.transpose(abs_spectrum))            #Add noise profile.
        self.noisy_abs_spectrum=np.transpose(noisy_abs_spectrum)                                 #Transpose so we can plot it
        
        self.drop_port_spectrum = []
        self.through_port_spectrum = []
        self.output_spectrum = []
        self.correlation =[]
        prop_constant=2*PI*self.n_eff/self.wavelength
        theta=prop_constant*self.ring_length #no shift
        
        index = 0
        for i in self.temperature:
            wave_shift=self.temperature_wavelength_shift(i)  #The wavelength shift for the spectrum with temeprature
            #theta is the phase of the resonance cavity (ring resonator)
            theta2=(2*PI*self.n_eff/(self.wavelength-wave_shift))*self.ring_length #with shift
            
            through_port_spectrum=((self.r2**2)*(self.a**2)-2*self.r1*self.r2*self.a*np.cos(theta2)+self.r1**2)/(1-2*self.r1*self.r2*a*np.cos(theta2)+(self.r1*self.r2*self.a)**2) #calculation of through port spectrum
            drop_port_spectrum=((1-self.r1**2)*(1-self.r2**2)*self.a)/(1-2*self.r1*self.r2*self.a*np.cos(theta2)+(self.r1*self.r2*self.a)**2)  #How the drop port spectrum is calculated
            output_spectrum  = abs_spectrum[0]*through_port_spectrum
            correlation  = sps(output_spectrum,self.wavelength)
            
            self.through_port_spectrum+=[through_port_spectrum]
            self.drop_port_spectrum+=[drop_port_spectrum]
            self.output_spectrum += [output_spectrum]
            self.correlation+=[correlation]
            index+=1

    
    def save_spectrum(self)->None:
        pass    


    def display_port(self,range:range)->None:
        plt.figure(dpi=300)
        
        legend = []
        for i in range:
            plt.plot(self.wavelength,self.drop_port_spectrum[i])
            plt.plot(self.wavelength,self.through_port_spectrum[i])
            legend+=[f"drop port @ T°={self.temperature[i]}°C",f"through port @ T° change={self.temperature[i]}°C"]    

        plt.legend(legend)
        plt.xlabel("Wavelength(m)")
        plt.ylabel("Normalised Power")
        plt.show()

    def display_output(self,range:range)->None:
        plt.figure(dpi=300)
        legend = ["absorption spectrum"]
        plt.plot(self.wavelength,self.temperature[-1]+self.abs_spectrum)
        for i in range:
            delta = self.drop_port_spectrum[i]-self.output_spectrum[i]
            plt.plot(self.wavelength,self.temperature[i]+self.output_spectrum[i],linewidth = 0.5)
            legend+=[f"Resonator Output @ T° change ={self.temperature[i]}°C"]
        
        #plt.legend(legend,framealpha = 0.01,loc='lower right')
        plt.xlabel("Wavelength(m)")
        plt.ylabel("Normalised Power")

    def display_correlation(self)->None:
        c_max = max(self.correlation)
        c_min = min(self.correlation)
        plt.figure(dpi=300)
        plt.scatter(self.temperature[self.correlation == c_max][0],c_max,c = "green")
        plt.scatter(self.temperature[self.correlation == c_min][0],c_min,c = "red")
        plt.plot(self.temperature,self.correlation)

        plt.xlabel("Temperature change °C")
        plt.ylabel("Correlation")
        plt.legend(["Max","Min"])
        
    def cmin(self) -> None:
        
        return(np.std(self.correlation))
        
    
    """Private Functions"""

    def _topkl(self,*args) -> None: 
        dump_pkl = []
        file_name = f"Training_C{self.concentration}.pkl"
        for i in args:
            dump_pkl += [i]
        with open(self.save_dir+file_name, 'wb') as f:
            pkl.dump(dump_pkl,f)
        print(f"{file_name} saved!")    


    def transmit_spectrum(self)->None: #send spectrum to the raspberry-pi
        pass


            