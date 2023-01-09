import time,os,urllib3,board,busio
import numpy as np 
import math as mt
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt  
from pathlib2 import Path
from astropy.io import fits 
import astropy.modeling.functional_models as astromodels
from scipy.integrate import simpson as sps   
import adafruit_mcp4725 as MCP
import adafruit_ads1x15 as ADS

#constants
path_length = 5 # in meters
DAC_res =  12
ADC_res =  16

#create I2C bus
i2c = busio.I2C(board.SCL,board.SDA)

class PI_Controller:
    """Raspberry pi code to activate the (S)LED, heat the u-chip and collect signal from the photodiode"""
    
    #Initialisation
    def __init__(self,test = 0,continuous = 0,test_duration = 5,prototype = 0) -> None:
        self.test = test 
        self.continuous = continuous #if reading loops itself
        self.prototype  = prototype # which prototype is being worked on 0:first,1:final

        if self.test: #if test duration in minutes
            self.test_duration = test_duration 

    """Private functions in charge of hardware signal sending and collection"""
    #function governing LED activation, DAC
    def __LED(self):

        if not self.prototype:
            pass
        else:
            pass
        pass

    #function producing ramp signal to heat the u-chip, DAC
    def __Temp(self):
        pass

    #function reading the photodiode output , ADC    
    def __PhotoDRead(self):
        pass    

    #function storing initial T°=0 absorption throughport spectrum to compare with T°>0 spectrums 
    def _Calibrate(self):
        pass