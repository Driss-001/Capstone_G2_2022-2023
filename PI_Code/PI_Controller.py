
import sys,time,os,urllib3,board,busio,pwmio,threading
from adafruit_extended_bus import ExtendedI2C as I2C
import math as mt
import numpy as np
import matplotlib.pyplot as plt  
from pathlib2 import Path
from astropy.io import fits 
from scipy.integrate import simpson as sps   
import adafruit_mcp4725 as MCP
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as Gpio
import datetime as dt
import pickle as pkl
from scipy.stats import linregress

#import pigpio  
#import progressbar as pbar

#constants
path_length = 3 # in meters
DAC_res =  12
ADC_res =  16
VMAX = 3.3 #max volt-pin output voltage

#constant functions
dac_raw = lambda volt: volt/VMAX*2**DAC_res
#create I2C buses

DAC_ADDRESS = 0x62
ADC_ADDRESS = 0x48
i2c1 = I2C(1)
i2c2 = I2C(6)


adc = ADS.ADS1115(address=ADC_ADDRESS,i2c = i2c2,data_rate=860)
#get_dacvolt  = lambda x: x*VMAX/2**ADC_res
#get_adcvolt = lambda x: x*VMAX/2**DAC_res
Gain = ADC_res/16 #gain of 1

#pwm init
RPI_pin = 13
PWM_f =2e3
#1MHz frequency
Gpio.setmode(Gpio.BCM)
Gpio.setup(RPI_pin, Gpio.OUT)
Gpio.setup(RPI_pin,Gpio.LOW)
duty_cycle = lambda x: round((2**16-1)/100*x) #Duty cycle is 16bits, return duty cycle percentage
#pwm = pwmio.PWMOut(board.D13,frequency = PWM_f) #initialising pwm with desired frequency
pwm =Gpio.PWM(RPI_pin,PWM_f)
pwm.start(0)

#pwm = pigpio.pi() 
#pwm.set_mode(RPI_pin, pigpio.OUTPUT)


#Number of triangle ramp signal/correlations wanted
CORR_NUM = 1
cwd  = os.getcwd()

class PI_Controller:
    """Raspberry pi code to activate the (S)LED, heat the u-chip and collect signal from the photodiode
       input: test #(0,1) currently supported, continuous 0, test duration: wanted test duration in sec
                dpi for graphical output, sampling_f sampling frequency 
    
    
    """
    
    #Initialisation
    def __init__(self,test = 0,test_duration = 5,dpi = 300,V_supply = 5,sampling_f = 100, autorun = 0,save_dir = cwd, c_noise = False,n_iter = 10, conc = 100, Training = False, detection = False,resistance = 72) -> None:

        self.test_status = test
        self.v_supply = V_supply
        self.concentration = conc
        self.Training = Training
        self.n_iter = n_iter
        self.resist = resistance
        self.detection =  detection
        if self.Training:
            self.save_dir = save_dir+'/PI_Code/training/'
        else:
            self.save_dir = save_dir+'/PI_Code/data/'
        self.data_dir =   save_dir+'/PI_Code/training/'                  
        self.m_run = c_noise
        self.current_date = dt.datetime.now().strftime('%m-%d-%H-%M')
        self.test_duration = test_duration           
        self.active = True
        self.dpi = dpi
        self.Reset_time()
        self.sampling_f = sampling_f
        if self.sampling_f >=860:
            self.sampling_f = 860
        self._init_arrays()
        self.num_samples = round(self.sampling_f*self.test_duration) #Number of samples wanted: constant int
        
        if bool(autorun): #Determine autorun
            if not self.m_run:
                self.Run() #switch on
            else:
                self.M_Run(self.n_iter)

       
            
    


    def Run(self) -> None: #activate switch function , auto-start
    
        #    #print("flag")
        self.t_start = self._now()
        self.perc_1=0
        self.th1 = threading.Thread(target = self._HW_start)
        self.th1.daemon = True
        self.th1.start()

        
        while self.counter< self.num_samples+1: #finish signal by points collected
            self._Temp()
            self._progbar(self.counter,self.num_samples,self._now(self.t_start),"Sampling")
            try:
                pass
            except KeyboardInterrupt:
                pwm.stop() #On KB int stop the pwm
        
        #self.pwm_stop()
        n = self.num_samples 
        if self.detection:
            self.conc_detect(n)
            print(f"CO2 concentration detected :{self.concentration}%")
        if not self.m_run:
            pwm.stop()
            self._figure_pkl(n)
            self._save_figs(n)
            print(f"{self._now(self.t_start)} secs have passed, test finished!")
        else:
            print("run finished")        
        self.Switch()
        self.th1.join()


    def M_Run(self,n_iter = 10)->None:
        ADC_Gauss = np.zeros(self.num_samples)
        for i in range(0,n_iter):
            self.Run()
            ADC_Gauss+= np.array(self.adc_output[0:self.num_samples])/n_iter
            pwm.stop()
            time.sleep(0.5) #buffer
            if i == n_iter-1:
                pass
            else:
                self._init_arrays()
                self.Switch()
                pwm.start(1)      
        self.adc_output = ADC_Gauss     
        n = self.num_samples
        self._figure_pkl(n)
        self._save_figs(n)

    def Switch(self) -> None: #toggle activate deactivate
        
        self.active = not self.active
        print(f"active bool is now {self.active}")

    def De_Activate(self)-> None:
        self.active = False

    def Reset_time(self):
        self.start_time = time.time()

    def Reset_Arrays(self):
        self._init_arrays()    


    def pwm_stop(self)->None:

        pwm.hardware_PWM(RPI_pin, 0, 0)               # turn off the PWM

    def conc_detect(self,n):
        f = self._model()
        current_min = self._corr_min(n)#min(self.adc_output[0:n])
        self.concentration = round(f(current_min),2)
         

    def display_Reg(self): #Display current linear regression
        f = self._model()
        dip = np.linspace(min(self._cmin)*.999,max(self._cmin)*1.001,100)
        conc = np.vectorize(f)(dip)
        plt.plot(dip,conc)
        plt.scatter(self._cmin,self._c,c="r")
        plt.ylabel('Concentration (%)')
        plt.xlabel('Minimum Voltage Detected(V)')
        plt.title(f"Current Training DB Regression Line ,date:{self.current_date}")
        

    """Private functions in charge of hardware signal sending and collection"""


    """Array Init private function"""
    def _init_arrays(self) -> None:
        self.t_period =  self.test_duration/CORR_NUM
        self.counter = 0
        
        # Initialise arrays
        match self.test_status:
            case 0: #PI simple signals
                self._init_DAC()
                self.dac_order = []
                self.dac_order_time =[]
                self.adc_output = []
                self.adc_output_time = []
                self.pwm_output = []
                self._test = np.bool_([0,0]) #voltage test
                ADS.mode = 0 # put ads in continuous mode for reading speed  
            case 1: #Proto 1
                self.adc_output = []
                self.adc_output_time = []                
                self._test = np.bool_([1,0]) 
                self.time = time.time()
                ADS.mode = 0
            case 2: #Proto 2
                self._test = np.bool_([0,1])  #signal test
                self.time = time.time()
         
            case 3: #Real case
                self._test = np.bool_([1,1]) 

    """Hardware functions"""

    #Streamlined ADC/DAC function to manipulate in/out voltages

    def _set_DAC(self,voltage)->None: #set dac ouput voltage
        if voltage > VMAX:
            voltage = VMAX
        dac_volt = round(voltage/VMAX*(2**DAC_res-1))    
          
        try:
            self.dac._write_fast_mode(dac_volt)#attempt write fast-mode
        except:
            print(f"DAC voltage output must be between 0 and {VMAX} V")  
    
    def _ADC_volt(self,channel = 0)->float: #return ADC voltage from selected channel
        match channel:
            case 0:
                chan0 = AnalogIn(adc,ADS.P0)
                return chan0.voltage
            case 1:
                chan1 = AnalogIn(adc,ADS.P1)
                return chan1.voltage                  
            case 2:
                chan2 = AnalogIn(adc,ADS.P2)
                return chan2.voltage                 
            case 3:
                chan3 = AnalogIn(adc,ADS.P3)
                return chan3.voltage 
            
    def _init_DAC(self)->None: #Initialise DAC module
        self.dac = MCP.MCP4725(address=DAC_ADDRESS,i2c=i2c1)

    #function governing LED activation, DAC
    def __LED(self) -> None:
        #no proto or proto 1
        if not self._test[1] and not self._test[0]:
            
            #print(f"counter aues is: {self.counter}")
            dac_order =self._triangle(self.t_period,VMAX)     
            self._set_DAC(dac_order)
            self.dac_order.append(dac_order)
            self.dac_order_time.append(self._now())
            return
        if not self._test[1] and  self._test[0]:
            pass
        pass

    #function driving ramp signal to heat the u-chip, DAC
    def _Temp(self) -> None:
        #DC = 10 #50% SC test
        DC = self._triangle(self.t_period,100)
        if DC>100:
            DC = 100
        elif DC <0:
            DC =0   
        #print("Applying voltage to the Chip...")
        #pwm.duty_cycle = duty_cycle(DC) #99% Duty cycle for DC voltage
        #print(DC)
        pwm.ChangeDutyCycle(DC)
        #pwm.ChangeDutyCycle(90)
        #pwm.hardware_PWM(RPI_pin, PWM_f, DC * 10000)

        
    #function reading the photodiode output , ADC    
    def __PhotoDRead(self) -> None:
   
        if not self._test[1] and not self._test[0]: #case signals    """Hardware functions"""
            self.adc_output.append(self._ADC_volt(0))
            self.pwm_output.append(self._ADC_volt(1))
            self.adc_output_time.append(self._now(self.t_start))
            return 
        if not self._test[1] and  self._test[0]:
            self.adc_output.append(self._ADC_volt(2))
            self.adc_output_time.append(self._now(self.t_start))
        pass    

        self.counter = len(self.adc_output) #recorded DAC output length

    def _now(self,t=0): #current time for performance tracking    """Hardware functions"""
        return round(time.time()-self.start_time-t,3) 
        
        
    def _Calibrate(self) -> None:
        if not self._test[1] and not self._test[0]:
            pass
        else:
            pass
        pass

    def _freq_AutoCal(self) -> None:
        new_freq = self.sampling_f
        self.perc_2 = self.counter/self.num_samples*100
        if abs(self.perc_2-self.perc_1)>10: #average check over 10%
            new_freq = self.counter/self._now(self.t_start)
            print(self.perc_2,new_freq,self.sampling_f)
            self.perc_1 = self.perc_2
        if abs(new_freq-self.sampling_f)>1:
            #self.Switch() #Turn off parallel thread
            print(f"Real frequncy is {new_freq} HZ instead of {self.sampling_f} Hz, recalibrating...")
            self.sampling_f = new_freq
            self.test_duration = self.num_samples/self.sampling_f
            print(f"New Test duration is {self.test_duration}")
            self._init_arrays()
            print("Status is now progress %s, Active:%s" %(self.counter/self.num_samples*100,self.active))
            



    def _HW_start(self) -> None: #activate all hardware signals and readers
        self._Calibrate()
        
        while self.active:    #main parallel thread loop
            self.__LED()
            self.__PhotoDRead()
            #elf._freq_AutoCal()

            #self.__Temp()
            #time.sleep(1/self.sampling_f-1/860) #adjust to sampling frequency + 860Hz round for DAC conversion
        #pwm.duty_cycle = duty_cycle(0) #reset the pwm    
        pwm.stop()    

    def _progbar(self,count_value, total,T_now,suffix=''):
        bar_length = 100
        filled_up_Length = int(round(bar_length* count_value / float(total)))
        percentage = round(100.0 * count_value/float(total),1)
        bar = '=' * filled_up_Length + '-' * (bar_length - filled_up_Length)
        sys.stdout.write('Progress[%s] %s%s,Time elapsed:[%s%s] ...%s\r' %(bar, percentage, '%', T_now,"s",suffix))
        sys.stdout.flush()

    #seasaw signal function for PWM-DC system    
    def _triangle(self,period,peak) -> float:
        now_time = self._now()
        now_frac = now_time//(period) #euclidian division 
        now_mod = now_frac%2 #which period are we on obtained by modulus
        #linear function going from 0 to peak in period/2

        #if now_mod == 0: #if in first half of the triangle positive slope
        #    return 2*peak/period*(now_time-period/2*now_frac)
        #else:
        #    return peak*(1-1/period*2*(now_time-period/2*now_frac))
        return peak/period*(now_time-period*now_frac) #seasaw function

    """Private functions for data handling"""

    def _model(self):
        l = []
        c = []
        corr_min = []
        for root, dirs, files in os.walk(self.data_dir):
            for names in files:
                l.append(names)
        for file  in l:
            with open(self.data_dir+file, 'rb') as f:
                fetch = pkl.load(f)
                c.append(fetch[3])
                corr_min.append(fetch[2])
        corr_min = np.array(corr_min)
        c = np.array(c)  
        m1 = linregress(corr_min,c)
        self._c = c
        self._cmin = corr_min
        lin_model = lambda x: m1.intercept+x*m1.slope  
        return lin_model       
    
    def _corr_min(self,n)->float:

        d = lambda i: (self.adc_output[i]-self.adc_output[i-1]) #difference function
        l_max = []
        V_100 = self.resist/10
        self.dip_perc = 1
        
        if V_100>=self.v_supply:
            self.dip_perc = self.v_supply/V_100/CORR_NUM
        

        for i in range(0,round(n*self.dip_perc)):
            if d(i) >0 and d(i+1)<0: #find local maximas in the non dip region
                t_min  =self.adc_output[i+1]
                l_max.append(t_min) 
        l_max = np.array(l_max)

        #returns the ratio between average on the maximas in thr non dip zone zone and minimum of dip zone for Vsupply = 5V
        level = np.average(self.adc_output[0:round(0.2*self.dip_perc*n)])
        return (level-np.average(self.adc_output[mt.floor(0.4*self.dip_perc*n):round(0.6*self.dip_perc*n)]))  #the minimum voltage in the anticipated dip zone

    
    def _figure_pkl(self,n):
        if not self._test[1] and not self._test[0]: #Test 0 save (x,y) coords
            self._topkl(self.adc_output_time[0:n],self.adc_output[0:n],self.pwm_output[0:n])
        if not self._test[1] and  self._test[0]:    #Test 1 save (x,y) coords, min(y) & gas concentration
            self._topkl(self.adc_output_time[0:n],self.adc_output[0:n],self._corr_min(n),self.concentration)         

    def _save_figs(self,n) -> None:
        if not self._test[1] and not self._test[0]:
            plt.plot(self.dac_order_time[0:n],self.dac_order[0:n],c ="black")
            print(len(self.adc_output),len(self.adc_output_time),n)
            plt.scatter(self.adc_output_time[0:n],self.adc_output[0:n],c="red")
            plt.scatter(self.adc_output_time[0:n],self.pwm_output[0:n],c="green")
            plt.legend(["DAC py-order","ADC chan0 output (DAC)","ADC chan 1 output(PWM)"])
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test0,sampling @ {self.sampling_f}Hz,date:{self.current_date}")
            plt.savefig("Test0_ADC_DAC_output",dpi = self.dpi)   

            plt.clf()

            #plt.plot(np.array(range(1,len(self.dac_order_time)+1)),self.dac_order_time,c="blue")
            #plt.plot(np.array(range(1,len(self.adc_output_time)+1)),self.adc_output_time,c="red")
            #plt.legend(["DAC order time","ADC output time"])
            #plt.ylabel("time(s)")
            #plt.title("Rpi4 IO Latency Test0")
            #plt.savefig("Test0_Latency_output",dpi = self.dpi)
            return
        if not self._test[1] and  self._test[0]:  
            plt.plot(self.adc_output_time[0:n],self.adc_output[0:n],c="green")
            plt.fill_between(self.adc_output_time[mt.floor(0.4*self.dip_perc*n):round(0.6*self.dip_perc*n)],len(self.adc_output_time[mt.floor(0.4*self.dip_perc*n):round(0.6*self.dip_perc*n)])*[np.max(self.adc_output)])
            plt.legend(["ADC chan1 output  (Photodiode)","Anticipated minimum zone"])    
            plt.ylabel('Voltage (V)')
            plt.xlabel('time(s)')
            plt.title(f"Rpi4 IO DAC/ADC Test1,CO2 concentration {self.concentration}%,Suppluy voltage {self.v_supply} V")
            plt.savefig(f"Test1_ADC_output_C{int(round(self.concentration))}_ {self.v_supply}V_{self.current_date}.png",dpi = self.dpi)
            plt.clf()   

        print("figure Saved!") 
        
        if  self._test[1] and  not self._test[0]:  
            pass
        


  

    """" Save & load Graphs pkl  """
    def _topkl(self,*args) -> None: 
        dump_pkl = []
        if not self.Training:
            folder = self.save_dir
            file_name = f"test{self.test_status}_{self.current_date}_C{round(self.concentration,2)}.pkl"
        else:
            folder = self.data_dir
            file_name = f"test{self.test_status}__C{self.concentration}.pkl"    
        for i in args:
            dump_pkl += [i]
        with open(folder+file_name, 'wb') as f:
            pkl.dump(dump_pkl,f)
        print(f"{file_name} saved!")

    def _frompkl(self, test_status,current_date = None,c = 100) -> list: #so far only for training

        if not self.Training:
            file_name = f"test{test_status}_{current_date}.pkl" # Date for non-training data, showcase
        else:    
            file_name = f"test{test_status}__C{c}.pkl"   #Training data only requires concentration as identifier

        with open(self.save_dir+file_name, 'rb') as f:
            pkl_fetch = pkl.load(f)
        return pkl_fetch    

if __name__ == '__main__':
    #test0 = PI_Controller(test_duration=20/60)
    test1 = PI_Controller(test =1,test_duration =.1,V_supply=5,n_iter = 15,sampling_f=100,autorun=1,conc=100,c_noise=False, Training = False,detection = True ) #10 points frequency teut