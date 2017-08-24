# Copyright Stuff For HTU21D-F Driver Code (found online)
#
# Raspberry Pi Driver for Adafruit HTU21D-F
# Copyright (c) 2014 D. Alex Gray
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from __future__ import division
import spidev
import time
import datetime
import matplotlib.dates as mdates
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import pifacedigitalio
import pigpio
import math
import pickle
GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)
pifacedigital = pifacedigitalio.PiFaceDigital()     # setup relay controller

# Setup for I2C communication for Humidity and Temperature Sensor
pi = pigpio.pi()

# HTU21D-F Address
addr = 0x40

# i2c bus, if you have a Raspberry Pi Rev A, change this to 0
bus = 1

# HTU21D-F Commands
rdtemp = 0xE3
rdhumi = 0xE5
wtreg = 0xE6
rdreg = 0xE7
reset = 0xFE

errors = []     # initialize I2C error counting array

def htu_reset():
	handle = pi.i2c_open(bus, addr) # open i2c bus
	pi.i2c_write_byte(handle, reset) # send reset command
	pi.i2c_close(handle) # close i2c bus
	time.sleep(0.2) # reset takes 15ms so let's give it some time

def read_temperature():
        successtemp = False
        exception_temp = None
        for _ in range(5):
                try:
                        handle = pi.i2c_open(bus, addr) # open i2c bus
                        pi.i2c_write_byte(handle, rdtemp) # send read temp command
                        time.sleep(0.055) # readings take up to 50ms, give it some time
                        (count, byteArray) = pi.i2c_read_device(handle, 3) # vacuum up 
                        												   # those bytes
                        pi.i2c_close(handle) # close the i2c bus
                        t1 = byteArray[0] # most significant byte msb
                        t2 = byteArray[1] # least significant byte lsb
                        temp_reading = (t1 * 256) + t2 # combine both bytes into one 
                        							   # big integer
                        temp_reading = math.fabs(temp_reading) # I'm an idiot and can't 
                        					# figure out any other way to make it a float 
                        temperature = ((temp_reading / 65536) * 175.72 ) - 47.8 # formula
                        												# from datasheet
                        return temperature
                        successtemp = True
                        break
                except pigpio.error as e:
                        errors.append(1)
                        time.sleep(1)
        if not successtemp:
                print "restart program, too many I2C write failure"
                        
def read_humidity():
        successhum = False
        exception_hum = None
        for _ in range(5):
                try:
                        handle = pi.i2c_open(bus, addr) # open i2c bus
                        pi.i2c_write_byte(handle, rdhumi) # send read humi command
                        time.sleep(0.055) # readings take up to 50ms, give it some time
                        (count, byteArray) = pi.i2c_read_device(handle, 3) # vacuum up 
                        												   # those bytes
                        pi.i2c_close(handle) # close the i2c bus
                        h1 = byteArray[0] # most significant byte msb
                        h2 = byteArray[1] # least significant byte lsb
                        humi_reading = (h1 * 256) + h2 # combine both bytes into one 
                        							   # big integer
                        humi_reading = math.fabs(humi_reading) # I'm an idiot and can't 
                        					# figure out any other way to make it a float
                        uncomp_humidity = ((humi_reading / 65536) * 125 ) - 6 # formula 
                        												# from datasheet
                        # to get the compensated humidity we need to read the temperature
                        temperature = read_temperature()
                        humidity = ((25 - temperature) * -0.15) + uncomp_humidity
                        return humidity
                        successhum = True
                        break
                except pigpio.error as e:
                        errors.append(2)
                        time.sleep(1)
        if not successhum:
                print "restart program, too many I2C write failures"
                
# Setup ADC and SPI Communication
def bitstring(n):
    s = bin(n)[2:]
    return '0'*(8-len(s)) + s

def read(adc_channel=0, spi_channel=0):
    conn = spidev.SpiDev(0, spi_channel)
    conn.max_speed_hz = 1200000 # 1.2 MHz SPI
    cmd = 128
    if adc_channel:
        cmd += 32
    reply_bytes = conn.xfer2([cmd, 0])
    reply_bitstring = ''.join(bitstring(n) for n in reply_bytes)
    reply = reply_bitstring[5:15]
    return int(reply, 2) / 2**10

if __name__ == '__main__':

#Test Parameters/Constants
        samples = 5              # total number of samples
        sPeriod = 1              # time between samples of soil moisture, s
        vPeriod = 7              # valve open duration for watering, s
        hPeriod = 1.5            # valve open duration for humidity control, s
        tWait = 60*5             # wait time before test is repeated, s
        VFR = 0.092              # valve flow rate, L/s
        tempref = 23             # reference/desired temperature, C
        humref = 35              # reference/desired humidity, %
        fanflag = 0              # initialize fan flag, start with the fan off
        
# Array Initialization
fanon = []              # how many times the fans turned on
fanoff = []             # how many times the fans turned off
soilcondavgar = []      # average soil conductance array
waterar = []            # how many times the valve was opened for watering
waterTot = []           # total water applied to the system
tempar = []             # temerature reading array
humar = []              # humidity reading array
humvar = []             # how many times the valve was opened for humidity control routine
timetemp = []           # time of each temperature reading
timehum = []            # time of each humidity reading
timehumv = []           # time of each valve opening for humidity control routine
timefanon = []          # time of each fan on 
timefanoff = []         # time of each fan off
timesoilcond = []       # time of each soil conductivity measurement
timewater = []          # time of each valve opening for watering
try:
        while True:
                scount = 0               # initialize sampling counter
                tcount = 0               # counter for checking temperature while waiting
                soilcond = []            # initialize measured voltage values array 
                						 # (for averaging)
                # Temperature Control Routine    
                htu_reset                    # reset Temp/Hum sensor
                temp = read_temperature()    # read temperature
                tempar.append(temp)
                now = datetime.datetime.now()
                timetemp.append(now)
                if (temp > tempref and fanflag == 0):
                        pifacedigital.relays[1].turn_on() # turn on fan
                        fanon.append(1)
                        fanflag = 1                        # set flag to 1
                        now = datetime.datetime.now()
                        timefanon.append(now)
                if (temp <= tempref and fanflag == 1):
                        pifacedigital.relays[1].turn_off() # turn off fan
                        fanoff.append(1)
                        fanflag = 0                        # set flag to 0
                        now = datetime.datetime.now()
                        timefanoff.append(now)
                # Data Collection for Soil Conductance and Humidity
                hum = read_humidity()           # read humidity
                humar.append(hum)
                now = datetime.datetime.now()
                timehum.append(now)
                GPIO.output(4, GPIO.HIGH)
                while (scount != samples):
                    Vin = read()                # take sample of voltage
                    soilcond.append(Vin)             # add sample to end of list
                    scount = scount + 1         # increment counter
                    time.sleep(sPeriod)
                GPIO.output(4, GPIO.LOW)
                soilcond = [soilcond*3.3 for soilcond in soilcond]
                # Watering Routine
                soilcondavg = np.average(soilcond)
                soilcondavgar.append(soilcondavg)
                now = datetime.datetime.now()
                timesoilcond.append(now)
                if (soilcondavg > 2.1):
                    #print "Too Wet!"
                    # Temperature Control Routine
                    while (tcount != 11):
                        time.sleep(tWait)            
                        tcount = tcount + 1
                        htu_reset                    # reset Temp/Hum sensor
                        temp = read_temperature()    # read temperature
                        tempar.append(temp)
                        now = datetime.datetime.now()
                        timetemp.append(now)
                        if (temp > tempref and fanflag == 0):
                                pifacedigital.relays[1].turn_on() # turn on fan
                                fanon.append(1)
                                fanflag = 1
                                now = datetime.datetime.now()
                                timefanon.append(now)
                        if (temp <= tempref and fanflag == 1):
                                pifacedigital.relays[1].turn_off() # turn off fan
                                fanoff.append(1)
                                fanflag = 0                        # set flag to 0
                                now = datetime.datetime.now()
                                timefanoff.append(now)
                elif (soilcondavg < 1):
                    water = VFR*vPeriod                   # calculate total applied water
                    waterTot.append(water)
                    waterar.append(water)
                    now = datetime.datetime.now()
                    timewater.append(now)
                    #print "Water Applied: %0.3f L" % water # print total applied water, L
                    pifacedigital.relays[0].turn_on()       # open water valve
                    time.sleep(vPeriod)                     # activated duration 
                    pifacedigital.relays[0].turn_off()      # close valve
                    time.sleep(120)                # wait 2 minutes to conduct test 
                    							   # again (to let soil absorb water)
                else:
                    #print "Wait to Water"
                    # Humidity Control Routine
                    if (hum < humref and temp > 23.5):
                            water = VFR*hPeriod           # calculate total applied water
                            waterTot.append(water)
                            #print "Water Applied: %0.3f L" % water # print total applied 
                            										# water, L
                            pifacedigital.relays[0].turn_on()       # open water valve
                            time.sleep(hPeriod)                     # activated duration 
                            pifacedigital.relays[0].turn_off()      # close valve
                            humvar.append(water)
                            now = datetime.datetime.now()
                            timehumv.append(now)
                    # Temperature Control Routine
                    while (tcount != 5):
                        time.sleep(tWait)
                        tcount = tcount + 1
                        htu_reset                    # reset Temp/Hum sensor
                        temp = read_temperature()    # read temperature
                        tempar.append(temp)
                        now = datetime.datetime.now()
                        timetemp.append(now)
                        if (temp > tempref and fanflag == 0):
                                pifacedigital.relays[1].turn_on() # turn on fan
                                fanon.append(1)
                                fanflag = 1
                                now = datetime.datetime.now()
                                timefanon.append(now)
                        if (temp <= tempref and fanflag == 1):
                                pifacedigital.relays[1].turn_off() # turn off fan
                                fanoff.append(1)
                                fanflag = 0                        # set flag to 0
                                now = datetime.datetime.now()
                                timefanoff.append(now)

except KeyboardInterrupt:                       # exit script with ctrl-c
        pifacedigital.relays[0].turn_off()
        pifacedigital.relays[1].turn_off()
        GPIO.cleanup()
        
# Total Figures
print "number of temperature measurements = %d" %len(tempar)
print "number of humidity measurements = %d" %len(humar)
print "number of saved average soil conduction measurements saved = %d" %len(soilcondavgar)
print "number of times the fans switched on = %d" %len(fanon)
print "number of times the fans switched off = %d" %len(fanoff)
print "total water applied = %0.3f L" %sum(waterTot)
print "I2C errors = %d" %len(errors)

# Save Arrays to File Using Pickle
pickle.dump(soilcondavgar, open("SoilCondAvgAr.p", "wb"))
pickle.dump(timesoilcond, open("TimeSoilCond.p", "wb"))
pickle.dump(waterTot, open("WaterTot.p", "wb"))
pickle.dump(waterar, open("WaterAr.p", "wb"))
pickle.dump(timewater, open("TimeWater.p", "wb"))
pickle.dump(tempar, open("TempAr.p", "wb"))
pickle.dump(timetemp, open("TimeTemp.p", "wb"))
pickle.dump(humar, open("HumAr.p", "wb"))
pickle.dump(timehum, open("TimeHum.p", "wb"))
pickle.dump(humvar, open("HumvAr.p", "wb"))
pickle.dump(timehumv, open("TimeHumv.p", "wb"))
pickle.dump(timefanon, open("TimeFanOn.p", "wb"))
pickle.dump(timefanoff, open("TimeFanOff.p", "wb"))

# Figure 1 - Average Soil Conductance
#samplesArr = np.arange(0, len(soilcondavgar), 1)
plt.figure(1)
plt.plot(timesoilcond,soilcondavgar,'bx')
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
plt.gcf().autofmt_xdate()
plt.title('Soil Conductance Throughout the Day and Night')
plt.ylabel('Voltage, V')
plt.xlabel('Time, hour:min:sec')   

# Figure 2 - Water Added
if (len(waterar) > 0):
        plt.figure(2)
        plt.plot(timewater,waterar,'bx')
        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
        plt.gcf().autofmt_xdate()
        plt.title('Amount of Water Added Vs Number of Waterings')
        plt.ylabel('Water Added, L')
        plt.xlabel('Time, hour:min:sec')

#Figure 3 - Temperature
#samplestemp = np.arange(0, len(tempar), 1)
plt.figure(3)
plt.plot(timetemp,tempar,'bx')
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
plt.gcf().autofmt_xdate()
plt.title('Temperature Throughout the Day and Night')
plt.ylabel('Temperature, C')
plt.xlabel('Time, hour:min:sec')

# Figure 4 - Humidity
#sampleshum = np.arange(0, len(humar), 1)
plt.figure(4)
plt.plot(timehum,humar,'bx')
plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
plt.gcf().autofmt_xdate()
plt.title('Humidity Throughout the Day And Night')
plt.ylabel('Humidity, %')
plt.xlabel('Time, hour:min:sec')

# Figure 5 - Fan On
if (len(fanon) > 0):
        plt.figure(5)
        plt.plot(timefanon,fanon,'bx')
        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
        plt.gcf().autofmt_xdate()
        plt.title('Number of Times the Fan Turned On')
        plt.ylabel('Fan On, (1)')
        plt.xlabel('Time, hour:min:sec')

# Figure 6 - Fan Off
if (len(fanoff) > 0):
        plt.figure(6)
        plt.plot(timefanoff,fanoff,'bx')
        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
        plt.gcf().autofmt_xdate()
        plt.title('Number of Times the Fan Turned Off')
        plt.ylabel('Fan Off, (1)')
        plt.xlabel('Time, hour:min:sec')

# Figure 7 - Humidity Valve Openings
if (len(humvar) > 0):
        plt.figure(7)
        plt.plot(timehumv,humvar,'bx')
        plt.gca().xaxis.set_major_formatter(mdates.DateFormatter('%X'))
        plt.gcf().autofmt_xdate()
        plt.title('Number of Times the Valve Opened for Humidity Control')
        plt.ylabel('Valve Open, (1)')
        plt.xlabel('Time, hour:min:sec')
plt.show()
