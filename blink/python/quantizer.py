from machine import Pin, ADC, PWM
from time import sleep
import time
import math

#CONSTANTS
adc_max = 64535
adc_step_size = adc_max / 7
volt_per_semitone = 1/12
frequencies = []
voltages = []
freq_a0 = 27.5
num_piano_keys = 88


#PINS
# For counting/displaying chosen scale
led1 = Pin(20, Pin.OUT)
led2 = Pin(19, Pin.OUT)
led4 = Pin(18, Pin.OUT)

pot = ADC(Pin(26))
osc = ADC(Pin(27))

pwm = PWM(Pin(15), freq=440, duty_u16=int(adc_max / 3))


def generateFrequencies():
    freq = freq_a0
    result = [freq]
    
    for x in range(0, 120):
        freq = freq_a0 * pow(pow(2, 1/12), x)
        result.append(freq)
        print(freq)
    return result
        
        
def generateVoltages():
    result = [0.0]
    for v in range(-5, 5):
        for x in range(12):
            volt = x * volt_per_semitone + v
            result.append(volt)
    return result
        
def set_counter_leds(num):
    if num & 1 > 0:
        led1.value(1)
    else:
        led1.value(0)
    if num & 2 > 0:
        led2.value(1)
    else:
        led2.value(0)
    if num & 4 > 0:
        led4.value(1)
    else:
        led4.value(0)

def read_pot():
    result = int(pot.read_u16() // adc_step_size)
    #print("Counter: " + str(result))
    return result

def read_osc():
    result = osc.read_u16()
    print("Osc: " + str(round(result / adc_max, 2)))
    return result

##define range 0V = minvolt, 5V = maxvolt
#find the nearest frequency from the frequencies list
def quantizeFrequency(freq, frequencies):
    n = len(frequencies)
    if (freq <= frequencies[0]):
        return frequencies[0]
    elif (freq >= frequencies[n - 1]):
        return frequencies[n - 1]
    l = 0 #lower limit
    u = n - 1 #upper limit
    while (u - l > 1):
        m = (u + l) >> 1 # compute a midpoint with a bitshift
        if (freq == frequencies[m]):
            return frequencies[m]
        elif (freq > frequencies[m]):
            l = m
        else:
            u = m
    
    return frequencies[l]
    


def main():
    # frequencies = generateFrequencies()
    # voltages = generateVoltages()

    #for f in range(220, 235): #any Hz between 220 and 235 should return 220Hz
    #    print(str(f) + " => " + str(quantizeFrequency(f, frequencies)))
        
    #while True:
    for i in range(100):
        # set_counter_leds(read_pot())
        read_osc()
        # for duty in range(adc_max):
        #     pwm.duty_u16(duty)
        #     sleep(0.0001)
        # for duty in range(adc_max, 0, -1):
        #     pwm.duty_u16(duty)
        #     sleep(0.0001)
        time.sleep_us(1)

# MAIN LOOP
main()
