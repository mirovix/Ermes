import numpy as np 
import matplotlib.pyplot as plt
from numpy.lib.npyio import loadtxt
#r3_122Hz_208_256_6step.txt
#r_30Hz_0_256_16step.txt"
run = "r"
freq = "30Hz"
min_pwm = "0"
max_pwm = "256"
steps = 16
calibration_factor = 696.0
tare = -8.245683452922070e+02

def plot_data(run = "r", freq = "30Hz", min_pwm = "0", max_pwm = "256", steps = 16, calibration_factor = 696.0, tare = 5):
    datas = "../freq_experiment/high_frequency_results/"+run+"_"+freq+"_"+min_pwm+"_"+max_pwm+"_"+str(steps)+"step.txt"

    read_data = loadtxt(datas)
    for i in range(0,steps):
        mask =  (read_data[:, 0] == i)
        mean_ = np.mean(read_data[mask][:,1])
        mean_ = mean_/calibration_factor - tare
        plt.plot(i, mean_, 'bo')
    plt.show()

#plt.scatter(read_data[:,0], read_data[:,1])
print("ciao")