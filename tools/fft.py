import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse

#---------------------------------------------------------------------------------
# When running this script, provide name of .csv file to be parsed as an argument, e.g.
#
# python fft.py filename.csv
#
# To choose windowing function, provide the following flag "-w" or "--windowing" with a value from 0 to 3.
# Default value is 0, which corresponds to no windowing. 1-3 are in order for: hanning, hamming, blackman.
# example for hamming window:
#
# python fft.py filename.csv -w=2
#---------------------------------------------------------------------------------

parser = argparse.ArgumentParser()
parser.add_argument("fname", help="name of .csv file to parse")
parser.add_argument("-w","--windowing", default=0, type=int, help="windowing mode, default=0:no windowing, 1:hanning, 2:hamming, 3:blackman")
parser.add_argument("-o", "--output", default="", type=str, help="output computed fft to a file")
args = parser.parse_args()

fname = args.fname
range_res = 0.39

df = pd.read_csv(fname)
data = df.to_numpy()

print(np.shape(data))

#perform signal windowing
def windowing_func(x):
    match x:
        case 0:
            print("no windowing")
            return(1)
        case 1:
            print("hanning window")
            return(np.hanning(data.shape[0]))
        case 2:
            print("hamming window")
            return(np.hamming(data.shape[0]))
        case 3:
            print("blackman window")
            return(np.blackman(data.shape[0]))
        case _:
            print("invalid windowing argument value")
            return(1)
        
window = windowing_func(args.windowing)
windowed_signal = window * data[:,0]

#perform rfft (this is needed as opposed to fft to prevent mirroring in case of real input data)
fft_result = np.fft.rfft(windowed_signal)

# if -o was supplied, write resulting array to a file and exit
# TODO: this should probably format it into some sensible csv format 
if args.output:
    with open(args.output, "w") as f:
              f.write(np.array2string(fft_result,separator=","))
              quit()
              
              


#compute x axis range values
ranges = np.arange(fft_result.shape[0]) * range_res

plt.figure()
plt.plot(ranges, np.abs(fft_result))
plt.title("Range profile")
plt.xlabel("Range (m)")
plt.ylabel("Corresponding frequency component intensity")
plt.grid(True)
plt.show()
