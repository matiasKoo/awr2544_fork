import io
import sys
import serial
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse


try:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="port")
    args = parser.parse_args()
    
    ser = serial.Serial(port=args.port,baudrate=115200);
    
    range_res = 0.39
    
    f = plt.figure()
    plt.ion()
    plt.title("Graph")
    plt.xlabel("Range (m)")
    plt.ylabel("This is the y axis")
    plt.grid(True)
    
    
    print(f"Waiting for input on {args.port}")
    while True:
        # This blocks until a stx is received, I hope?
        # not that anything should be sent after init is done
        ser.read_until(expected=b'\x02')
    
    
        # And then read whatever is sent between STX and ETX
        x = ser.read_until(expected=b'\x03')
        
        # convert to a string 
        x = str(x, 'utf-8')
    
        # strip ETX and comma from the end
        x = x[:-2]
    
        df = pd.read_csv(io.StringIO(x), header=None)
    
        data = df.to_numpy(dtype=np.int16)
        data = np.reshape(data,shape=(512,))
    
    
        real = data[::2]
        imag = data[1::2]
    
    
        # needs to be dtype int32 since apparently no overflow check
        real = np.square(real,dtype=np.int32)
        imag = np.square(imag,dtype=np.int32)
    
    
        out = np.add(real, imag)
        out = np.sqrt(out)
    
        out = out[1:127]
    
    
        ranges = np.arange(out.shape[0]) * range_res
    
        # clear and replot every time
        plt.clf()
        plt.plot(ranges, out)
        plt.show()
        plt.pause(1)

except KeyboardInterrupt:
    sys.exit(0)







