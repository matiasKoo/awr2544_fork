import socket
import numpy as np
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP, UDP_PORT))

dt = np.dtype(np.int16)
dt = dt.newbyteorder('<')

data_complex = np.zeros(128, dtype=np.complex128) #TODO:length of this should be derived from params

print("UDP receiver starting...")

# I'm about to write hypernaive logic
#TODO: the packets could be of any format, this script should be made to handle any dimensions
#TODO: also maybe implement some logic for the initial test pkt? Necessary?
while True:
    data,addr = sock.recvfrom(1024)
    data = np.frombuffer(data,dtype=dt) #512 datapoints per pkt, fft is mirrored
    data = data[:256] #dropping mirrored fft result, temporarily needed

    data_complex = data[::2] + (1j*data[1::2])
    print(data_complex)
    print(np.shape(data_complex))


    time.sleep(0.1)