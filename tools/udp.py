import socket
import numpy as np
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP, UDP_PORT))

dt = np.dtype(np.int16)
dt = dt.newbyteorder('<')

range_res = 0.39

f = plt.figure()
plt.ion()
plt.title("Graph")
plt.xlabel("Range (m)")
plt.ylabel("This is the y axis")
plt.grid(True)
while True:
    data,addr = sock.recvfrom(1024)

    ia = np.frombuffer(data, dtype=dt)

    real = ia[::2]
    imag = ia[1::2]

    # needs to be dtype int32 since apparently no overflow check
    real = np.square(real,dtype=np.int32)
    imag = np.square(imag,dtype=np.int32)

    out = np.add(real, imag)

    out = np.sqrt(out)

    out = out[1:127]


    ranges = np.arange(out.shape[0]) * range_res
    # clear and replot every time
    print(out.shape)
    plt.clf()
    plt.plot(ranges, out)
    plt.show()
    plt.pause(0.01)



