import socket
import numpy as np
import matplotlib.pyplot as plt
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP, UDP_PORT))

dt = np.dtype(np.int16)
dt = dt.newbyteorder('<')

range_res = 0.39

plt.ion()
plt.title("Graph")
plt.xlabel("Range (m)")
plt.ylabel("This is the y axis")
plt.grid(True)

out = np.empty((4,256))

fig, axs = plt.subplots(4, 1, figsize=(10, 8))
lines = []

# Initial plot
for i in range(4):
    line, = axs[i].plot(out[i])
    lines.append(line)
    axs[i].set_title(f"Plot {i+1}")

plt.tight_layout()
plt.show()

while True:
    # super optimized home-made hand-crafted organic
    # unrolled loops sourced from a local keyboard 
    # belonging to a programmer in your community
    data1,addr = sock.recvfrom(1024)
    data2,addr = sock.recvfrom(1024)
    data3,addr = sock.recvfrom(1024)
    data4,addr = sock.recvfrom(1024)

    data1 = np.frombuffer(data1,dtype=dt)
    data2 = np.frombuffer(data2,dtype=dt)
    data3 = np.frombuffer(data3,dtype=dt)
    data4 = np.frombuffer(data4,dtype=dt)

    ia = np.empty((4,512),dtype=dt)

    ia[0] = data1
    ia[1] = data2
    ia[2] = data3
    ia[3] = data4


    real = ia[:,::2]
    imag = ia[:,1::2]


    # needs to be dtype int32 since apparently no overflow check
    real = np.square(real,dtype=np.int32)
    imag = np.square(imag,dtype=np.int32)

    out = real + imag
    out = np.sqrt(out)

    out = out[:,1:127]

    for i in range(4):
        lines[i].set_ydata(out[i])
        lines[i].set_xdata(np.arange(len(out[i]))*range_res)  # update x if needed
        axs[i].relim()
        axs[i].autoscale_view()

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.1)  # small pause to make it less hayai~




