import socket
import math
import time


teleplotAddr = ("127.0.0.1", 47269)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def sendTelemetry(name, value):
    now = time.time() * 1000
    msg = name + ":" + str(now) + ":" + str(value) + "|g"
    print(msg)
    sock.sendto(msg.encode(), teleplotAddr)

i=0
while i < 1000:
	
	sendTelemetry("rool", 1)
	sendTelemetry("cos", 2)

	i+=0.1
	time.sleep(0.01)