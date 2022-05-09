#################################
#				#
# William T. Jarratt		#
# 			       	#
#				#
################################# 

import logging
import select
import socket
import sys
import datetime
import time


# Initialization
heart_str = str.encode("Thump Thump")
done_str = str.encode("Done")

socket_info2 = ("127.0.0.1", 20002)

udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_socket.bind(socket_info2)

# Video processing loop. Exit after 15 seconds.

start_time = datetime.datetime.utcnow() # When script started.
time_elapsed = datetime.datetime.utcnow() - start_time
heart_beat = datetime.datetime.utcnow()

format = "%(asctime)s: %(message)s"
#logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
#logging.info("OCV  : Entering video processing loop.")

x = 65535
y = 65535
z = 65535

while (True):
  
  time_elapsed = datetime.datetime.utcnow() - start_time
  heart_elapsed = datetime.datetime.utcnow() - heart_beat
  
  if(time_elapsed.total_seconds() > 7):
    x = 2100
    y = 1440
    z = 49

  if (heart_elapsed.total_seconds() > 3):
    #logging.info("Main : Thump thump.")
    #udp_socket.sendto(heart_str, ("127.0.0.1", 20001))
    heart_beat = datetime.datetime.utcnow()
    my_bytes = bytearray()
    xlsb = (x & 0x00FF)
    xmsb = (x & 0xFF00) >> 8
    my_bytes.append(xmsb)
    my_bytes.append(xlsb)
    ylsb = (y & 0x00FF)
    ymsb = (y & 0xFF00) >> 8
    my_bytes.append(ymsb)
    my_bytes.append(ylsb)
    zlsb = (z & 0x00FF)
    zmsb = (z & 0xFF00) >> 8
    my_bytes.append(zmsb)
    my_bytes.append(zlsb)
    udp_socket.sendto(my_bytes, ("127.0.0.1", 20001))

    #message, address = udp_socket.recvfrom(1024)
    #my_bytes = bytearray(message)
    #x2 = 0  | (my_bytes[0] << 8)
    #x2 = x2 | (my_bytes[1])
    #y2 = 0  | (my_bytes[2] << 8)
    #y2 = y2 | (my_bytes[3]) 
    #z2 = 0  | (my_bytes[4] << 8)
    #z2 = z2 | (my_bytes[5])
    #print("Received a response.")
    #print("(x, y, diameter) = (%d, %d, %d)" % (x2, y2, z2))
    #break

    if((x == 2100) and (y == 1440)):
      break


#udp_socket.sendto(done_str, ("127.0.0.1", 20001))

time.sleep(1)
udp_socket.close()
