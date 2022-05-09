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

udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Video processing loop. Exit after 15 seconds.

start_time = datetime.datetime.utcnow() # When script started.
time_elapsed = datetime.datetime.utcnow() - start_time
heart_beat = datetime.datetime.utcnow()

format = "%(asctime)s: %(message)s"
#logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
#logging.info("OCV  : Entering video processing loop.")

x = 1440
y = 2100

while (time_elapsed.total_seconds() < 15):
  
  time_elapsed = datetime.datetime.utcnow() - start_time
  heart_elapsed = datetime.datetime.utcnow() - heart_beat
  
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
    udp_socket.sendto(my_bytes, ("127.0.0.1", 20001))

#udp_socket.sendto(done_str, ("127.0.0.1", 20001))

time.sleep(1)
udp_socket.close()
