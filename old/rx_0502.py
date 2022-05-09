#################################
#				#
# William T. Jarratt		#
# 			       	#
#				#
################################# 

import logging
import os
import socket
import sys


# Initialization
socket_info = ("127.0.0.1", 20001)

format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
logging.info("Navi : Drone navigation script has begun.")

proc_flag = True

udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_socket.bind(socket_info)
logging.info("Navi : Socket is open at address %s, port %d.", socket_info[0], socket_info[1])

logging.info("Navi : Now running video process.")
#os.system("python videoproc.py &")
logging.info("Navi : Video process started in background.")

while (proc_flag):
  
  message, address = udp_socket.recvfrom(1024)

  my_bytes = bytearray(message)
  x = 0 | (my_bytes[0] << 8)
  x = x | (my_bytes[1])
  y = 0 | (my_bytes[2] << 8)
  y = y | (my_bytes[3]) 
  z = 0 | (my_bytes[4] << 8)
  z = z | (my_bytes[5])
  print("(x, y, diameter) = (%d, %d, %d)" % (x, y, z))

  #print(str.decode(message))
  #logging.info("Navi : [%s : %d] %s", socket_info[0], socket_info[1], message.decode())
  #print(str.decode(address))

  #if(message.decode() == "Done"):
  #  proc_flag = False


udp_socket.close()
sys.exit(0)
