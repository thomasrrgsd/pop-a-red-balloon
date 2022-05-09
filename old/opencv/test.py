from __future__ import print_function

import 	math
import 	time
import 	sys
import  socket
import  os


socket_info = ("127.0.0.1", 20001)
udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_socket.bind(socket_info)  

print("Starting video feed.")
os.system("gnome-terminal -e \"python3.7 ./red_find_avi.py\"")
print("Giving feed a few seconds to start up.")
time.sleep(3)
count = 0

while (True):
      
      time.sleep(1)
      if(count > 10):
        break
        
      count = count + 1
      
      message, address = udp_socket.recvfrom(1024)
      my_bytes = bytearray(message)
      x = 0 | (my_bytes[0] << 8)
      x = x | (my_bytes[1])
      y = 0 | (my_bytes[2] << 8)
      y = y | (my_bytes[3])
      z = 0 | (my_bytes[4] << 8)
      z = z | (my_bytes[5])
      print("( %d, %d, %d )" % (x, y, z))
      
udp_socket.close()
