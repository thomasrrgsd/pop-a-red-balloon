import socket
    
while(True): 
  print("Socket has been closed")
  socket_info = ("127.0.0.1", 20001)
  udp_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
  udp_socket.bind(socket_info)
  print("Socket has been opened")
  udp_socket.close()


