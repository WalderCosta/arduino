import socket
import sys
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.4.1', 9000)
print('connecting to %s port %s'% server_address)
sock.connect(server_address)
print('Connection made!')

try:
 while 1:
  data = sock.recv(16)
  print('%s' % data)

finally:
  print('closing socket')
  sock.close()
