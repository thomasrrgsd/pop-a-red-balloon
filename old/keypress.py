#################################
#				#
# William T. Jarratt		#
# 			       	#
#				#
################################# 

import sys
import select
import time

while (1):
  readable, writable, exceptional = select.select([sys.stdin], [], []) # Blocking.
  
  if not (readable or writable or exceptional):
    print('We timed out.')
  else:
    print('We got a key press: ')
    print(sys.stdin.readline().strip())
