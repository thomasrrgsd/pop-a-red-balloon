#################################
#				#
# William T. Jarratt		#
# 			       	#
#				#
################################# 

import logging
import select
import sys
import threading
import time


def thread_function(name, e):
  logging.info("Thread %s: Started", name)
  
  while (1):
    time.sleep(3)
    e.wait()
    e.clear()
    logging.info("Thread %s: Heartbeat!", name)
    e.set()

  logging.info("Thread %s: Exited", name)

def thread_function2(name, e):
  logging.info("Thread %s: Started", name)
  a=5
  while (1):
    a = a + 1;
    time.sleep(1)
    e.wait()
    e.clear()
    logging.info("Thread %s: %d", name, a)
    e.set()

  logging.info("Thread %s: Exited", name)


if __name__ == "__main__":
  format = "%(asctime)s: %(message)s"
  logging.basicConfig(format=format, level=logging.INFO,
                      datefmt="%H:%M:%S")

  logging.info("Main    : Main thread started.")

  output_flag = threading.Event() # Event flag that notifies when console can be printed too.
  output_flag.set()               # Set it now since we are not printing to the console in main.
                                  # May not want to set if we need to print here.
  print(output_flag.is_set())

  x = threading.Thread(target=thread_function, args=(1,e))
  y = threading.Thread(target=thread_function2, args=(2,e))
  y.daemon = True
  x.daemon = True
  x.start()
  y.start()
  

  logging.info("Main    : Thread finished.")
  x.join()
  y.join()
  logging.info("Main    : all done")

