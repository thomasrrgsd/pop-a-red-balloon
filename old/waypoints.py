#
# Opens up text file that contains the waypoint list and
# converts each entry into latitude and longitude. Returns
# an array for longitude and an array for latitude.
#

import sys
import logging

def waypoints_from_file():
      
   waypointStrList = []
   longitude = []
   latitude = []
   waypointFile = open('waygrid.txt', 'r')
   
   waypointStrList = waypointFile.readlines()
   
   #print(waypointStrList)

   for i in waypointStrList:
      lineStr = i.split(', ')
      longitude.append(float(lineStr[0]))
      latitude.append(float(lineStr[1]))

   #print(longitude)
   #print(latitude)

   waypointFile.close()

   #logging.basicConfig(filename='example.log', encoding='utf-8', level=logging.DEBUG)
   #logging.info('Longitude: ')
   #logging.info(longitude)

   return longitude, latitude

#waypoints_from_file()
