#
# Returns the latitude and longitude pairs for the waypoints
# that make up a grid sweep pattern. User can specify yaw and
# how many rows to sweep. Also can specify at which point to
# start the sweep. The drone moves left on first sweep.
# 

def generate_grid_waypoints (yaw, rows, lon, lat):
  
  LONG_INIT = lon
  LATI_INIT = lat
  GPS_1M = 1e-5
  
  longitude = []
  latitude = []

  if(yaw == 0):
    # North yaw
    for i in range(0, rows):
      longitude.append(LONG_INIT - 5*GPS_1M*i)
      longitude.append(LONG_INIT - 5*GPS_1M*i)
      if((i % 2) == 0):
        latitude.append(LATI_INIT)
        latitude.append(LATI_INIT - 50*GPS_1M)
      else:
        latitude.append(LATI_INIT - 50*GPS_1M)
        latitude.append(LATI_INIT)

  if(yaw == 180):
    # South yaw
    for i in range(0, rows):
      longitude.append(LONG_INIT + 5*GPS_1M*i)
      longitude.append(LONG_INIT + 5*GPS_1M*i)
      if((i % 2) == 0):
        latitude.append(LATI_INIT)
        latitude.append(LATI_INIT + 50*GPS_1M)
      else:
        latitude.append(LATI_INIT + 50*GPS_1M)
        latitude.append(LATI_INIT)

  if(yaw == 270):
    # West yaw
    for i in range(0, rows):
      latitude.append(LATI_INIT + 5*GPS_1M*i)
      latitude.append(LATI_INIT + 5*GPS_1M*i)
      if((i % 2) == 0):
        longitude.append(LONG_INIT)
        longitude.append(LONG_INIT - 50*GPS_1M)
      else:
        longitude.append(LONG_INIT - 50*GPS_1M)
        longitude.append(LONG_INIT)

  if(yaw == 90):
    # East yaw
    for i in range(0, rows):
      latitude.append(LATI_INIT - 5*GPS_1M*i)
      latitude.append(LATI_INIT - 5*GPS_1M*i)
      if((i % 2) == 0):
        longitude.append(LONG_INIT)
        longitude.append(LONG_INIT + 50*GPS_1M)
      else:
        longitude.append(LONG_INIT + 50*GPS_1M)
        longitude.append(LONG_INIT)


  return longitude, latitude   
