from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import numpy as np
from matplotlib import pyplot as plt

# vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=57600)

# obstacle = (  -35.36181360, 149.16664259 )
obstacle = (  28.75336994, 77.11687245  )
point1 = LocationGlobalRelative(28.75381144, 77.11684644, 10)
point2 = LocationGlobalRelative( 28.75362759, 77.11547868, 10  )
goal = (  28.75299374, 77.11685583 ) 

# lat_goal =  -35.36141012
# lon_goal = 149.16701319   
lat_goal = goal[0]
lon_goal = goal[1]
# lat_ob =   -35.36204534  
# lon_ob =   149.16650916
lat_ob = obstacle[0]
lon_ob = obstacle[1]      
k_att = 0.02
k_rep1 = 20
a1 = 8
b1 = 15

def goto(loc):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(loc)
        while True:
          print("hello",distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,loc.lat,loc.lon))
          if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,loc.lat,loc.lon) <= 0.95*2:
            break

def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    #print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def theta_calculator(lat,lon):
  d = lon-vehicle.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vehicle.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vehicle.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def theta_calculator_returns(lat,lon):
  d = lon-149.16523606
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(-35.36326490))*math.sin(math.radians(lat))-math.sin(math.radians(-35.36326490))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle

    vehicle.send_mavlink(msg)
    time.sleep(0.1)   

def distance_calculation(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):


    """

    This function returns the distance between two geographiclocations using
    the haversine formula.

    Inputs:
        1.  homeLattitude          -   Home or Current Location's  Latitude
        2.  homeLongitude          -   Home or Current Location's  Longitude
        3.  destinationLattitude   -   Destination Location's  Latitude
        4.  destinationLongitude   -   Destination Location's  Longitude

    """

    # Radius of earth in metres
    R = 6371e3

    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)
    rlat2, rlon2 = destinationLattitude * (math.pi/180), destinationLongitude * (math.pi/180)
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)

    # Haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Distance (in meters)
    distance = R * c

    return distance

def mpf_grad(x_o,y_o,a,b,theta1,k_rep,x_goal,y_goal):
    
    thetu = theta_calculator(x_goal,y_goal)
    thets = theta_calculator(x_o, y_o)
    d = distance_calculation(x_o,y_o,vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    dist = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

    S0 = math.sin(math.radians(thetu))
    C0 = math.cos(math.radians(thetu))
    S1 = math.sin(math.radians(thets))
    C1 = math.cos(math.radians(thets))
    k = (C0*S1-S0*C1)
    
    x = d*math.sin(math.radians(thets))
    y = d*math.cos(math.radians(thets))

  

    # print('theta: ', theta1)

    fx = -(k_rep*((2*math.cos(theta1)*(x*math.cos(theta1) - y*math.sin(theta1)))/a**2 + (2*math.sin(theta1)*(y*math.cos(theta1) + x*math.sin(theta1)))/b**2))/((x*math.cos(theta1) - y*math.sin(theta1))**2/a**2 + (y*math.cos(theta1) + x*math.sin(theta1))**2/b**2 + 1)**2
    fy = -(k_rep*((2*math.cos(theta1)*(y*math.cos(theta1) + x*math.sin(theta1)))/b**2 - (2*math.sin(theta1)*(x*math.cos(theta1) - y*math.sin(theta1)))/a**2))/((x*math.cos(theta1) - y*math.sin(theta1))**2/a**2 + (y*math.cos(theta1) + x*math.sin(theta1))**2/b**2 + 1)**2

    # if d<=100:
        # print("if1 dist: ", d)
        # if np.sign(k)==0:
            # print("if2 ", k)
            # ax = fx+fy
            # ay = fx-fy
        # else:
            # print("else1:", np.sign(k))
            # ax = fx-np.sign(k)*fy
            # ay = fx+np.sign(k)*fy
    # else:
        # print("dist: ", d)
        # print("else2: ", k)
        # ax = fx+fy
        # ay = fx-fy        

    # ax = fx+fy
    # ay = fx-fy

    # print("fx: ",fx)
    # print("fy: ",fy)
    # print("fx + fy = ", ay)
    # print("fx - fy = ", ax)

    if np.sign(k)==0:
            print("if2 ", k)
            ax = fx-fy
            ay = fx+fy
    else:
            print("else1:", np.sign(k))
            ax = fx+np.sign(k)*fy
            ay = fx-np.sign(k)*fy

    v_hdg = [ax, ay]

    dl.append(d)

    return v_hdg


dist = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

i = 0
xpo = []
ypo = []
v_x = 1
v_y = 1

arm_and_takeoff(10)

goto(point1)

time.sleep(5)

tacc = time.time()
al = []
dl = []
cumma = 0
lat_gps1 = []
lon_gps1 = []
altitude1 = []
vel_x = 0
vel_y = 0

while True:
    j = theta_calculator(lat_goal, lon_goal)
    phi = theta_calculator(lat_ob, lon_ob)
    vx1 = -v_x
    vy1 = -v_y
    angle = math.atan2(v_y,v_x)
    dx = dist*math.sin(math.radians(j))
    dy = dist*math.cos(math.radians(j))
    xd = dist*math.cos(math.radians(phi))
    yd = dist*math.sin(math.radians(phi))  
    phiii = (xd*vx1 + yd*vy1)/((math.sqrt(xd**2+yd**2)*math.sqrt(vx1**2+vy1**2)))
    phi_dash = math.radians(90-phi)

    # print(math.sin(3.14*phiii))

    k_rep11 = k_rep1*(math.sin(math.radians((3.14*phiii/2))) + 1)

    if k_rep11 < 0:
        k_rep11 = 0

    # print(k_rep11)

    #pos = np.zeros((2,1))

    g = mpf_grad(lat_ob, lon_ob, a1, b1, angle, k_rep1, lat_goal, lon_goal)

    v1 = [k_att*dx, k_att*dy]
    v =  [g[0]+k_att*dx,g[1]+k_att*dy]

    print("v = ", v)
    # print("vbef", v1)
    # print("vaf", v)


    v_x = v[0]
    v_y = v[1]

    r = math.sqrt(v_x*v_x + v_y*v_y)

    c = v_x/r
    s = v_y/r

    vx = 5*c
    vy = 5*s

    print("VX:", vx,"VY:", vy)

    d2 = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)
    print(d2)
    if d2 <= 2:
      send_global_velocity(0,0,0,1)
      break
    else:
      send_global_velocity(vy,vx,0,1)

    home_lat = -35.36326490
    home_lon  = 149.16523606
    dd = distance_calculation(home_lat, home_lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    gg = theta_calculator_returns(vehicle.location.global_relative_frame.lat, vehicle.location.global_frame.lon)
    df = distance_calculation(home_lat, home_lon, lat_ob, lon_ob)
    gf = theta_calculator_returns(lat_ob,lon_ob)

    velo = vehicle.velocity

    a_x = (velo[0]-vel_x)/(time.time()-tacc)
    a_y = (velo[1]-vel_y)/(time.time()-tacc)


    vel_x = velo[0]
    vel_y = velo[1]
    tacc = time.time()

    axeler = math.sqrt(a_x**2 + a_y**2)
    if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_ob,lon_ob) <=30:
      al.append(axeler)
    
    cumma = cumma + axeler
    lat_gps1.append(vehicle.location.global_relative_frame.lat)
    lon_gps1.append(vehicle.location.global_relative_frame.lon)
    altitude1.append(vehicle.location.global_relative_frame.alt + 200)

    # print("theta start:", gg)
    # print("distttttaaaa: ", dd)

    # print("x hai:",dd*math.sin(math.radians(gg)))

    xpo.append(dd*math.sin(math.radians(gg)))
    ypo.append(dd*math.cos(math.radians(gg)))
    time.sleep(0.1)

time.sleep(5)

goto(point2)


xpo.append(df*math.sin(math.radians(gf)))
ypo.append(df*math.cos(math.radians(gf)))

# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()


print("minimum distance: ", min(dl))

print("About to Land")

time.sleep(5)

vehicle.mode = VehicleMode("LAND")

print("Landed.")

print("max accel: ", max(al))
print("cummulative acceleration: ", cumma)
print(al)

# f = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

# print("accuracy :", f) 
# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()

f = open('flight_copter.kml', 'w')

#Writing the kml file.
f.write("<?xml version='1.0' encoding='UTF-8'?>\n")
f.write("<kml xmlns='http://earth.google.com/kml/2.2'>\n")
f.write("<Document>\n")
f.write("<Placemark>\n")
f.write("   <name>flight</name>\n")
f.write("   <LineString>\n")
f.write("       <extrude>1</extrude>\n")
f.write("       <altitudeMode>absolute</altitudeMode>\n")
f.write("       <coordinates>\n")
for i in range(0,len(altitude1),10):  #Here I skip some data
    f.write("        "+str(lon_gps1[i]) + ","+ str(lat_gps1[i]) + "," + str(altitude1[i]) +"\n")    
f.write("       </coordinates>\n")
f.write("   </LineString>\n")
f.write("</Placemark>\n")
f.write("</Document>")
f.write("</kml>\n")
f.close()

vehicle.close()