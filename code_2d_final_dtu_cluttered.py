from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import numpy as np
from matplotlib import pyplot as plt

# vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = connect('127.0.0.1:14552', wait_ready=True, baud=57600)
vehicle2 = connect('127.0.0.1:14562', wait_ready=True, baud=57600)

# obstacle = (  -35.36181360, 149.16664259 )

point1 = LocationGlobalRelative(28.75292667,77.11609898,10)
point2 = LocationGlobalRelative( 28.75445781,77.11602699, 10  )
pointgoalulti = LocationGlobalRelative(28.74930623,77.11878257,10)
goal1 = (  28.75212612,77.11556868  ) 
goal2 = (28.75102305,77.11681749 )
goal3 = (28.75029068,77.11793920 )
goal4 = ( 28.74952574,77.11839279 )
goalhehe = (28.75445781,77.11602699)

# obstacle = [28.75167337,77.11599121,28.75164867,77.11571265,28.75083028,77.11654754,28.75076991,77.11670247,28.75068540,77.11676444,28.75081519,77.11733250,28.75093894,77.11748399,28.75112608,77.11763891,28.75122267,77.11776630,28.75132228,77.11784893,28.75149734,77.11802451,28.75041847,77.11705497,28.75041847,77.11705497,28.75043039,77.11721044,28.75043210,77.11731149,28.75044743,77.11741838,28.75049173,77.11689367,28.75067744,77.11790616,28.75057862,77.11799944,28.74946089,77.11788348,28.74934364,77.11822206,28.74981261,77.11827043, 28.74973279,77.11853219 ]

obstacle = [28.75402975,77.11582981, 28.75385962,77.11634624,28.75364559,77.11584233,28.75338492,77.11635563 ]

# lat_goal =  -35.36141012
# lon_goal = 149.16701319   
# lat_goal = goal[0]
# lon_goal = goal[1]
# lat_ob =   -35.36204534  
# lon_ob =   149.16650916

bhai = len(obstacle)
print(bhai)

# obstacle = (-35.35206193, 149.17084008)

jkkk = 0

u = 1

for i in range(4):
    globals()["lat_ob"+str(u)] = obstacle[jkkk]
    globals()["lon_ob"+str(u)] = obstacle[jkkk + 1]
    u = u + 1
    jkkk = jkkk+2

print("ye le lat: ", lat_ob1)
print("ye le lat: ", lon_ob2)
print("ye le lat: ", lon_ob3)
print("ye le lat: ", lat_ob4)

lat_ob = obstacle[0]
lon_ob = obstacle[1]      
k_att = 0.02
k_rep1 = 30
a1 = 15
b1 = 15.49

def goto(loc):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(loc)
        while True:
          print("hello",distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,loc.lat,loc.lon))
          if distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,loc.lat,loc.lon) <= 0.95*2:
            "yo"
            break

def goto_2(loc):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle2.simple_goto(loc)
        while True:
          print("hello",distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle2.location.global_relative_frame.lon,loc.lat,loc.lon))
          if distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle2.location.global_relative_frame.lon,loc.lat,loc.lon) <= 0.95*2:
            break

def arm_and_takeoff(aTargetAltitude,vecx):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vecx.is_armable:
    print(" Waiting for vecx to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vecx.mode    = VehicleMode("GUIDED")
  vecx.armed   = True

  while not vecx.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vecx.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vecx has reached takeoff altitude
  while True:
    #print(" Altitude: ", vecx.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vecx.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
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

def theta_calculator1(lat,lon):
  d = lon-vehicle2.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vehicle2.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vehicle2.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
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

def send_global_velocity1(velocity_x, velocity_y, velocity_z, duration):
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

    vehicle2.send_mavlink(msg)
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

def mpf_grad(x_o,y_o,a,b,theta1,k_rep,x_goal,y_goal,veccc):
    
    thetu = theta_calculator(x_goal,y_goal)
    thets = theta_calculator(x_o, y_o)
    d = distance_calculation(x_o,y_o,veccc.location.global_frame.lat, veccc.location.global_frame.lon)
    dist = distance_calculation(veccc.location.global_frame.lat,veccc.location.global_frame.lon,x_goal,y_goal)

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

    return v_hdg


# dist = distance_calculation(veccc.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

i = 0
xpo = []
ypo = []
dl1 = []
dl2 = []
dl3 = []
dl4 = []
dl5 = []
dl6 = []
dl7 = []
dl8 = []
dl = []


arm_and_takeoff(10,vehicle)
arm_and_takeoff(10,vehicle2)

goto(point1)
goto_2(point2)



def goto_avoidance_integ(lat_goal,lon_goal):
    lat_goal1 = vehicle.location.global_relative_frame.lat
    lon_goal1 = vehicle.location.global_relative_frame.lon
    dist = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)
    distt = distance_calculation(vehicle2.location.global_relative_frame.lat, vehicle2.location.global_relative_frame.lon,lat_goal1,lon_goal1)

    v_x = 1
    v_y = 1
    v_x1 = 1
    v_y1 = 1
    while True:
        lat_obdy1 = vehicle2.location.global_relative_frame.lat
        lon_obdy1 = vehicle2.location.global_relative_frame.lon
        lat_obdy2 = vehicle.location.global_relative_frame.lat
        lon_obdy2 = vehicle.location.global_relative_frame.lon
        print("hello")
        j = theta_calculator(lat_goal, lon_goal)
        tt = theta_calculator1(lat_goal1, lon_goal1)
        phi = theta_calculator(lat_ob, lon_ob)
        vx1 = -v_x
        vy1 = -v_y
        angle = math.atan2(v_y,v_x)
        dx = dist*math.sin(math.radians(j))
        dy = dist*math.cos(math.radians(j))
        ddx = distt*math.sin(math.radians(tt))
        ddy = distt*math.cos(math.radians(tt))
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

        # g = mpf_grad(lat_ob, lon_ob, a1, b1, angle, k_rep1, lat_goal, lon_goal)
# 
        # v1 = [k_att*dx, k_att*dy]
        # v =  [g[0]+k_att*dx,g[1]+k_att*dy]

        # print("vbef", v1)
        # print("vaf", v)

        vl = 1
        for f in range(4):
            globals()["g"+str(vl)] = mpf_grad(globals()["lat_ob"+str(vl)],globals()["lon_ob"+str(vl)],a1,b1,angle,k_rep1,lat_goal,lon_goal,vehicle)
            globals()["g2"+str(vl)] = mpf_grad(globals()["lat_ob"+str(vl)],globals()["lon_ob"+str(vl)],a1,b1,angle,k_rep1,lat_goal,lon_goal,vehicle2)
            g_dyna1 = mpf_grad(lat_obdy1,lon_obdy1,a1,b1,angle,k_rep1,lat_goal,lon_goal,vehicle)
            g_dyna2 = mpf_grad(lat_obdy2,lon_obdy2,a1,b1,angle,k_rep1,lat_goal1,lon_goal1,vehicle2)
            vl= vl + 1
    
        sl = 1
        v = [k_att*dx+g_dyna1[0], k_att*dy+g_dyna1[1]]
        v2 = [k_att*ddx+g_dyna2[0], k_att*ddy+g_dyna2[1]]
        for dsn in range(4):
            v[0] = v[0] + globals()["g"+str(sl)][0]
            v[1] = v[1] + globals()["g"+str(sl)][1]
            v2[0] = v2[0] + globals()["g2"+str(sl)][0]
            v2[1] = v2[1] + globals()["g2"+str(sl)][1]
            sl = sl + 1


        v_x = v[0]
        v_y = v[1]
        v_x1 = v2[0]
        v_y1 = v2[1]

        r = math.sqrt(v_x*v_x + v_y*v_y)

        c = v_x/r
        s = v_y/r

        r1 = math.sqrt(v_x1*v_x1 + v_y1*v_y1)

        c1 = v_x1/r1
        s1 = v_y1/r1


        vx = 10*c
        vy = 10*s
        vx1 = 10*c1
        vy1 = 10*s1


        print("VX:", vx,"VY:", vy)

        d2 = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)
        print(d2)
        if d2 <= 2:
          send_global_velocity(0,0,0,1)
          send_global_velocity1(0,0,0,1)
          break
        else:
          send_global_velocity(vy,vx,0,1)
          send_global_velocity1(vy1,vx1,0,1)

        home_lat = -35.36326490
        home_lon  = 149.16523606
        dd = distance_calculation(home_lat, home_lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        gg = theta_calculator_returns(vehicle.location.global_relative_frame.lat, vehicle.location.global_frame.lon)
        df = distance_calculation(home_lat, home_lon, lat_ob, lon_ob)
        gf = theta_calculator_returns(lat_ob,lon_ob)

        dl1.append(distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[0],obstacle[1]))
        dl2.append(distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[2],obstacle[3]))
        dl3.append(distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[4],obstacle[5]))
        dl4.append(distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[6],obstacle[7]))
        dl5.append(distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[0],obstacle[1]))
        dl6.append(distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[2],obstacle[3]))
        dl7.append(distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[4],obstacle[5]))
        dl8.append(distance_calculation(vehicle2.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,obstacle[6],obstacle[7]))


        # print("theta start:", gg)
        # print("distttttaaaa: ", dd)

        # print("x hai:",dd*math.sin(math.radians(gg)))

        xpo.append(dd*math.sin(math.radians(gg)))
        ypo.append(dd*math.cos(math.radians(gg)))



goto_avoidance_integ(goalhehe[0],goalhehe[1])

# goto_avoidance_integ(goal2[0],goal2[1])
# goto_avoidance_integ(goal3[0],goal3[1])
# goto_avoidance_integ(goal4[0],goal4[1])


# xpo.append(df*math.sin(math.radians(gf)))
# ypo.append(df*math.cos(math.radians(gf)))

# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()



print("About to Land")

time.sleep(5)

vehicle.mode = VehicleMode("LAND")

print("Landed.")

# f = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

# print("accuracy :", f) 
# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()

vehicle.close()