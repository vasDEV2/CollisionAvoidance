from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import numpy as np
from matplotlib import pyplot as plt

# vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = connect('127.0.0.1:14552', wait_ready=True, baud=57600)
vehicle1 = connect('127.0.0.1:14562', wait_ready=True, baud=57600)

# obstacle = (  -35.36181360, 149.16664259 )
# obstacle1 = (  -35.36240860, 149.16532861  )
# obstacle2 = (  -35.36630663, 149.17260311   )

goal = ( -35.36427776,149.17629579) 
goal1 = (  -35.36606315,149.17407888  )

# lat_goal =  -35.36141012
# lon_goal = 149.16701319   


# lat_goal = goal[0]
# lon_goal = goal[1]
# lat_goal1 = goal1[0]
# lon_goal1 = goal1[1]
lat_goal = vehicle1.location.global_relative_frame.lat
lon_goal = vehicle1.location.global_relative_frame.lon
lat_goal1 = vehicle.location.global_relative_frame.lat
lon_goal1 = vehicle.location.global_relative_frame.lon


# lat_ob =   -35.36204534  
# lon_ob =   149.16650916
# lat_ob = obstacle1[0]
# lon_ob = obstacle1[1]
# lat_ob2 = obstacle2[0]
# lon_ob2 = obstacle2[1]      
k_att = 0.02
k_rep1 = 500
a1 = 8
b1 = 15
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

def arm_and_takeoffs(aTargetAltitudes):

  print(" 2 Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle1.is_armable:
    print(" Waiting for vehicle 2 to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle1.mode    = VehicleMode("GUIDED")
  vehicle1.armed   = True

  while not vehicle1.armed:
    print(" 2 Waiting for arming...")
    time.sleep(1)

  print(" 2 Taking off!")
  vehicle1.simple_takeoff(aTargetAltitudes) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    #print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
    if vehicle1.location.global_relative_frame.alt>=aTargetAltitudes*0.95: 
      print(" 2 Reached target altitude")
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

def theta_calculator(lat,lon,veccu):
  d = lon-veccu.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(veccu.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(veccu.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def theta_calculator1(lat,lon):
  d = lon-vehicle1.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vehicle1.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vehicle1.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
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

    vehicle1.send_mavlink(msg)
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
    d = distance_calculation(x_o,y_o,vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    dist = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_goal,lon_goal)

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
            print("if :", np.sign(k))
            ax = fx-fy
            ay = fx+fy
    else:
            print("else :", np.sign(k))
            ax = fx+np.sign(k)*fy
            ay = fy-np.sign(k)*fx

    v_hdg = [ax, ay]

    return v_hdg

def mpf_grad1(x_o,y_o,a,b,theta1,k_rep,x_goal,y_goal):
    
    thetu = theta_calculator1(x_goal,y_goal)
    thets = theta_calculator1(x_o, y_o)
    d = distance_calculation(x_o,y_o,vehicle1.location.global_relative_frame.lat, vehicle1.location.global_relative_frame.lon)
    dist = distance_calculation(vehicle1.location.global_relative_frame.lat,vehicle1.location.global_relative_frame.lon,x_goal,y_goal)

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
            print("if 1 :", np.sign(k))
            ax = fx-fy
            ay = fx+fy
    else:
            print("else 1 :", np.sign(k))
            ax = fx+np.sign(k)*fy
            ay = fy-np.sign(k)*fx

    v_hdg = [ax, ay]

    return v_hdg

def mpf_grad_3d(k,theta,phi,vecx,vecc):
    diss = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle1.location.global_relative_frame.lat,vehicle1.location.global_relative_frame.lon)
    tat = theta_calculator(vecx.location.global_relative_frame.lat,vecx.location.global_relative_frame.lon,vecc)
    x = diss*math.sin(math.radians(tat))
    y = diss*math.cos(math.radians(tat))
    z = vecc.location.global_relative_frame.alt - vecx.location.global_relative_frame.alt
    print("tat: ",tat)
    print(x)
    print(y)
    print(z)
    a = 60
    b = 80
    c = 28
    f5 = -(k*((2*math.cos(theta)*math.sin(phi)*(y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta)))/b**2 - (2*math.sin(theta)*(z*math.cos(theta) - x*math.sin(theta)))/c**2 + (2*math.cos(phi)*math.cos(theta)*(x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta)))/a**2))/((z*math.cos(theta) - x*math.sin(theta))**2/c**2 + (x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta))**2/a**2 + (y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta))**2/b**2 + 1)**2
    f6 = (k*((2*math.sin(phi)*(x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta)))/a**2 - (2*math.cos(phi)*(y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta)))/b**2))/((z*math.cos(theta) - x*math.sin(theta))**2/c**2 + (x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta))**2/a**2 + (y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta))**2/b**2 + 1)**2
    f7 = -(k*((2*math.cos(theta)*(z*math.cos(theta) - x*math.sin(theta)))/c**2 + (2*math.cos(phi)*math.sin(theta)*(x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta)))/a**2 + (2*math.sin(phi)*math.sin(theta)*(y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta)))/b**2))/((z*math.cos(theta) - x*math.sin(theta))**2/c**2 + (x*math.cos(phi)*math.cos(theta) - y*math.sin(phi) + z*math.cos(phi)*math.sin(theta))**2/a**2 + (y*math.cos(phi) + x*math.cos(theta)*math.sin(phi) + z*math.sin(phi)*math.sin(theta))**2/b**2 + 1)**2
    heading_UAV = [f5,f6,f7]
    return heading_UAV    



# dist = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_goal,lon_goal)
# distt = distance_calculation(vehicle1.location.global_relative_frame.lat, vehicle1.location.global_relative_frame.lon,lat_goal1,lon_goal1)

i = 0
xpo = []
ypo = []
v_x = 1
v_y = 1
v_z = 0
v_x2 = -1
v_y2 = -1
v_z2 = 0
v_x_ob = -1
v_y_ob = -1

arm_and_takeoff(300)

arm_and_takeoffs(300)
lat_gps1 = []
lon_gps1 = []
altitude1 = []
lat_gps2 = []
lon_gps2 = []
altitude2 = []


while True:
    dist = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_goal,lon_goal)
    distt = distance_calculation(vehicle1.location.global_relative_frame.lat, vehicle1.location.global_relative_frame.lon,lat_goal1,lon_goal1)

    lat_ob = vehicle1.location.global_relative_frame.lat
    lon_ob = vehicle1.location.global_relative_frame.lon
    lat_ob2 = vehicle.location.global_relative_frame.lat
    lon_ob2 = vehicle.location.global_relative_frame.lon
    j = theta_calculator(lat_goal, lon_goal,vehicle)
    tt = theta_calculator(lat_goal1, lon_goal1,vehicle1)
    # phi = theta_calculator(lat_ob, lon_ob)
    vx = -v_x + v_x2
    vy = -v_y + v_y2
    vz = -v_z + v_z
    vx2 = -vx
    vy2 = -vy
    vz2 = -vz
    phi1 = math.atan2(abs(vx),abs(vy))
    phi2 = math.atan2(abs(vx2),abs(vy2))
    theta1 = math.asin(vz/(math.sqrt(vx**2 + vy**2 + vz**2)))
    theta2 = math.asin(vz2/(math.sqrt(vx2**2 + vy2**2 + vz2**2)))
    # angle = math.atan2(vy1,vx1)
    # angle1 = math.atan2(-vy1,-vx1)
    # print("ANgle 1:", math.degrees(angle1))
    dx = dist*math.sin(math.radians(j))
    dy = dist*math.cos(math.radians(j))
    ddx = distt*math.sin(math.radians(tt))
    ddy = distt*math.cos(math.radians(tt))
    dz = vehicle.location.global_relative_frame.alt - 300
    dz2 = vehicle1.location.global_relative_frame.alt - 300
    # xd = dist*math.cos(math.radians(phi))
    # yd = dist*math.sin(math.radians(phi))  
    # phiii = (xd*vx1 + yd*vy1)/((math.sqrt(xd**2+yd**2)*math.sqrt(vx1**2+vy1**2)))
    # phi_dash = math.radians(90-phi)

    # print(math.sin(3.14*phiii))

    # k_rep11 = k_rep1*(math.sin(math.radians((3.14*phiii/2))) + 1)

    # if k_rep11 < 0:
        # k_rep11 = 0

    # print(k_rep11)

    #pos = np.zeros((2,1))

    g = mpf_grad_3d(k_rep1,theta1,phi1,vehicle1,vehicle)
    g1 = mpf_grad_3d(k_rep1,theta2,phi2,vehicle,vehicle1)

    v1 = [k_att*dx, k_att*dy]
    v =  [g[0]+k_att*dx,g[1]+k_att*dy, g[2] + 0.2*dz]
    vo = [g1[0]+k_att*ddx,g1[1]+k_att*ddy, g1[2] + 0.2*dz2]

    print(v)
    print(vo)

    # print("v = ", v)
    # print("vbef", v1)
    # print("vaf", v)


    v_x = v[0]
    v_y = v[1]
    v_z = v[2]
    v_x2 = vo[0]
    v_y2 = vo[1]
    v_z2 = vo[2]

    r = math.sqrt(v_x*v_x + v_y*v_y)

    c = v_x/r
    s = v_y/r

    r1 = math.sqrt(v_x_ob*v_x_ob + v_y_ob*v_y_ob)

    c1 = v_x_ob/r1
    s1 = v_y_ob/r1

    sine = v_y/(math.sqrt(v_x**2+v_y**2))
    cosine = v_x/(math.sqrt(v_y**2+v_x**2))
    zsine = v_z/(math.sqrt(v_x**2+v_y**2+v_z**2))
    zcosine = (v_x**2+v_y**2)/(v_x**2+v_y**2+v_z**2)
    
    v_z = 5*zsine
    v_r = 5*zcosine
    v_x = v_r*cosine
    v_y = v_r*sine

    sine2 = v_y2/(math.sqrt(v_x2**2+v_y2**2))
    cosine2 = v_x2/(math.sqrt(v_y2**2+v_x2**2))
    zsine2 = v_z2/(math.sqrt(v_x2**2+v_y2**2+v_z2**2))
    zcosine2 = (v_x2**2+v_y2**2)/(v_x2**2+v_y2**2+v_z2**2)
    
    v_z2 = 5*zsine2
    v_r2 = 5*zcosine2
    v_x2 = v_r2*cosine2
    v_y2 = v_r2*sine2

    # vx = 1*c
    # vy = 1*s
    # vxo = 1*c1
    # vyo = 1*s1

    # print("VX:", vx,"VY:", vy)
    # print("VXO", vxo, "VYO", vyo)

    d2 = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_goal,lon_goal)
    d3 = distance_calculation(vehicle1.location.global_relative_frame.lat,vehicle1.location.global_relative_frame.lon,lat_goal1,lon_goal1)
    d1 = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_ob,lon_ob)
    dob = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle1.location.global_relative_frame.lat,vehicle1.location.global_relative_frame.lon)
    print("distance from obstacle : ", dob)
    # ddd1 = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_ob1,lon_ob1)    # print(d2)
    print("dist from obstacle:", d1)
    print("disssss: ",d3)
    # print("distance from obstacle 2 : ", ddd1)
    if d2 <= 2:
      send_global_velocity(0,0,0,1)
      break
    else:
      send_global_velocity(v_y,v_x,v_z,1)
      send_global_velocity1(v_y2,v_x2,v_z2,1)
  

    lat_gps1.append(vehicle.location.global_relative_frame.lat)
    lon_gps1.append(vehicle.location.global_relative_frame.lon)
    altitude1.append(vehicle.location.global_relative_frame.alt + 700)
    lat_gps2.append(vehicle1.location.global_relative_frame.lat)
    lon_gps2.append(vehicle1.location.global_relative_frame.lon)
    altitude2.append(vehicle1.location.global_relative_frame.alt + 700)
    
    home_lat = -35.36326490
    home_lon  = 149.16523606
    dd = distance_calculation(home_lat, home_lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    gg = theta_calculator_returns(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    df = distance_calculation(home_lat, home_lon, lat_ob, lon_ob)
    gf = theta_calculator_returns(lat_ob,lon_ob)
    help = distance_calculation(vehicle1.location.global_relative_frame.lat,vehicle1.location.global_relative_frame.lon,lat_goal1,lon_goal1)
    print("distance of goal1 :", help)
    # print("theta start:", gg)
    # print("distttttaaaa: ", dd)

    # print("x hai:",dd*math.sin(math.radians(gg)))

    xpo.append(dd*math.sin(math.radians(gg)))
    ypo.append(dd*math.cos(math.radians(gg)))


# xpo.append(df*math.sin(math.radians(gf)))
# ypo.append(df*math.cos(math.radians(gf)))

# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()



f = open('flight7.kml', 'w')

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

f = open('flight8.kml', 'w')

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
for i in range(0,len(altitude2),10):  #Here I skip some data
    f.write("        "+str(lon_gps2[i]) + ","+ str(lat_gps2[i]) + "," + str(altitude2[i]) +"\n")    
f.write("       </coordinates>\n")
f.write("   </LineString>\n")
f.write("</Placemark>\n")
f.write("</Document>")
f.write("</kml>\n")
f.close()

print("About to Land")

vehicle.mode = VehicleMode("RTL")
vehicle1.mode = VehicleMode("RTL")

print("Landed.")

f = distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,lat_goal,lon_goal)

# print("accuracy :", f) 
# print(xpo)
# print("######################")
# print(ypo)
# plt.plot(xpo,ypo)
# plt.show()

vehicle.close()
vehicle1.close()
