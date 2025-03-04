from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import numpy as np
from matplotlib import pyplot as plt

# vehicle = connect('/dev/ttyTHS1', wait_ready=True, baud=57600)
vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=57600)
# vehicle2 = connect('127.0.0.1:14562', wait_ready=True, baud=57600)

goal2 = (  -35.36003206, 149.18010157 )
goal1 = ( 28.74523023, 77.11690684 )
obstacle = [ 28.75532660,77.11580188, 28.75441063, 77.11496787, 28.75391246,77.11441797,28.75324557,77.11375809, 28.75231351,77.11285075, 28.75139751,77.11199841,28.75070649,77.11139352, 28.75017617, 77.11176928, 28.74926015, 77.11279576, 28.74897892, 77.11337316, 28.74926015, 77.11373976, 28.74913159, 77.11440880, 28.74887446, 77.11499536, 28.74838276, 77.11522786,  28.74748828, 77.11544965, 28.74710909, 77.11553836, 28.74635071, 77.11573797, 28.74569928, 77.11584887, 28.74492154, 77.11833286, 28.74555353, 77.11878752,  28.74603966, 77.11924219, 28.74641885, 77.11959705, 28.74714805, 77.12026241, 28.74741056, 77.12051747,  28.74814948, 77.12123828, 28.74831476, 77.12136026, 28.74857727, 77.12164858, 28.74903422, 77.12211434, 28.74961757, 77.12263554, 28.75015230, 77.12193691, 28.75009397, 77.12199236, 28.75058980, 77.12138244, 28.75110509, 77.12070599, 28.75180509, 77.11979666, 28.75254397, 77.11888733, 28.75327312, 77.11802245, 28.75412867, 77.11702438, 28.75449811, 77.11653644, 28.75351569, 77.11726854, 28.75362526, 77.11518559, 28.75282907, 77.11535223, 28.75266837, 77.11414412, 28.75228854, 77.11474401, 28.75208401, 77.11661866, 28.75279985, 77.11777677, 28.75106137, 77.11538555, 28.75097371, 77.11595211, 28.75072535, 77.11645202, 28.75019942, 77.11639370, 28.74965156, 77.11616041, 28.75022864, 77.11711856,  28.75095180, 77.11746850, 28.75146312, 77.11798507, 28.75090797, 77.11821836, 28.74938859, 77.11776011, 28.74941781, 77.11701025, 28.74878960, 77.11658533, 28.74860698, 77.11714356, 28.74863620, 77.11776011, 28.74878230, 77.11826002, 28.74895761, 77.11885158, 28.74962965, 77.11909320, 28.74999489, 77.11914319, 28.75006063, 77.11820170, 28.75101754 ,77.11875159, 28.75147773, 77.11879325, 28.75114902, 77.11913486, 28.74932296, 77.12025119, 28.74895042, 77.12070943, 28.74798621, 77.11870151, 28.74830762, 77.11920141, 28.74705851, 77.11686023, 28.75097042, 77.11562471,  28.75107273, 77.11540181, 28.75076594, 77.11541848, 28.75082437, 77.11586006, 28.75111656, 77.11595171, 28.75064906, 77.11618501, 28.75067097, 77.11681822, 28.75070750, 77.11646829,  28.74913705, 77.11675985, 28.74940002, 77.11721809, 28.74936350, 77.11759302, 28.74942924, 77.11774299, 28.74991865, 77.11817624, 28.75116046, 77.11451027 , 28.75067106, 77.11480188, 28.75098515, 77.11467691, 28.75065645, 77.11400204 ]

# obstacle = [ 28.75530316 77.11581013 28.75505038 77.11555506  ]

bhai = len(obstacle)

# obstacle = (-35.35206193, 149.17084008)

jkkk = 0

u = 1

for i in range(89):
    globals()["lat_ob"+str(u)] = obstacle[jkkk]
    globals()["lon_ob"+str(u)] = obstacle[jkkk + 1]
    u = u + 1
    jkkk = jkkk+2

print("ye le lat: ", lat_ob1)
print("ye le lat: ", lon_ob2)
print("ye le lat: ", lon_ob3)
print("ye le lat: ", lat_ob4)


lat_goal1 = goal1[0]
lon_goal1 = goal1[1]
lat_goal2 = goal2[0]
lon_goal2 = goal2[1]

point = LocationGlobalRelative( lat_goal1, lon_goal1, 80)
point2 = LocationGlobalRelative( lat_goal2, lon_goal2, 80)

k_att = 0.02
k_rep1 = 1300
a1 = 120
b1 = 100

def goto(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle.simple_goto(location)

def goto_2(location):
        """ Go to a location
        
        Input:
            location    - LocationGlobal or LocationGlobalRelative object
        
        """
        vehicle2.simple_goto(location)

def take_off(alti,vec):
    print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
    while not vec.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vec.mode    = VehicleMode("TAKEOFF")
    vec.armed   = True
    while True:
    #print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
        if vec.location.global_relative_frame.alt>= alti*0.95: 
            print("Reached target altitude")
            vehicle.mode = VehicleMode("GUIDED")
            goto(point2)
            break
        time.sleep(1)

def take_off_2(alti,vec):
    print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
    while not vec.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vec.mode    = VehicleMode("TAKEOFF")
    vec.armed   = True
    while True:
    #print(" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
        if vec.location.global_relative_frame.alt>= alti*0.95: 
            print("Reached target altitude")
            vehicle2.mode = VehicleMode("GUIDED")
            goto_2(point)
            break
        time.sleep(1)

def _get_location_metres( dNorth, dEast, is_global=False):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `vehicle.location.global_relative`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*vehicle.location.global_relative_frame.lat/180))
    #New position in decimal degrees
    newlat = vehicle.location.global_relative_frame.lat + (dLat * 180/math.pi)
    newlon = vehicle.location.global_relative_frame.lon + (dLon * 180/math.pi)
    
    if is_global:
        return LocationGlobalRelative(newlat, newlon,vehicle.location.global_relative_frame.alt)    
    else:
        return LocationGlobalRelative(newlat, newlon,vehicle.location.global_relative_frame.alt)

def get_target_from_bearing( ang, dist, altitude=None):
    """ Create a TGT request packet located at a bearing and distance from the original point
    
    Inputs:
        ang     - [rad] Angle respect to North (clockwise) 
        dist    - [m]   Distance from the actual location
        altitude- [m]
    Returns:
        location - Dronekit compatible
    """
    
    if altitude is None: 
        altitude = 80
    
    # print '---------------------- simulate_target_packet'
    dNorth  = dist*math.cos(ang)
    dEast   = dist*math.sin(ang)
    # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast) 
    
    #-- Get the Lat and Lon
    tgt     = _get_location_metres( dNorth, dEast)
    
    tgt.alt = altitude
    # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt
    return tgt

def ground_course_2_location( angle_deg, altitude=None ):
    """ Creates a target to aim to in order to follow the ground course
    Input:
        angle_deg   - target ground course
        altitude    - target altitude (default the current)
    
    """
    tgt = get_target_from_bearing( ang=math.radians(angle_deg),dist=5000,altitude=altitude)
    return(tgt)

def set_ground_course( angle_deg, altitude=None):
    """ Set a ground course
    
    Input:
        angle_deg   - [deg] target heading
        altitude    - [m]   target altitude (default the current)
    
    """
    
    #-- command the angles directly
    goto(ground_course_2_location(angle_deg, altitude))

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

def _get_location_metres_2( dNorth, dEast, is_global=False):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `vehicle.location.global_relative`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*vehicle2.location.global_relative_frame.lat/180))
    #New position in decimal degrees
    newlat = vehicle2.location.global_relative_frame.lat + (dLat * 180/math.pi)
    newlon = vehicle2.location.global_relative_frame.lon + (dLon * 180/math.pi)
    
    if is_global:
        return LocationGlobalRelative(newlat, newlon,vehicle2.location.global_relative_frame.alt)    
    else:
        return LocationGlobalRelative(newlat, newlon,vehicle2.location.global_relative_frame.alt)

def get_target_from_bearing_2( ang, dist, altitude=None):
    """ Create a TGT request packet located at a bearing and distance from the original point
    
    Inputs:
        ang     - [rad] Angle respect to North (clockwise) 
        dist    - [m]   Distance from the actual location
        altitude- [m]
    Returns:
        location - Dronekit compatible
    """
    
    if altitude is None: 
        altitude = 80
    
    # print '---------------------- simulate_target_packet'
    dNorth  = dist*math.cos(ang)
    dEast   = dist*math.sin(ang)
    # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast) 
    
    #-- Get the Lat and Lon
    tgt     = _get_location_metres_2( dNorth, dEast)
    
    tgt.alt = altitude
    # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt
    return tgt

def ground_course_2_location_2( angle_deg, altitude=None):
    """ Creates a target to aim to in order to follow the ground course
    Input:
        angle_deg   - target ground course
        altitude    - target altitude (default the current)
    
    """
    tgt = get_target_from_bearing_2( ang=math.radians(angle_deg),dist=5000,altitude=altitude)
    return(tgt)

def set_ground_course_2( angle_deg, altitude=None):
    """ Set a ground course
    
    Input:
        angle_deg   - [deg] target heading
        altitude    - [m]   target altitude (default the current)
    
    """
    
    #-- command the angles directly
    goto_2(ground_course_2_location_2(angle_deg, altitude))

def theta_calculator(lat,lon,vecc):
  d = lon-vecc.location.global_relative_frame.lon
  x = math.cos(math.radians(lat))*math.sin(math.radians(d))
  y = math.cos(math.radians(vecc.location.global_relative_frame.lat))*math.sin(math.radians(lat))-math.sin(math.radians(vecc.location.global_relative_frame.lat))*math.cos(math.radians(lat))*math.cos(math.radians(d))
  theta = math.degrees(math.atan2(x,y))
  return theta

def mpf_grad(x_o,y_o,a,b,theta1,k_rep,x_goal,y_goal,veccc):
    
    thetu = theta_calculator(x_goal,y_goal,veccc)
    thets = theta_calculator(x_o, y_o,veccc)
    d = distance_calculation(x_o,y_o,veccc.location.global_frame.lat, veccc.location.global_frame.lon)
    # dist = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal,lon_goal)

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
            ay = fy-np.sign(k)*fx

    v_hdg = [ax, ay]

    return v_hdg


point = LocationGlobalRelative( lat_goal1, lon_goal1, 80)
point2 = LocationGlobalRelative( lat_goal2, lon_goal2, 80)
take_off(20,vehicle)
# take_off_2(20,vehicle2)

an = theta_calculator(lat_goal1,lon_goal1,vehicle)
# an2 = theta_calculator(lat_goal2,lon_goal2,vehicle2)

v_x = math.cos(math.radians(an))
v_y = math.sin(math.radians(an))
# v_x2 = math.cos(math.radians(an2))
# v_y2 = math.sin(math.radians(an2))
vx = -v_x 
vy = -v_y 
vx2 = -vx
vy2 = -vy
roll = []
pitch = []
yaw = []
roll1 = []
pitch1 = []
yaw1 = []
ti = []

# while True:
    # goto(point2)
    # goto_2(point)
    # if distance_calculation(vehicle2.location.global_relative_frame.lat, vehicle2.location.global_relative_frame.lon,lat_goal1, lon_goal1) <=100:
        # goto(point)
        # goto(point2)
        # break

goto(point)

fl = 1
vl = 1
while True:
    fl = 1
    vl = 1
    sl = 1
    dist = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_goal1,lon_goal1)
    # dist2 = distance_calculation(vehicle2.location.global_frame.lat,vehicle2.location.global_frame.lon,lat_goal2,lon_goal2)

    # lat_ob1 = vehicle2.location.global_relative_frame.lat
    # lon_ob1 = vehicle2.location.global_relative_frame.lon
    # lat_ob2 = vehicle.location.global_relative_frame.lat
    # lon_ob2 = vehicle.location.global_relative_frame.lon
    
    
    # distob = distance_calculation(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,lat_ob,lon_ob)
    j = theta_calculator(lat_goal1, lon_goal1, vehicle)
    # j2 = theta_calculator(lat_goal2, lon_goal2, vehicle2)
    # phi = theta_calculator(lat_ob1, lon_ob1,vehicle)
    
    
    vx = -v_x  
    vy = -v_y 
    # vx2 = -vx
    # vy2 = -vy

    angle = math.atan2(vy,vx)
    angle2 = math.atan2(vy2,vx2)
    dx = dist*math.sin(math.radians(j))
    dy = dist*math.cos(math.radians(j))
    # dx2 = dist*math.sin(math.radians(j2))
    # dy2 = dist*math.cos(math.radians(j2))
    # xd = dist*math.cos(math.radians(phi))
    # yd = dist*math.sin(math.radians(phi))  
    # phiii = (xd*vx1 + yd*vy1)/((math.sqrt(xd**2+yd**2)*math.sqrt(vx1**2+vy1**2)))
    # phi_dash = math.radians(90-phi)
    # g = mpf_grad(lat_ob1, lon_ob1, a1, b1, angle, k_rep1, lat_goal1, lon_goal1,vehicle)
    # g2 = mpf_grad(lat_ob2, lon_ob2, a1, b1, angle2, k_rep1, lat_goal2, lon_goal2,vehicle2
    # 
    # )
    for f in range(89):
        globals()["g"+str(vl)] = mpf_grad(globals()["lat_ob"+str(vl)],globals()["lon_ob"+str(vl)],a1,b1,angle,k_rep1,lat_goal1,lon_goal1,vehicle)
        vl= vl + 1
    
    
    v = [k_att*dx, k_att*dy]
    for dsn in range(89):
        v[0] = v[0] + globals()["g"+str(sl)][0]
        v[1] = v[1] + globals()["g"+str(sl)][1]
        sl = sl + 1

    # v2 = [g2[0] +k_att*dx2, g2[1] + k_att*dy2]
    print("vel: ", v)
    v_x = v[0]
    v_y = v[1]
    # v_x2 = v2[0]
    # v_y2 = v2[1]

    r = math.sqrt(v_x*v_x + v_y*v_y)

    if v_x > 0 and v_y > 0:
        c = math.acos(v_y/r)
        print("hello1")
    elif v_x >0 and v_y < 0:
        c = math.radians(90) + math.acos(abs(v_x/r))
        print("hello2")
    elif v_x<0 and v_y <0:
        c = math.radians(180) + math.acos(abs(v_y/r))
        print("hello3")
    elif v_x < 0 and v_y > 0:
        c = math.radians(270) + math.acos(abs(v_x/r))
        print("hello4")

    # r2 = math.sqrt(v_x2*v_x2 + v_y2*v_y2)
# 
    # if v_x2 > 0 and v_y2 > 0:
        # c2 = math.acos(v_y2/r2)
        # print("hello1")
    # elif v_x2 >0 and v_y2 < 0:
        # c2 = math.radians(90) + math.acos(abs(v_x2/r2))
        # print("hello2")
    # elif v_x2<0 and v_y2 <0:
        # c2 = math.radians(180) + math.acos(abs(v_y2/r2))
        # print("hello3")
    # elif v_x2 < 0 and v_y2 > 0:
        # c2 = math.radians(270) + math.acos(abs(v_x2/r2))
        # print("hello4")    

    # print("distance to obstacle:", distob)
    print("degrees", math.degrees(c))
    set_ground_course(math.degrees(c))
    # set_ground_course_2(math.degrees(c2))

    roll.append(vehicle.attitude.roll)
    pitch.append(vehicle.attitude.pitch)
    yaw.append(vehicle.attitude.yaw)
    # roll1.append(vehicle2.attitude.roll)
    # pitch1.append(vehicle2.attitude.pitch)
    # yaw1.append(vehicle2.attitude.yaw)
    ti.append(time.time())
    
    if distance_calculation(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, lat_goal1, lon_goal1) <=30:
        goto(point)
        break    

# figure, axis = plt.subplots(1, 3)
# axis[0,0].plot(ti,roll)
# axis[0,0].set_title("Roll vs Time (Vehicle 1)")
# axis[0,1].plot(ti,pitch)
# axis[0,1].set_title("Pitch vs Time (Vehicle1)")
# axis[0,2].plot(ti,yaw)
# axis[0,2].set_title("Yaw vs Time (Vehicle 1 )")
# axis[1,0].plot(ti,roll1)
# axis[1,0].set_title("Roll vs Time (Vehicle 2)")
# axis[1,1].plot(ti,pitch1)
# axis[1,1].set_title("Pitch vs Time (Vehicle 2)")
# axis[1,2].plot(ti,yaw1)
# axis[1,2].set_title("Yaw vs Time (Vehicle 2)")
plt.show()
