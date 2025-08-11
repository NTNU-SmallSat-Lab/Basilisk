#
# Python file filled with different useful functions
#
# Purpose:  Implementation of tools that can be used to have the simulation

import math
import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities import RigidBodyKinematics
from datetime import datetime
from datetime import datetime, timezone
import spiceypy as spice


def julian_day(date):
    """Computes the Julian Day (JD) from a UTC datetime."""
    a = (14 - date.month) // 12
    y = date.year + 4800 - a
    m = date.month + 12*a - 3
    jdn = date.day + (153*m + 2)//5 + 365*y + y//4 - y//100 + y//400 - 32045
    jd = jdn + (date.hour - 12)/24 + date.minute/1440 + date.second/86400
    return jd

def greenwich_sidereal_angle_deg(date_utc):
    """Computes the Greenwich Apparent Sidereal Time (GAST) in degrees."""
    # 1. Compute the Julian Day
    jd = julian_day(date_utc)
    
    # 2. Julian centuries since J2000.0
    T = (jd - 2451545.0) / 36525.0
    
    # 3. Greenwich Mean Sidereal Time (GMST) in degrees
    # Using the IERS Conventions 2003 formula
    gmst_deg = 280.46061837 + 360.98564736629*(jd - 2451545.0) + 0.000387933*T**2 - T**3/38710000.0
    
    # 4. Nutation correction (simplified)
    # Mean longitude of the Sun (L) and the Moon (Lp)
    L = math.radians(280.4665 + 36000.7698*T)
    Lp = math.radians(218.3165 + 481267.8813*T)
    
    # Nutation in longitude (Δψ) in degrees
    delta_psi = -17.20*math.sin(125.04 - 1934.136*T) / 3600  
    
    # 5. Apparent sidereal time (GAST) in degrees
    epsilon = 23.4393 - 0.0130*T  # Obliquity of the ecliptic
    gast_deg = gmst_deg + delta_psi*math.cos(math.radians(epsilon))
    
    # Normalize to range [0°, 360°)
    gast_deg %= 360
    
    return gast_deg




def initialCoordinate(altitude,latitude, longitude, inclination, down=False):
    """
    The function calculate the initial arguments needed in the run function to start the simulation 
    with the satellite situated at a wanted position (given with its geographic coordinates) 
    and a chosen direction :

    Args:
        altitude (float): Not really needed in the calculations but necessary to initialize the simulation
        latitude (float) : the latitude of the wanted position
        longitude (float) : the longitude of the wanted position
        inclination (float) : the initial orbital inclination and the direction we want

    """
    left = False
    angle = inclination
    up = not down

    if inclination%360 > 180:
        angle = 360 - angle%360.
        up = not up
    
    if angle > 90.:
        angle = 180 - angle
        left = True

    if angle < np.abs(latitude):

        # if the inclination of the orbit is inferior to the latitude we want the satellite to be situated above, there is no way for the satellite to reach this latitude

        raise ValueError("The inclination of the orbit cannot be inferior to the latitude of the place you want the satellite")
    elif (angle> 90. or angle<-90.) and np.abs((angle-180)%360-360) < np.abs(latitude):
        # the same goes if the orbit goes in a couterclockwise direction

        raise ValueError("The inclination of the orbit cannot be inferior to the latitude of the place you want the satellite")


    tab = [altitude, inclination]
    
    
    if angle == 0.:

        # if the orbit inclination is equal to 0 it means that the orbit is situated right above the equator 
        # we then suppose that the longitude of the ascending node is 0 
        # and thus the satellite is situated on the orbit at a true anomaly corresponding to the longitude we want

        
        if left: 
            nodeLongitude = 180.
            posOrbit = 180 - longitude
        else :
            nodeLongitude = 0.
            posOrbit = longitude

    elif angle == 90.:
        # if the orbit inclination is equal to 90 it means that the orbit is polar
        # we then suppose that the longitude of the ascending node is 0 
        # and thus the satellite is situated on the orbit at a true anomaly corresponding to the latitude we want

        if up :
            nodeLongitude = longitude
            posOrbit = latitude
        else :
            nodeLongitude = 180. + longitude
            posOrbit = 180 - latitude

    elif angle == np.abs(latitude):

        # if the orbit inclination and the latitude are equal it means the satellite is at the maximum latitude it could have on this orbit
        # which means that the ascending node is at an angle of 90 degrees before the longitude we want the satellite to be at
        # and is situated at a quarter of the orbit after the ascending node
        if latitude>0 :
            nodeLongitude = (longitude -90. * (-1 if left else 1)) %360.
        else :
            nodeLongitude = (longitude -90. * (1 if left else -1)) %360.
        
        posOrbit = 90. + (180 if latitude<0 else 0.)

    else :
        if not up : 

            # calculate the longitude of the ascending node with the parameters we have
            nodeLongitude = longitude - math.asin(math.tan(latitude*macros.D2R)/math.tan(angle*macros.D2R))*macros.R2D + 180.

            # calculate the position on the orbit (true anomaly) with a simple formula given by the latitude and the inclination of the orbit
            posOrbit = 180. - math.asin(math.sin(latitude*macros.D2R)/math.sin(angle*macros.D2R)) * macros.R2D

        else :
            
            # calculate the position on the orbit (true anomaly) with a simple formula given by the latitude and the inclination of the orbit
            posOrbit = math.asin(math.sin(latitude*macros.D2R)/math.sin(angle*macros.D2R)) * macros.R2D

            # calculate the longitude of the ascending node with the parameters we have
            nodeLongitude = longitude - math.asin(math.tan(latitude*macros.D2R)/math.tan(angle*macros.D2R))*macros.R2D 

        

    # add the values to the initial conditions we want to return
    tab.append(nodeLongitude)
    tab.append(posOrbit)

    
    print(f"Initial orbital parameters : {tab}")
    return tab



def initialCoordinateTraject(altitude, latitude1, longitude1,latitude2, longitude2):
    """
    The function calculate the initial arguments needed in the run function to start the simulation 
    with the satellite situated at a wanted position (given with its geographic coordinates) 
    and a chosen destination which is supposed to be close and thus, 
    the difference between longitudes and latitudes of the initial location and the destination 
    must be less than 90 degrees :

    Args:
        altitude (float): Not really needed in the calculations but necessary to initialize the simulation
        longitude1 (float) : the longitude of the wanted initial position
        latitude1 (float) : the latitude of the wanted initial position
        longitude2 (float) : the longitude of the wanted destination
        latitude2 (float) : the latitude of the wanted destination

    """
    down = (latitude2 - latitude1)<0

    
    r1 = geodetic_to_cartesian(latitude1, longitude1)
    r2 = geodetic_to_cartesian(latitude2, longitude2)

    n = np.cross(r1, r2)
    print(n)
    norm_n = np.linalg.norm(n)
    if norm_n == 0:
        raise ValueError("identicals points or antipodes")
    n_unit = n / norm_n


    # Inclinate = angle between the orbital plan and z
    inclinate = math.acos(n_unit[2]) * macros.R2D

    return initialCoordinate(altitude, latitude1, longitude1, inclinate, down)

def geodetic_to_cartesian(lat_deg, lon_deg):
    lat = lat_deg * macros.D2R
    lon = lon_deg * macros.D2R
    x = math.cos(lat) * math.cos(lon)
    y = math.cos(lat) * math.sin(lon)
    z = math.sin(lat)
    return np.array([x, y, z])

def cartesian_to_geodetic(vec):
    x, y, z = vec
    lat = np.arcsin(z)
    lon = np.arctan2(y, x)
    return np.degrees(lat), np.degrees(lon)


def rotate_vector_in_body_frame(vector, quaternion, position, velocity):
    """
    Rotates a vector using a quaternion in a body frame defined as:
    - X: position (normalized)
    - Y: perpendicular to position and velocity (right-hand rule)
    - Z: velocity (normalized)        Parameters:
        vector (np.ndarray): Vector in ECI frame to rotate.
        quaternion (array-like): Quaternion [x, y, z, w] for rotation (body frame).
        position (np.ndarray): Position vector (defines X axis).
        velocity (np.ndarray): Velocity vector (defines Z axis).        Returns:
        rotated_vector_in_eci (np.ndarray): Rotated vector in ECI frame.
        x_body (np.ndarray): X axis of body frame.
        y_body (np.ndarray): Y axis of body frame.
        z_body (np.ndarray): Z axis of body frame.
    """
    # Step 1: Construct body frame basis
    z_body = - np.linalg.norm(position)  # X-axis
    x_body = np.linalg.norm(velocity)  # Y-axis
    y_body = np.linalg.norm(np.cross(z_body, x_body))  # Recompute Y to ensure orthogonality     

    # Transformation matrix (body frame to ECI frame)
    body_to_eci = np.array([x_body, y_body, z_body]).T  # Basis vectors as columns        
    
    # Step 2: Transform the vector to the body frame
    eci_to_body = np.linalg.inv(body_to_eci)  # Inverse of the transformation
    vector_in_body_frame = np.dot(eci_to_body, vector)        
    
    # Step 3: Rotate the vector in the body frame
    rotation = math.from_quat(quaternion)  # Create rotation object
    rotated_vector_in_body_frame = rotation.apply(vector_in_body_frame)        
    
    # Step 4: Transform the rotated vector back to the ECI frame
    rotated_vector_in_eci = np.dot(body_to_eci, rotated_vector_in_body_frame)        
    return rotated_vector_in_eci, x_body, y_body, z_body  # Also return body basis


def rotation_matrix_to_quaternion(R):
    """
    Convertit une matrice de rotation 3x3 en quaternion [w, x, y, z]
    
    Args:
        R: matrice de rotation 3x3 (numpy array)
    
    Returns:
        quaternion [w, x, y, z] où w est la partie scalaire
    """
    
    # Vérification que c'est bien une matrice 3x3
    assert R.shape == (3, 3), "La matrice doit être 3x3"
    
    # Extraction des éléments de la matrice
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
    
    # Calcul de la trace (somme des éléments diagonaux)
    trace = r11 + r22 + r33
    
    
    # Algorithme de Shepperd (le plus stable numériquement)
    if trace > 0:
        # Cas où la trace est positive - on calcule w en premier
        s = math.sqrt(trace + 1.0) * 2  # s = 4 * w
        w = 0.25 * s
        x = (r32 - r23) / s
        y = (r13 - r31) / s
        z = (r21 - r12) / s
        
    elif r11 > r22 and r11 > r33:
        # Cas où r11 est le plus grand élément diagonal
        s = math.sqrt(1.0 + r11 - r22 - r33) * 2  # s = 4 * x
        w = (r32 - r23) / s
        x = 0.25 * s
        y = (r12 + r21) / s
        z = (r13 + r31) / s
        
    elif r22 > r33:
        # Cas où r22 est le plus grand élément diagonal
        s = math.sqrt(1.0 + r22 - r11 - r33) * 2  # s = 4 * y
        w = (r13 - r31) / s
        x = (r12 + r21) / s
        y = 0.25 * s
        z = (r23 + r32) / s
        
    else:
        # Cas où r33 est le plus grand élément diagonal
        s = math.sqrt(1.0 + r33 - r11 - r22) * 2  # s = 4 * z
        w = (r21 - r12) / s
        x = (r13 + r31) / s
        y = (r23 + r32) / s
        z = 0.25 * s
    
    # Normalisation (au cas où il y aurait des erreurs d'arrondi)
    quaternion = np.array([w, x, y, z])
    norm = np.linalg.norm(quaternion)
    quaternion = quaternion / norm
    
    
    return quaternion

def multiply_quaternions(q1, q2):
    """
    Multiplie deux quaternions q1 ⊗ q2
    
    Args:
        q1, q2: quaternions [w, x, y, z]
    
    Returns:
        quaternion résultant [w, x, y, z]
    """
    
    # Extraction des composantes
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    

    # Partie scalaire (w)
    w_result = w1*w2 - x1*x2 - y1*y2 - z1*z2
    
    # Partie i (x)
    x_result = w1*x2 + x1*w2 + y1*z2 - z1*y2
    
    # Partie j (y)
    y_result = w1*y2 - x1*z2 + y1*w2 + z1*x2
    
    
    # Partie k (z)
    z_result = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    result = np.array([w_result, x_result, y_result, z_result])
    
    # Vérification de la norme
    norm_q1 = np.linalg.norm(q1)
    norm_q2 = np.linalg.norm(q2)
    norm_result = np.linalg.norm(result)
    
    
    return result


def EulerToQuat(attitude, position, celerity):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion representations.
    
    Args:
        attitude (list): List of Euler angles [roll, pitch, yaw] in radians.
        
    Returns:
        list: Quaternion in Local Vertical Local Horizontal frame [qx, qy, qz, qw].
        list: Quaternion in Earth Centered Inertial frame [qx, qy, qz, qw].
    """
    yaw, roll, pitch = attitude[0], attitude[1], attitude[2]
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q_LVLH = [cr*cp*sy - cy*sr*sp, cr*sp*cy + sy*cp*sr, sr*cp*cy - sy*cr*sp, cy*cr*cp + sy*sr*sp]

    r_vec = - position / np.linalg.norm(position)  # Position vector normalized
    h_vec = np.cross(position, celerity/np.linalg.norm(celerity))  # Angular momentum vector
    h_vec /= np.linalg.norm(h_vec)  # Normalize angular momentum vector
    s_vec = np.cross(h_vec, r_vec)  # Perpendicular vector to h and r

    rotation_mat = [s_vec, h_vec, r_vec]  # Construct the rotation matrix from the body frame to ECI frame
    
    quat_rotation = rotation_matrix_to_quaternion(np.array(rotation_mat).T)  # Convert rotation matrix to quaternion

    # Quaternion in ECI frame
    q_ECI = multiply_quaternions(quat_rotation,q_LVLH)

    return [q_LVLH, q_ECI]



import numpy as np

def get_direction(vec, orientation):
    # Convert angles from degrees to radians
    psi = np.radians(orientation[0])    # Yaw (Z)
    theta = np.radians(orientation[1]) # Pitch (Y)
    phi = np.radians(orientation[2])    # Roll (X)
    
    # The rotation matrices for yaw, pitch, and roll
    Rz = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    Ry = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    
    # Composed rotation Z -> Y -> X
    R = Rz @ Ry @ Rx
    
    # Applicate the rotation to the vector vec
    v_dir = R @ vec
    
    return v_dir.tolist()

def angular_velocity(r_BN_N, v_BN_N, inclination):
    """
    Calculate the initial angular velocity of the satellite based on its altitude.
    
    Args:
        altitude (float): The altitude of the satellite in kilometers.
        
    Returns:
        float: The initial angular velocity in rad/s.
    """
    mu=3.986004418e14

    r_vec = - r_BN_N / np.linalg.norm(r_BN_N)  # Position vector normalized
    h_vec = np.cross(r_BN_N, v_BN_N)  # Angular momentum vector
    h_vec /= np.linalg.norm(h_vec)  # Normalize angular momentum vector
    s_vec = np.cross(h_vec, r_vec)  # Perpendicular vector to h and r

    rotation_mat = np.array([s_vec, h_vec, r_vec]).T  # Construct the rotation matrix from the body frame to ECI frame
    # Vecteur moment angulaire orbital
    n = np.sqrt(mu / (np.linalg.norm(r_BN_N)**3))  # Orbital mean motion
    

    return [np.dot(rotation_mat, np.array([0, n, 0])), RigidBodyKinematics.C2MRP(rotation_mat)]  # Angular velocity and orientation in ECI frame


def coordinates_target(r_BN_N, direction):
    """
    This function computes the target's coordinates in the ECI frame based on the satellite's position and orientation (yaw, pitch, roll).

    The satellite looks in the direction of its Z-axis, which is defined by the orientation angles. 
    Then we search the nearer point where the direction line and the earth intersect.
    
    Args:
        r_BN_N (np.ndarray): The position of the satellite in ECI coordinates [x, y, z] in meters.
        direction (list): The direction of the camera.
        
    Returns:
        array: the position of the target in the ECI frame [x, y, z] in meters.
    """
    

    direction_ECI =  direction

    r_cam = np.array(r_BN_N)
    d = direction_ECI / np.linalg.norm(direction_ECI)

    R_earth = 6378137.0  # rayon Terre en m

    # We want to solve || r_cam + t * d || = R_earth

    # Coefficients de l'équation quadratique : At^2 + Bt + C = 0
    A = np.dot(d, d)
    B = 2 * np.dot(d, r_cam)
    C = np.dot(r_cam, r_cam) - R_earth**2

    discriminant = B**2 - 4*A*C

    if discriminant < 0:
        return None
    else:
        t = (-B - np.sqrt(discriminant)) / (2*A)  # nearest intersection point
        intersection = r_cam + t * d
    
    return np.array(intersection)


def get_camera_frame(r_BN_N, v_BN_N, orientation, field_of_view):
    """
    This function computes the camera frame based on the satellite's position, orientation, and field of view.
    The camera frame is defined by the satellite's position and the direction it is looking at, which is determined by its orientation angles (yaw, pitch, roll).

    Args:
        r_BN_N (np.ndarray): The position of the satellite in ECI coordinates [x, y, z] in meters.
        orientation (list): The orientation of the camera defined by yaw, pitch, and roll angles in degrees.
        field_of_view (float): The field of view of the camera in degrees.
    Returns:
        list: A list containing the coordinates of the camera frame in the format [up_left, center, bottom_right].
    """
    r_vec = - r_BN_N / np.linalg.norm(r_BN_N)  # Position vector normalized
    h_vec = np.cross(r_BN_N, v_BN_N)  # Angular momentum vector
    h_vec /= np.linalg.norm(h_vec)  # Normalize angular momentum vector
    s_vec = np.cross(h_vec, r_vec)  # Perpendicular vector to h and r
    eci_mat = np.array([s_vec, h_vec, r_vec]).T

    DCM_BN = RigidBodyKinematics.MRP2C(orientation)
    direction_cam = np.array([0, 0, 1])

    direction_ECI = eci_mat @ DCM_BN @ direction_cam  # Convert the camera direction from body frame to ECI frame
    center = coordinates_target(r_BN_N, direction_ECI)


    tan_x = np.tan(np.radians(field_of_view[0]) / 2)
    tan_y = np.tan(np.radians(field_of_view[1]) / 2)
    
    direction_up_left = eci_mat @ DCM_BN @ np.array([-tan_x, tan_y, 1])
    top_left = coordinates_target(r_BN_N, direction_up_left)

    direction_top_right = eci_mat @ DCM_BN @ np.array([tan_x, tan_y, 1])
    top_right = coordinates_target(r_BN_N, direction_top_right)

    direction_bottom_left = eci_mat @ DCM_BN @ np.array([-tan_x, -tan_y, 1])
    bottom_left = coordinates_target(r_BN_N, direction_bottom_left)

    direction_bottom_right = eci_mat @ DCM_BN @ np.array([tan_x, -tan_y, 1])
    bottom_right = coordinates_target(r_BN_N, direction_bottom_right)

    
    

    if center is None and top_left is None and bottom_right is None and top_right is None and bottom_left is None:
        print("The camera can't see Earth in this direction.")
        return None
    
    return [top_left, top_right, center, bottom_left, bottom_right]


def eci_to_geodetic(r_ECI, theta_GST):
    if r_ECI is None or len(r_ECI) != 3:
        return [1000., 0.0, 0.0]  # Return zero coordinates if input is invalid
    # Earth constants
    a = 6378137.0         # Equatorial radius (m)
    b = 6356752.3142      # Polar radius (m)
    e2 = 1 - (b**2 / a**2)
    
    # Step 1: Rotate ECI to ECEF
    R = np.array([
        [np.cos(theta_GST), np.sin(theta_GST), 0],
        [-np.sin(theta_GST), np.cos(theta_GST), 0],
        [0, 0, 1]
    ])
    r_ECEF = R @ r_ECI
    x, y, z = r_ECEF
    p = np.sqrt(x**2 + y**2)

    # Step 2: Compute latitude, longitude, altitude
    theta = np.arctan2(z * a, p * b)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z + ((a**2 - b**2)/b**2) * b * np.sin(theta)**3,
                     p - e2 * a * np.cos(theta)**3)
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    alt = p / np.cos(lat) - N

    # Convert to degrees
    lat_deg = np.degrees(lat)
    lon_deg = np.degrees(lon)
    
    return [lat_deg, lon_deg, alt]

def geodetic_to_eci(pos, theta_GST):
    """
    Convert geodetic coordinates (latitude, longitude, altitude) to ECI coordinates.
    
    Args:
        pos (list): Geodetic coordinates [latitude, longitude, altitude] in degrees and meters.
        theta_GST (float): Greenwich Sidereal Time (in radians).
        
    Returns:
        np.ndarray: ECI coordinates [x, y, z] in meters.
    """
    # Earth constants
    a = 6378137.0         # Equatorial radius (m)
    b = 6356752.3142      # Polar radius (m)
    e2 = 1 - (b**2 / a**2)

    # Convert degrees to radians
    lat = np.radians(pos[0])
    lon = np.radians(pos[1])

    # Prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

    # Compute ECEF coordinates
    x = (N + pos[2]) * np.cos(lat) * np.cos(lon)
    y = (N + pos[2]) * np.cos(lat) * np.sin(lon)
    z = ((1 - e2) * N + pos[2]) * np.sin(lat)
    r_ECEF = np.array([x, y, z])

    # Rotate ECEF to ECI using theta_GST
    R = np.array([
        [ np.cos(theta_GST), -np.sin(theta_GST), 0],
        [ np.sin(theta_GST),  np.cos(theta_GST), 0],
        [ 0,                  0,                 1]
    ])
    r_ECI = R @ r_ECEF

    return r_ECI


def interpolate_image(start, end, N):
    """
    Objective : 
    calculate the coordinates of the targets that we want to image between a coordinate A and another coordinate B
        where A is the initial position we want to target
        and B is the position where the imaging ends

    Return : 
    the lists of N targets that we want to image in ECI coordinates 

    Args :
        start (list): the coordinates of the first target position in geographic coordinates [lat, lon, alt] in degrees and meters
        end (list): the coordinates of the final target ground position in geographic coordinates [lat, lon, alt] in degrees and meters
        N (int): the number of steps we want to divide the imaging into
    """

    p0 = geodetic_to_cartesian(start[0], start[1])
    p1 = geodetic_to_cartesian(end[0], end[1])

    # Angle entre les deux points
    omega = np.arccos(np.clip(np.dot(p0, p1), -1.0, 1.0))

    points = []
    for i in range(N):
        f = i / (N - 1)
        sin_omega = np.sin(omega)
        if sin_omega == 0:
            p = p0  # les deux points sont les mêmes
        else:
            a = np.sin((1 - f) * omega) / sin_omega
            b = np.sin(f * omega) / sin_omega
            p = a * p0 + b * p1
            p /= np.linalg.norm(p)  # sécurité

        lat, lon = cartesian_to_geodetic(p)
        points.append((lat, lon, 6371. * 1000 ))
    
    return points

def quaternion_to_euler312(q):
    """Quaternion [x, y, z, w] → Euler 3-1-2 (yaw, roll, pitch)."""
    x, y, z, w = q
    # DCM
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y + z*w),         2*(x*z - y*w)],
        [    2*(x*y - z*w),   1 - 2*(x**2 + z**2),       2*(y*z + x*w)],
        [    2*(x*z + y*w),       2*(y*z - x*w),     1 - 2*(x**2 + y**2)]
    ])
    # Sequence 3-1-2
    yaw = math.atan2(R[0,1], R[0,0])
    roll = math.asin(-R[0,2])
    pitch = math.atan2(R[1,2], R[2,2])
    return np.array([yaw, roll, pitch])

def get_att(position, target):
    """
    Calculate the quaternion that rotates the satellite's position vector to point towards the target.
    Args:
        position (np.ndarray): The position vector of the satellite in ECI coordinates [x, y, z].
        target (np.ndarray): The target vector in ECI coordinates [x, y, z].
    Returns:
        np.ndarray: The quaternion [qx, qy, qz, qw] representing the rotation.
    """
    if position is None or target is None or np.linalg.norm(position) == 0 or np.linalg.norm(target) == 0:
        return np.zeros(3)
    
    dir_vec = target - position
    dir_vec /= np.linalg.norm(dir_vec)  # Normalize the direction vector

    nadir_axis = -position/np.linalg.norm(position)  # Nadir direction (towards Earth center)

    y_body = np.cross(dir_vec, nadir_axis)  # Y-axis in body frame (perpendicular to nadir and direction)
    y_body /= np.linalg.norm(y_body)

    x_body = np.cross(y_body, dir_vec)
    
    att = np.column_stack([x_body, y_body, dir_vec])  # Convert quaternion to Modified Rodrigues Parameters (MRP)
    

    return RigidBodyKinematics.C2MRP(att)

def calculate_quat_imaging(N, r, targets):
    """
    Objective : 
    calculate the orientation of the satellite needed during imaging between a coordinate A and another coordinate B
        where A is the initial position of the satellite
        and B is the position of the satellite after imaging

    Return : 
    the lists of N quaternions that allows the camera to go from A to B in N steps while staying on the same orbit

    Args :
        N (int): the number of steps we want to divide the imaging into
        r (list): the list of positions of the satellite in ECI coordinates [x, y, z] in meters corresponding to each step

    """
    if len(r) != len(targets):
        raise ValueError("The length of r and targets must be the same")
    
    list_quat = []
    for i in range(N):
        # Here we would calculate the quaternion for each step
        list_quat.append(get_att(r[i], targets[i]))
        pass

    return list_quat
    


def propagate_kepler(oe, dt, tol=1e-10, maxiter=100, mu=3.986004418e14):
    """
    Propagate classical orbital elements (oe) forward by dt seconds (elliptical case).
    oe must contain at least oe.a (m), oe.e (scalar), oe.f (true anomaly in rad).
    Returns (rN_new, vN_new, oe_new) where oe_new is a copy with updated f.
    """
    a = oe.a
    e = oe.e
    if abs(e) > 1e-12:
        raise ValueError("Cette fonction est pour e=0 uniquement")

    # angular velocity
    n = np.sqrt(mu / a**3)

    # true anomaly
    oe_new = type(oe)()
    for attr in oe.__dict__:
        setattr(oe_new, attr, getattr(oe, attr))
    oe_new.f = (oe.f + n * dt) % (2 * np.pi)

    # position and velocity in ECI frame
    rN_new, vN_new = orbitalMotion.elem2rv(mu, oe_new)
    return rN_new, vN_new, oe_new