# #
# # Python file filled with different useful functions
# #
# # Purpose:  Implementation of tools that can be used to have the simulation


# # def julian_day(date):
# #     """Computes the Julian Day (JD) from a UTC datetime."""
# #     a = (14 - date.month) // 12
# #     y = date.year + 4800 - a
# #     m = date.month + 12*a - 3
# #     jdn = date.day + (153*m + 2)//5 + 365*y + y//4 - y//100 + y//400 - 32045
# #     jd = jdn + (date.hour - 12)/24 + date.minute/1440 + date.second/86400
# #     return jd

# def greenwich_sidereal_angle_deg(date_utc):
#     """Computes the Greenwich Apparent Sidereal Time (GAST) in degrees."""
#     # 1. Compute the Julian Day
#     jd = julian_day(date_utc)
    
#     # 2. Julian centuries since J2000.0
#     T = (jd - 2451545.0) / 36525.0
    
#     # 3. Greenwich Mean Sidereal Time (GMST) in degrees
#     # Using the IERS Conventions 2003 formula
#     gmst_deg = 280.46061837 + 360.98564736629*(jd - 2451545.0) + 0.000387933*T**2 - T**3/38710000.0
    
#     # 4. Nutation correction (simplified)
#     # Mean longitude of the Sun (L) and the Moon (Lp)
#     L = math.radians(280.4665 + 36000.7698*T)
#     Lp = math.radians(218.3165 + 481267.8813*T)
    
#     # Nutation in longitude (Δψ) in degrees
#     delta_psi = -17.20*math.sin(125.04 - 1934.136*T) / 3600  
    
#     # 5. Apparent sidereal time (GAST) in degrees
#     epsilon = 23.4393 - 0.0130*T  # Obliquity of the ecliptic
#     gast_deg = gmst_deg + delta_psi*math.cos(math.radians(epsilon))
    
#     # Normalize to range [0°, 360°)
#     gast_deg %= 360
    
#     return gast_deg








# def initialCoordinateTraject(altitude, longitude1, latitude1, longitude2, latitude2):
#     """
#     The function calculate the initial arguments needed in the run function to start the simulation 
#     with the satellite situated at a wanted position (given with its geographic coordinates) 
#     and a chosen destination which is supposed to be close and thus, 
#     the difference between longitudes and latitudes of the initial location and the destination 
#     must be less than 90 degrees :

#     Args:
#         altitude (float): Not really needed in the calculations but necessary to initialize the simulation
#         longitude1 (float) : the longitude of the wanted initial position
#         latitude1 (float) : the latitude of the wanted initial position
#         longitude2 (float) : the longitude of the wanted destination
#         latitude2 (float) : the latitude of the wanted destination

#     """
#     down = False
        
#     if (latitude2-latitude1)<0 :
#         down = True


    
#     r1 = geodetic_to_cartesian(latitude1, longitude1)
#     r2 = geodetic_to_cartesian(latitude2, longitude2)

#     n = np.cross(r1, r2)
#     norm_n = np.linalg.norm(n)
#     if norm_n == 0:
#         raise ValueError("identicals points or antipodes")
#     n_unit = n / norm_n

#     print(r1)
#     print(r2)

#     # Inclinaison = angle entre la normale au plan orbital et l'axe z
#     inclinate = math.acos(abs(n_unit[2])) * macros.R2D

#     if n_unit[2]:
#         inclinate = - inclinate


#     return initialCoordinate(altitude, longitude1, latitude1, inclinate)

# def geodetic_to_cartesian(lat_deg, lon_deg):
#     lat = lat_deg * macros.D2R
#     lon = lon_deg * macros.D2R
#     x = math.cos(lat) * math.cos(lon)
#     y = math.cos(lat) * math.sin(lon)
#     z = math.sin(lat)
#     return np.array([x, y, z])

# def displayQuaternions():
#     return



# def rotate_vector_in_body_frame(vector, quaternion, position, velocity):
#         """
#         Rotates a vector using a quaternion in a body frame defined as:
#         - X: position (normalized)
#         - Y: perpendicular to position and velocity (right-hand rule)
#         - Z: velocity (normalized)        Parameters:
#             vector (np.ndarray): Vector in ECI frame to rotate.
#             quaternion (array-like): Quaternion [x, y, z, w] for rotation (body frame).
#             position (np.ndarray): Position vector (defines X axis).
#             velocity (np.ndarray): Velocity vector (defines Z axis).        Returns:
#             rotated_vector_in_eci (np.ndarray): Rotated vector in ECI frame.
#             x_body (np.ndarray): X axis of body frame.
#             y_body (np.ndarray): Y axis of body frame.
#             z_body (np.ndarray): Z axis of body frame.
#         """
#         # Step 1: Construct body frame basis
#         x_body = np.linalg.norm(position)  # X-axis
#         z_body = np.linalg.norm(velocity)  # Y-axis
#         y_body = np.linalg.norm(np.cross(z_body, x_body))  # Recompute Y to ensure orthogonality     

#         # Transformation matrix (body frame to ECI frame)
#         body_to_eci = np.array([x_body, y_body, z_body]).T  # Basis vectors as columns        
        
#         # Step 2: Transform the vector to the body frame
#         eci_to_body = np.linalg.inv(body_to_eci)  # Inverse of the transformation
#         vector_in_body_frame = np.dot(eci_to_body, vector)        
        
#         # Step 3: Rotate the vector in the body frame
#         rotation = math.from_quat(quaternion)  # Create rotation object
#         rotated_vector_in_body_frame = rotation.apply(vector_in_body_frame)        
        
#         # Step 4: Transform the rotated vector back to the ECI frame
#         rotated_vector_in_eci = np.dot(body_to_eci, rotated_vector_in_body_frame)        
#         return rotated_vector_in_eci, x_body, y_body, z_body  # Also return body basis