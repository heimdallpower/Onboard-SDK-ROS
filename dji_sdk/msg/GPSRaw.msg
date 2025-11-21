std_msgs/Header header
float64 lat             # latitude  [rad]
float64 lon             # longitude [rad]
float64 alt             # altitude  [m]
float64 var_pos_hor     # variance estimate of horizontal position  [m^2]
float64 var_pos_vert    # variance estimate of vertical position    [m^2]

float64 vel_e           # East velocity     [m/s]
float64 vel_n           # North velocity    [m/s]
float64 vel_u           # Up velocity       [m/s]
float64 var_speed       # variance estimate of speed [(m/s)^2]

float64 hdop            # Horizontal dilution of precision  [m]
float64 pdop            # Position dilution of precision    [m]

float64 fix

uint32 num_visible_sats
