def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    
    import math  # This uses a lot of math.
    
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering = abs(params['steering_angle']) # Only need the absolute steering angle
    current_speed = params['speed']
    closest_waypoints_indices = params['closest_waypoints']
    next_waypoint_index = closest_waypoints_indices[1]
    waypoints = params['waypoints']
    car_pos = (params['x'], params['y'])
    speed = params['speed']
    heading = params["heading"]
    is_offtrack = params['is_offtrack']
    is_reversed = params['is_reversed']

    # Punish harshly for very bad behavior
    if is_offtrack or is_reversed:
        return float(0)
    
    # At the very least, head towards the next waypoint
    target_waypoint = waypoints[next_waypoint_index]
    
    # Keep the maximum distance between the mid-waypoints and the target line.
    # This is a proxy for how intense the curve is.
    max_waypoint_distance_from_target_line = 0
    # index_of_maximum_distance_waypoint = 0
    
    # Assume that the next waypoint is in front of the car.
    # Also assume that the line between the car and the next waypoint is always valid.
    # Iterate over the waypoint AFTER the next waypoint up to 4 after the next waypoint
    for idx in range(next_waypoint_index + 2, next_waypoint_index + 7):
        
        mod_idx = idx % len(waypoints)
        
        
        b = -1
        
        # Slope between car and the point we are evaluating
        a = (car_pos[1] - waypoints[mod_idx][1]) / (car_pos[0] - waypoints[mod_idx][0])
        
        # Y intercept
        #c = car_pos[1] - (b * car_pos[0])
        c = -(car_pos[0] * (a) + car_pos[1])
        
        # For each point between the next point and the curent point, 
        # See if the distance between the the point and the line we are testing
        # Is greater than a threshold that will determine if we are or are not on the road.
        line_invalid = False
        for test_pt_idx in range(next_waypoint_index, idx):
            
            test_pt = waypoints[test_pt_idx % len(waypoints)] 
            dist_to_line = abs((a * test_pt[0]) + (b * test_pt[1]) + c) / math.sqrt(a**2 + b**2)
            
            # Check if the line is invalid
            if dist_to_line > 0.5 * track_width:
                line_invalid = True
                break
            
            # Keep track of the waypoint that is furthest from the line
            if dist_to_line > max_waypoint_distance_from_target_line:
                max_waypoint_distance_from_target_line = dist_to_line
                # index_of_maximum_distance_waypoint = test_pt_idx % len(waypoints)
            
                
        if line_invalid:
            target_waypoint = idx-1 % len(waypoints)
            break
            
            
    # Reward based on difference in heading angle and target line angle
    target_line_angle = math.degrees(math.atan(a))
    heading_actual = heading
    if heading < 0:
        heading_actual = heading + 360
            
    # angle_diff = abs(target_line_angle - heading_actual)
    angle_diff = target_line_angle - heading_actual
    
    # Calculate reward based on how close the car is to being on the heading of the target line
    # angle_reward = 1 - (angle_diff / 180)
    angle_reward = 1 - ((angle_diff/180)**2)
    
    # The maximum speed of the car, as defined in the car's action space.
    MAX_SPEED = 2.5

    # Determine if we are going straight or curving
    if max_waypoint_distance_from_target_line < 0.1 * track_width:
        
        speed_reward = (speed / MAX_SPEED)**0.5
        
    # Else if we are on a curve
    else:
        speed_norm = speed / MAX_SPEED
        # Generate a curve based on the normalized speed that peaks at about 75% max speed.
        #speed_reward = (-8 * (speed_norm**3)) + (9.3714 * (speed_norm ** 2)) + (-1.4714 * speed_norm) + 0.1214
        # Generate a curve that peaks at 50% max speed. Should work well with speed granularity 2.
        speed_reward = (8.1 * (speed_norm ** 3)) + (-14.4 * (speed_norm ** 2)) + (6 * (speed_norm)) + 0.2
        # speed_reward = (-4 * (speed_norm**2)) + (4 * speed_norm)
        # speed_reward *= 0.75
        
    # Model validation sometimes generates a math domain warning for the final 
    # reward calculation unless these lines are here.
    if speed_reward < 0:
        speed_reward = 0
    if angle_reward < 0:
        angle_reward = 0
        
    # Calculate final reward.
    # Reward is 1 part speed, 2 parts heading.
    # reward = math.sqrt((speed_reward + (2 * angle_reward)) / 3)
    reward = ((speed_reward*angle_reward)**0.5)

    return float(reward)