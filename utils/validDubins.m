% Checks if valid Dubins paths can be generated between the waypoints
num_waypoints = size(waypoints,1);
for i = 2:num_waypoints
    dubins(i-1) = isempty(dubinsParameters(waypoints(i-1,1:4),waypoints(i,1:4),P.R_min));
end
valid = ~any(dubins)
