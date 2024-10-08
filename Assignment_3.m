function vel = swarm(rad_rep, rad_ori, rad_att, x, N, dxi)
% FILE: swarm.m implements a Boids-like behavior
%
% DESCRIPTION:
% Boids-like repulsion-orientation-attraction behavior based loosely on the 
% behavior described by Couzin et al. in the Collective Memory paper. 
%
% INPUTS:
% rad_rep - radius of repulsion
% rad_ori - radius of orientation
% rad_att - radius of attraction
% x - matrix containing the pose of all the robots; x(1, ii) is the
% position of robot ii along the horizontal axis; x(2, ii) is the position
% of robot ii along the vertical axis; x(3, ii) is the heading of robot ii
% in radians. Easier alternative to dealing with radians is to use
% dxi(:,ii) instead, which is the heading or velocity of robot ii, as a
% vector
% blind_neighbors - matrix tracking the robots in a robot's blind spot
% blind_neighbors not used in the Assignment 
% neighbors - NxN matrix; entry (ii, jj) is 1 if agents ii and jj are
% neighbors; otherwise, entry is 0
% neighbors not used in the Assignment
% N - the number of robots in the swarm
% dxi - the current velocity of the robots (2 x N vector); dxi(1, ii) is
% robot ii's velocity component along the horizontal axis, while dxi(2, ii) 
% is robot ii's velocity component along the vertical axis 
%
% OUTPUTS:
% vel - the resulting velocity of the robots (2 x N vector)
%
% TODO:
% Return the velocity (i.e., heading) that emerges from implementing 
% repulsion, orientation, and attracton interaction rules

%% Authors: Aidan Alme
%%%%%%%%%%%%%

% dist(ii, jj) is the distance between robots ii and jj
dist = distances_from_others(x, N);
% temp_dxi is where the resultant vectors are stored
temp_dxi = dxi;

% Update temp_dxi for each robot
for i = 1:N
    % Gather information from each other robot
    % dx is the total difference in the x direction
    dx = 0;
    % dy is the total difference in the y direction
    dy = 0;
    % mag is the magnitude of the resultant vector post averaging
    mag = 1;
    % the number of neighbors encountered
    num_neighbors = 0;
    for j = 1:N
        % A robot cannot respond to itself
        if i ~= j
            % Repulsion
            if dist(i, j) <= rad_rep
                dx = dx + x(1, i) - x(1, j);
                dy = dy + x(2, i) - x(2, j);
                num_neighbors = num_neighbors + 1;
            % Orientation
            elseif dist (i, j) <= rad_ori
                dx = dx + dxi(1, j);
                dy = dy + dxi(2, j);
                num_neighbors = num_neighbors + 1;
            % Attraction
            elseif dist (i, j) <= rad_att
                dx = dx + x(1, j) - x(1, i);
                dy = dy + x(2, j) - x(2, i);
                num_neighbors = num_neighbors + 1;
            % Neighbor is not in any of the radii
            else
                break;
            end
        end
    end
    % Change dxi if the change is not zero
    if dx ~= 0 || dy ~= 0
        % Average the vector
        dx = dx / num_neighbors;
        dy = dy / num_neighbors;
        % Normalize the vector
        mag = sqrt(dx^2 + dy^2);
        temp_dxi(1, i) = dx / mag;
        temp_dxi(2, i) = dy / mag;
    end
    
end

% Return the resulting velocity
vel = temp_dxi;

end


