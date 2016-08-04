% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin'; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

% N = size(pose,2);
% for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
%     
% 
%     % Find occupied-measurement cells and free-measurement cells
%    
% 
%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
% end

N = size(pose,2);
M = size(ranges,1);
res = myResol;

% allocating
angles = zeros(size(scanAngles));
robot_pos = zeros(M,2); 
robot_pos_ind = zeros(M,2); 
loc = zeros(M,2); 
loc_ind = zeros(M,2); 
map_origin = repmat(myorigin,M,1);

for i = 1:N
    
    % determine hit location
    r = ranges(:,i);
    angles = pose(3,i) + scanAngles;
    robot_pos = repmat(pose(1:2,i)',M,1);
    robot_pos_ind_temp = ceil(res*pose(1:2,i))';
    robot_pos_ind = repmat(robot_pos_ind_temp,M,1) + map_origin;
    
    loc = [r.*cos(angles), -r.*sin(angles) ] + robot_pos;
    loc_ind = ceil(res*loc ) + map_origin;
    
    for j = 1:M;
        occ = loc_ind(j,:);
        orig = robot_pos_ind(1,:);
        [fx, fy ] = bresenham2(orig(1),orig(2),occ(1),occ(2)); 
        if all(flip(occ) <= size(myMap));
            myMap(occ(2),occ(1))   = myMap(occ(2),occ(1)) + lo_occ;
        end
        
        if (all(fx <= size(myMap,1)) && all(fy <= size(myMap,2)));            
            free_ind = sub2ind(size(myMap),fy,fx);
            myMap(free_ind) = myMap(free_ind) - lo_free;
        end
        
        
    end
    
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;
    
    
end

% % This script is to show how to use bresenham.
% map = zeros(30,30);
% orig = [10,5]; % start point
% occ = [20,20]; % end point 
% % get cells in between
% [freex, freey] = bresenham(orig(1),orig(2),occ(1),occ(2));  
% % convert to 1d index
% free = sub2ind(size(map),freey,freex);
% % set end point value 
% map(occ(2),occ(1)) = 3;
% % set free cell values
% map(free) = 1;

end

