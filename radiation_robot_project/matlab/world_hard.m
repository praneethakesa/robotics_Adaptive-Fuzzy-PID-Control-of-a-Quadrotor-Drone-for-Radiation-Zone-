function W = world_hard()
% Dense obstacle field + height mounds + rubble patches + a U-shaped DEAD END (goal is behind it).
obs = [2.5 1.2 .5; 3.5 -1.8 .5; 5.5 3.4 .6; 7.5 -3.5 .6; ...
       9.5 1.6 .5; 10.5 -2.0 .5; 11.5 0.4 .5; 12.5 2.6 .6; 12.8 -1.4 .5; 13.5 1.2 .5];
for yy = -2.7:0.9:2.71, obs(end+1,:) = [20.0 yy .5]; end             %#ok<AGROW>  back wall of the U
for xx = [19.1 18.2 17.3 16.4]                                       % two arms of the U (opening faces the robot)
    obs(end+1,:) = [xx  2.7 .5]; obs(end+1,:) = [xx -2.7 .5];        %#ok<AGROW>
end
W.obstacles  = obs;                                                    % x y radius
W.waypoints  = [4.5 0; 9.0 -0.4; 14.5 -0.5; 24.0 0];                   % goal is behind the U
W.leaks      = [9.4 -1.2; 14.6 0.9; 23.2 0.6];
W.slip_zones = [6 9; 15 18];
% height mounds: x, y, height h, width s   z = h exp(-r^2/(2 s^2)); last one is a steep pile (slope ~0.42)
W.mounds = [3.0 -0.3 .20 1.0; 9.5 -0.3 .20 1.0; 13.0 -0.4 .20 1.0; 6.5 0.2 .55 0.8];
end
