function random_paths

h = figure(1);
handles.start = [];
handles.goal = [];
handles.region_p = [];
handles.region_e = [];
handles.region_seed = [];
handles.traj = [];
handles.obs = [];
clf

% Configuration
start = [-17;0];
% goal = [16;-15];
% goal_iregion_idx = 2;

goal = [16;18];
goal_iregion_idx = 3;

% goal = [-16;0.8];

MAX_VEL = 0.25;

lb = [-20; -20; -MAX_VEL; -MAX_VEL];
ub = [20; 20; MAX_VEL; MAX_VEL];
% lb = [-20;-20];
% ub = [20;20];
n_obstacles = 20;
n_regions = 20;
% n_obstacles = 5;
% n_regions = 5;
MAX_VEL = 0.25;

% Generate obstacles
% from Michael's code
%function obs_map=gen_rand_obstacles(x_low,x_high,z_low,z_high,seedval)
x_low = lb(1);
x_high = ub(1);
z_low = lb(2);
z_high = ub(2);

%seedval = 11;
seedval = 99;
% seedval = 37

%Obstacle parameters
OBSTACLE_DENSITY=1.2; %obstacles per 1 unit in z
NUM_OBSTACLES=n_obstacles;
%NUM_OBSTACLES=4;
OBS_MIN_EDGE=4;
OBS_MAX_EDGE=4;
OBS_MIN_DIM=1.5;
OBS_MAX_DIM=3.2;

obstacle_pts = zeros(2,4,n_obstacles);

% Generate random obstacles
S = RandStream('mt19937ar','Seed',seedval);
obs_pos=[S.rand(1,NUM_OBSTACLES)*(x_high-x_low)+x_low; S.rand(1,NUM_OBSTACLES)*(z_high-z_low)+z_low];
obs_num_edges=S.randi(OBS_MAX_EDGE-OBS_MIN_EDGE+1,1,NUM_OBSTACLES)+OBS_MIN_EDGE-1;

for k=1:NUM_OBSTACLES
    obs_angles=S.rand(1,obs_num_edges(1,k)); %Random angles between rays
    obs_angles=obs_angles*2*pi/sum(obs_angles);
    for n=2:length(obs_angles)
        obs_angles(n)=obs_angles(n-1)+obs_angles(n);
    end
    obs_angles=obs_angles+2*pi*S.rand; %Randomize the absolute orientation
    obs_lengths=S.rand(1,obs_num_edges(1,k))*(OBS_MAX_DIM-OBS_MIN_DIM)+OBS_MIN_DIM;
    obs_coords=[cos(obs_angles).*obs_lengths + obs_pos(1,k); sin(obs_angles).*obs_lengths + obs_pos(2,k)];
    obs_coords=obs_coords(:,convhull(obs_coords(1,:)',obs_coords(2,:)'));
    

    % create the obstacle object
    obstacle_pts(:,:, k) = obs_coords(:,1:OBS_MIN_EDGE);

end

clf
hold on
for j = 1:size(obstacle_pts, 3)
k = convhull(obstacle_pts(1,1:4,j), obstacle_pts(2,1:4,j));
handles.obs(j) = patch(obstacle_pts(1,k,j), obstacle_pts(2,k,j), 'k');
end

%% Generate random or fixed IRIS regions

fixed_IRIS_regions = 0;
if fixed_IRIS_regions

    seeds = [start, [0;-5], [-5;-17], goal];
    A_bounds = [-eye(2); eye(2)];
    b_bounds = [-lb(1:2); ub(1:2)];

    % Build the initial IRIS regions
    safe_regions = struct('A', {}, 'b', {}, 'C', {}, 'd', {}, 'point', {});
    for j = 1:size(seeds, 2)

        s = seeds(:,j);

      [A, b, C, d, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, s, struct('require_containment', true, 'error_on_infeas_start', true));
      safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', s(1:2));
    end
else
    
    % Generate IRIS regions
    
    seeds = [...
             start';
             goal';
             ]';

    num_steps = 25;
    safe_regions = iris.util.auto_seed_regions(obstacle_pts, lb(1:2), ub(1:2), seeds, n_regions, num_steps, []);

    
end


 for j = 1:length(safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A, safe_regions(j).b);
    V = V';
    V = V(1:2, convhull(V(1,:), V(2,:)));
    handles.region_p(j) = plot(V(1,:), V(2,:), 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
    th = linspace(0,2*pi,100);
%     y = [cos(th);sin(th)];
%     x = bsxfun(@plus, safe_regions(j).C(1:2,1:2)*y, safe_regions(j).d(1:2));
%     handles.region_e(j) = plot(x(1,:), x(2,:), 'Color', [.3,.3,.9], 'LineStyle', '-', 'LineWidth', 1);
    handles.region_seed(j) = plot(safe_regions(j).point(1), safe_regions(j).point(2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
 end
 

 
 %%
 % Compute graph

 P = {};
num_regions = length(safe_regions);
% figure; hold on;
for k=1:num_regions
    P{k} = Polyhedron(safe_regions(k).A, safe_regions(k).b);
    safe_regions(k).P = P{k};
%     plot(P{k})
end

 % Check intersection of polyhedra and compute graph
tic
G = sparse(num_regions, num_regions);
for i=1:num_regions
    for j=i+1:num_regions
        intersection = P{i}.intersect(P{j});
        if ~intersection.isEmptySet()
            fprintf('Region %i intersects with region %i\n',i,j);
            G(i,j) = 1;
            G(j,i) = 1;
        else
            fprintf('Region %i DOESNT intersect with region %i\n',i,j);
        end
    end
end
toc

% APSP
tic
[C, paths] = apsp(G)
toc
extractAPSPpath(paths, 1, 2)
% graphallshortestpaths(G)


% Define regions of interest (as Polyhedron)
interest_regions = {}

interest_regions{1} = Polyhedron([-19 -2;-19 2;-15 2;-15 -2]);
interest_regions{2} = Polyhedron([15 -16;15 -13;18 -13;18 -16]);
interest_regions{3} = Polyhedron([15 17;15 19;17 19;17 17]);
interest_regions{4} = Polyhedron([-6 17.5;-6 19.5;-4 19.5;-4 17.5]);

n_interest = length(interest_regions);

interest_to_safe = zeros(n_interest,1);

for k=1:length(interest_regions)
    % Plot
    V = interest_regions{k}.V;
    V = [V; V(1,:)];
    handles.i_region(k) = plot(V(:,1), V(:,2), 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5);
    
    % Find best safe region
    best_vol = 0;
    for j=1:num_regions
        intersection = safe_regions(j).P.intersect(interest_regions{k});
        intersection_vol = intersection.volume ;
        if intersection_vol > best_vol
            best_vol = intersection_vol;
            interest_to_safe(k) = j;
        end
    end
end

% interest_to_safe

% Define point of interest
for j=1:num_regions
        if safe_regions(j).P.contains(start)
            point_region = j;
            V = safe_regions(j).P.V;
            V = [V; V(1,:)];
            V = V(convhull(V), :);
            handles.p_region(1) = plot(V(:,1), V(:,2), 'Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5);
            break;
        end
end

%%

% Calculate path from interest point to interest region

tic
safe_reg_path = extractAPSPpath(paths, point_region, interest_to_safe(goal_iregion_idx))

for i=1:length(safe_reg_path)
    reg_idx = safe_reg_path(i);
    V = safe_regions(reg_idx).P.V;
    V = [V; V(1,:)];
    V = V(convhull(V), :);
    handles.p_region(1) = plot(V(:,1), V(:,2), 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.0);
end

% Calculate line path

goal_point = mean(interest_regions{goal_iregion_idx}.V)';


[ytraj, cost, max_time] = findLinearTrajectory(safe_regions(safe_reg_path), start, goal, MAX_VEL)
fprintf('Found solution with cost %.3f and max time %.3f\n', cost, max_time);
tmax = ytraj.tspan(2);
t = 0:0.01:tmax;
xtr = ytraj.eval(t);
hold on
plot(xtr(1,:), xtr(2,:), 'g--', 'LineWidth', 1.5)
toc

%%
% SOS path
disp('Looking for SOS path...');
% endg=[-5;-5];
endg=[16.5;-14.5];
tic
poly_deg = 3;
num_seg = 10;
ytraj = findMixedTrajectory(safe_regions(safe_reg_path), start, goal, lb, ub, [], poly_deg, num_seg, [])
tmax = ytraj.tspan(2);
t = 0:0.01:tmax;
xtr = ytraj.eval(t);
hold on
plot(xtr(1,:), xtr(2,:), 'm')
toc

end