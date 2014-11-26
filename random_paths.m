
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
lb = [-20;-20];
ub = [20;20];
n_obstacles = 20;
n_regions = 25;

% Generate obstacles
% from Michael's code
%function obs_map=gen_rand_obstacles(x_low,x_high,z_low,z_high,seedval)
x_low = lb(1);
x_high = ub(1);
z_low = lb(2);
z_high = ub(2);

%seedval = 11;
seedval = 99;


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


% Generate IRIS regions
start = [-20;0];
goal = [10;-20];
seeds = [...
         start';
         goal';
         ]';
        
num_steps = 25;
safe_regions = iris.util.auto_seed_regions(obstacle_pts, lb, ub, seeds, n_regions, num_steps, []);

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
  
 % Compute graph

 P = {};
num_regions = length(safe_regions);
% figure; hold on;
for k=1:num_regions
    P{k} = Polyhedron(safe_regions(k).A, safe_regions(k).b);
%     plot(P{k})
end

 % Check intersection of polyhedra and compute graph
tic
G = sparse(n_regions, n_regions);
for i=1:n_regions
    for j=i+1:n_regions
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
