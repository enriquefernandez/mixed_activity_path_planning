function ytraj = traj2d
% This function creates an interactive demonstration of the way we can combine
% IRIS with a mixed-integer program to solve path planning problems around
% obstacles. The figure which is created shows an environment with several
% obstacles (black) and several IRIS regions which have been seeded. Each IRIS
% region is shown by a green dot at its seed point, a red polytope, and a blue
% ellipsoid. In addition, a trajectory for a simple "UAV" model is shown in
% magenta. The UAV is modeled as a double integrator, and we plan a trajectory
% to minimize total acceleration over a discrete trajectory. You can interact
% with the plot by dragging the seed points of the IRIS regions and by
% dragging the start or end of the UAV path.
%
% You may notice that by default, the discrete poses of the UAV are all
% outside the obstacles (or, more accurately, inside the obstacle-free IRIS
% regions) but the straight-line path between them is not. To fix this, we can
% simply lift our representation of the obstacles into four dimensions: x, y,
% xdot, and ydot and require that the 4D pose of the UAV in that space be
% outside the lifted obstacles. To try this, just set dim to '4d'.
% 
% The mixed-integer program used to find the UAV path can be seen in the
% uav_path function at the end of this file.
%
% @param dim a string, either '2d' (default) or '4d'. '2d' causes the
%            obstacles to be represented only in x, y, which has the effect of ensuring
%            that the discrete poses of the UAV trajectory are outside the obstacles,
%            but not the straight-line path between those poses. '4d' uses a four-
%            dimensional representation of the obstacles  in x, y, xdot, ydot to ensure 
%            that the straight-line path is also obstacle-free.

MAX_VEL = 0.25;

lb = [-1; -1; -MAX_VEL; -MAX_VEL];
ub = [1; 1; MAX_VEL; MAX_VEL];

A_bounds = [-eye(2); eye(2)];
  b_bounds = [-lb(1:2); ub(1:2)];

 obstacle_pts = zeros(2,4,0);

obs = [-.2, -.2, 0, 0; -.2, .5, .5, -.6];

  obstacle_pts = cat(3, obstacle_pts, obs);

obs = [.4, .4, .6, .6; -.9, -0.2, -0.2, -0.9];

 obstacle_pts = cat(3, obstacle_pts, obs);
  
% obs = [.2, .2, .4, .4; .05, 1, 1, .05];
% 
%  obstacle_pts = cat(3, obstacle_pts, obs);

% obs = [.2, .2, .4, .4; -.05, -1, -1, -.05];
% 
%   obstacle_pts = cat(3, obstacle_pts, obs);


start = [-.75; 0.4];
goal = [0.8; -0.7];

% Choose the initial IRIS seed points
% seeds = [[-0.75; 0.3], [-0.1; .75], [-0.1;-.75], [.75;-0.3], [0.1;0.3], [0.3; 0]];
seeds = [[-0.6; 0.0], [-0.1;-.75], [.2;-0.5], [.4;0.6], [.8;-0.5]];

% Build the initial IRIS regions
safe_regions = struct('A', {}, 'b', {}, 'C', {}, 'd', {}, 'point', {});
for j = 1:size(seeds, 2)

    s = seeds(:,j);

  [A, b, C, d, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, s, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', s(1:2));
end

% v = uav_path(start, goal, safe_regions);

h = figure(1);
handles.start = [];
handles.goal = [];
handles.region_p = [];
handles.region_e = [];
handles.region_seed = [];
handles.traj = [];
handles.obs = [];
clf

function draw(start, goal, safe_regions, obstacle_pts, v)
  clf
  hold on
  for j = 1:size(obstacle_pts, 3)
    k = convhull(obstacle_pts(1,1:4,j), obstacle_pts(2,1:4,j));
    handles.obs(j) = patch(obstacle_pts(1,k,j), obstacle_pts(2,k,j), 'k');
  end


  for j = 1:length(safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A, safe_regions(j).b);
    V = V';
    V = V(1:2, convhull(V(1,:), V(2,:)));
    handles.region_p(j) = plot(V(1,:), V(2,:), 'Color', [.9,.3,.3], 'LineStyle', '-', 'LineWidth', 1);
    th = linspace(0,2*pi,100);
    y = [cos(th);sin(th)];
    x = bsxfun(@plus, safe_regions(j).C(1:2,1:2)*y, safe_regions(j).d(1:2));
    handles.region_e(j) = plot(x(1,:), x(2,:), 'Color', [.3,.3,.9], 'LineStyle', '-', 'LineWidth', 1);
    handles.region_seed(j) = plot(safe_regions(j).point(1), safe_regions(j).point(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
  end

  handles.start = plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 15);
  handles.goal = plot(goal(1), goal(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 15);

%   handles.traj = plot(v.x.value(1,:), v.x.value(2,:), 'mo-', 'LineWidth', 2);
  xlim([-1.05,1.05])
  ylim([-1.05,1.05])
end
draw(start, goal, safe_regions, obstacle_pts, []);
% 
% 
% 
% function p = minkowski_sum(a, b)
%   p = zeros(2, size(a,2) * size(b,2));
%   idx = 1;
%   for j = 1:size(a,2)
%     for k = 1:size(b,2)
%       p(:,idx) = a(:,j) + b(:,k);
%       idx = idx + 1;
%     end
%   end
% 
%   k = convhull(p(1,:), p(2,:));
%   assert(k(1) == k(end));
%   p = p(:,k(1:end-1));
% end

tic
[ytraj, cost, max_time] = findLinearTrajectory(safe_regions, start, goal, MAX_VEL)
fprintf('Found solution with cost %.3f and max time %.3f\n', cost, max_time);
tmax = ytraj.tspan(2);
t = 0:0.01:tmax;
xtr = ytraj.eval(t);
hold on
plot(xtr(1,:), xtr(2,:), 'k--')
toc

tic
poly_deg = 3;
% num_seg = 10;
num_seg = 7;
ytraj = findMixedTrajectory(safe_regions, start, goal, lb, ub, [], poly_deg, num_seg, [])
tmax = ytraj.tspan(2);
t = 0:0.01:tmax;
xtr = ytraj.eval(t);
hold on
plot(xtr(1,:), xtr(2,:), 'g')
toc


% Play with Polyhedra
P = {};
num_regions = length(safe_regions);
% figure; hold on;
for k=1:num_regions
    P{k} = Polyhedron(safe_regions(k).A, safe_regions(k).b);
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
[C, P] = apsp(G)
full(G)
extractAPSPpath(P, 4, 1)
% graphallshortestpaths(G)

end
