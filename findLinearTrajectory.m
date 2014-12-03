function [ytraj, cost, max_t]= findLinearTrajectory(safe_regions_seq, start, goal, vmax)

% Start and end can either be 2x1 vectors (contained in the last and end
% safe regions), or Polyhedrons (areas that need to be reached).
% safe_regions_seq ordered list of intersecting regions from start to goal



% Define Yalmip variables
% these are the points in the trajectory
dim = 2;
num_segments = length(safe_regions_seq);

num_points = num_segments + 1;

X = {};
for i=1:num_points
    X{i} = sdpvar(dim,1);
end

% Add constraints
C = [];


% Initial and end positions or regions
% C = [C, X{1} == start];
% C = [C, X{num_points} == goal];
C = addBoundaryConstraint(C, X{1}, start);
C = addBoundaryConstraint(C, X{num_points}, goal);

% Points along trajectory need to be in two consecutive regions
for i=2:num_points-1
    Am = safe_regions_seq(i-1).A;
    bm = safe_regions_seq(i-1).b;
    A = safe_regions_seq(i).A;
    b = safe_regions_seq(i).b;
    
    C = [C, Am * X{i} <= bm];
    C = [C, A * X{i} <= b];
    
end

% Quadratic objective (min length of segments)
obj = 0;

for i=2:num_points
    difference = X{i} - X{i-1};
    obj = obj + difference' * difference;
end

% Solve QP
options = sdpsettings('verbose',1,'solver','gurobi');

sol = optimize(C, obj, options);

points = zeros(2, num_points);
solution = value(X);

for i=1:num_points
    points(:,i) = solution{i};
end

% hold on
% plot(points(1,:), points(2,:),'k')

% Find trajectory
breaks = zeros(num_points,1);
coeffs = zeros([dim, length(breaks)-1, 2]);
prev_t = 0;
for i=1:num_segments
    t_segment = norm(points(:,i+1) - points(:,i)) / vmax;
    breaks(i+1) = t_segment + prev_t;
    
    coeffs(:,i,1) = (points(:, i+1) - points(:, i)) / t_segment;
    % free coefficient
    coeffs(:,i,2) = points(:, i);
    
    prev_t = breaks(i+1);
end


ytraj = PPTrajectory(mkpp(breaks, coeffs, dim));
cost = sqrt(value(obj));
max_t = breaks(num_points);


    function C = addBoundaryConstraint(C, opt_var, bound_condition)
        if isa(bound_condition, 'Polyhedron')
            C = [C, bound_condition.A * opt_var <= bound_condition.b];
        elseif isa(bound_condition, 'double')
            C = [C, opt_var == bound_condition];
        else
            error('Start and end can only be either arrays or Polyhedrons');
        end
    end

end
