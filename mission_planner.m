%% Plotting stuff
h = figure(1); 
handles.start = [];
handles.goal = [];
handles.region_p = [];
handles.region_e = [];
handles.region_seed = [];
handles.traj = [];
handles.obs = [];
clf
hold on; %axis equal

%% Mission Setup
seedval = 63 ;
MAX_VEL = 0.25;
lb = [-20; -20; -MAX_VEL; -MAX_VEL];
ub = [20; 20; MAX_VEL; MAX_VEL];
num_random_obs = 20;
n_regions = 20;

%Obstacle parameters
OBSTACLE_DENSITY=1.2; %obstacles per 1 unit in z
%NUM_OBSTACLES=4;
OBS_MIN_EDGE=4;
OBS_MAX_EDGE=4;
OBS_MIN_DIM=1.0;
OBS_MAX_DIM=2;
x_low = lb(1)+4;
x_high = ub(1)-4;
z_low = lb(2)+4;
z_high = ub(2)-4;

xlim(1.2*[lb(1) ub(1)])
xlim(1.2*[lb(2) ub(2)])

plot([lb(1) lb(1) ub(1) ub(1) lb(1)],[lb(2) ub(2) ub(2) lb(2) lb(2)], 'Color', 'k', 'LineStyle', '-', 'LineWidth', 3);

%% Define obstacles and interest regions

% Interest regions
interest_regions = containers.Map;
interest_regions('opcenter') = Polyhedron([-18 -18; -18 -15; -15 -15; -15 -18]);
interest_regions('recharge1') = Polyhedron([2 -1; 2 0; 3 0; 3 -1]);
interest_regions('recharge2') = Polyhedron([17 11; 17 12; 18 12; 18 11]);
interest_regions('a') = Polyhedron([-17 15; -17 17; -14 17; -14 15]);
interest_regions('b') = Polyhedron([-1 -19; -1 -17; 2 -17; 2 -19]);
interest_regions('c') = Polyhedron([9 13; 9 16; 10 16; 10 13]);
interest_regions('d') = Polyhedron([17 -2; 17 1; 18 1; 18 -2]);
interest_regions('e') = Polyhedron([17 -12; 17 -10; 19 -10; 19 -12]);
hold on;
% Plot regions
k = interest_regions.keys;
v = interest_regions.values;
interest_reg_means = zeros(length(v), 2);
for i=1:length(v)
    V = v{i}.V;
    m = mean(V);
    interest_reg_means(i,:) = m;
    V = [V; V(1,:)];    
    plot(V(:,1), V(:,2), 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1.5);
    text(m(1),m(2), k{i}, 'FontSize', 18, 'HorizontalAlignment', 'center', 'Color', 'b');
    fprintf('%s area plotted.\n',k{i});
end

% Obstacles
obstacle_pts = [];

% Fixed obstacles
obstacle_pts(:,:,1) = [-19 -14.8; -19.5 10; -16 8; -13 -14]';
obstacle_pts(:,:,2) = [9 -19; 9 -14; 19 -14; 19 -19]';
obstacle_pts(:,:,3) = [9 -14; 9 -7; 12 -7; 12 -14]';
obstacle_pts(:,:,4) = [15 -7; 15 -5; 18 -5; 18 -7]';

num_fixed_obs = size(obstacle_pts,3);

% Generate random obstacles
S = RandStream('mt19937ar','Seed',seedval);
obs_pos=[S.rand(1,num_random_obs)*(x_high-x_low)+x_low; S.rand(1,num_random_obs)*(z_high-z_low)+z_low];
obs_num_edges=S.randi(OBS_MAX_EDGE-OBS_MIN_EDGE+1,1,num_random_obs)+OBS_MIN_EDGE-1;

for k=1:num_random_obs
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
    obstacle_pts(:,:, k+num_fixed_obs) = obs_coords(:,1:OBS_MIN_EDGE);

end

% Plot obstacles
for j = 1:size(obstacle_pts, 3)
k = convhull(obstacle_pts(1,1:4,j), obstacle_pts(2,1:4,j));
handles.obs(j) = patch(obstacle_pts(1,k,j), obstacle_pts(2,k,j), 'k');
end

%% Compute safe IRIS regions
% writerObj = VideoWriter('videos/safe_regions.mpeg');
% writerObj.FrameRate = 1;
% open(writerObj);

seeds = [interest_reg_means' [14 -6]'];

num_steps = 25;
safe_regions = iris.util.auto_seed_regions(obstacle_pts, lb(1:2), ub(1:2), seeds, n_regions, num_steps, []);

% Plot regions
for j = 1:length(safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A, safe_regions(j).b);
    V = V';
    V = V(1:2, convhull(V(1,:), V(2,:)));
    handles.region_p(j) = plot(V(1,:), V(2,:), 'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.5);
    th = linspace(0,2*pi,100);
%     frame = getframe(gcf);
%     figinfo = hardcopy(figure(1),'-dzbuffer','-r0');
%     writeVideo(writerObj, im2frame(figinfo));
%     writeVideo(writerObj,frame);
%     export_fig(sprintf('figures/iris_regions/iris_%02i.pdf', j))
    pause();
    %     y = [cos(th);sin(th)];
    %     x = bsxfun(@plus, safe_regions(j).C(1:2,1:2)*y, safe_regions(j).d(1:2));
    %     handles.region_e(j) = plot(x(1,:), x(2,:), 'Color', [.3,.3,.9], 'LineStyle', '-', 'LineWidth', 1);
%     handles.region_seed(j) = plot(safe_regions(j).point(1), safe_regions(j).point(2), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
end
% close(writerObj);

%% Compute safe regions graph and APSP

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

% Find mapping from interest regions to safe regions
k = interest_regions.keys;
v = interest_regions.values;
interest_to_safe = containers.Map;
for i=1:length(v)
    region_name = k{i};
    % Find best safe region
    best_vol = 0;
    for j=1:num_regions
        intersection = safe_regions(j).P.intersect(v{i});
        intersection_vol = intersection.volume ;
        if intersection_vol > best_vol
            best_vol = intersection_vol;
            interest_to_safe(region_name) = j;
        end
    end
end


%% Find all pairs linear trajectories: distances + time
% writerObj = VideoWriter('videos/allpairs_lines.mpeg');
% writerObj.FrameRate = 5;
% open(writerObj);

cost_matrix = {};
k = interest_regions.keys;
v = interest_regions.values;
plot_count=1;
for i=1:length(v)
    region_name1 = k{i};
    region1 = v{i};
    for j=i:length(v)
        region2 = v{j};
        region_name2 = k{j};
        if ~(region1==region2)
           safe_reg_path = extractAPSPpath(paths, interest_to_safe(region_name1), interest_to_safe(region_name2));
           tic
           [ytraj, cost, time] = findLinearTrajectory(safe_regions(safe_reg_path), region1, region2, MAX_VEL);
           toc
           
           cost_element1 = {region_name1 region_name2 cost time};
           cost_element2 = {region_name2 region_name1 cost time};
           cost_matrix = {cost_matrix{:} cost_element1 cost_element2};
           
%            % plot trajectory
            tmax = ytraj.tspan(2);
            t = 0:0.01:tmax;
            xtr = ytraj.eval(t);
            hold on
            
            
            % plot safe regions
            region_path = [];
            for l=1:length(safe_reg_path)
                reg_idx = safe_reg_path(l);
                V = safe_regions(reg_idx).P.V;
                V = [V; V(1,:)];
                V = V(convhull(V), :);
                region_path(l) = plot(V(:,1), V(:,2), 'Color', 'm', 'LineStyle', '--', 'LineWidth', 2.0);
            end
            line_plot = plot(xtr(1,:), xtr(2,:), 'g--', 'LineWidth', 3.5);
            
%             figinfo = hardcopy(figure(1),'-dzbuffer','-r0');
%             writeVideo(writerObj, im2frame(figinfo));
            export_fig(sprintf('figures/linear_paths/line_%02i.pdf', plot_count))
            pause();
            plot_count = plot_count+1;
            delete(line_plot);
            delete(region_path);
        end
    end
end

% close(writerObj);

%% Print results and form PDDL string
    pddl_properties = '';
   fprintf('Pairwise distances:\n');
for i=1:length(cost_matrix)
   element = cost_matrix{i};
   fprintf('%s  ->  %s  [%.3f]  [%.3f]\n', element{1}, element{2}, element{3}, element{4});
   pddl_properties = sprintf('%s\n(can_traverse quad0 loc-%s loc-%s) (= (cost loc-%s loc-%s) %.3f) (= (mintime loc-%s loc-%s) %.3f) ',pddl_properties, element{1}, element{2}, element{1}, element{2}, element{3}, element{1}, element{2}, element{4});
end

%% Write PDDL actions from all pairs linear trajectories
pddl_problem = fileread('pddl/problem.template');
pddl_problem = sprintf(pddl_problem, pddl_properties);
fid = fopen('pddl/problem_matlab.pddl', 'w');
fprintf(fid, pddl_problem);
fclose(fid);

%% Call COLIN, PDDL planner
disp('Running PDDL planner...')
cheat = 1;
if cheat
     colin_status = 0;
     colin_out = fileread('pddl/example_solution.plan');
else
    [colin_status, colin_out] = system('pddl/colin-cplex -E pddl/domain.pddl pddl/problem_matlab.pddl');
end
disp('PDDL Planner returned.')
%% Extract travel action from plan
succesful_plan = colin_status == 0 & regexp(colin_out, 'Solution Found');

path_regex = '\(navigate quad0 loc-([A-z,0-9]*) loc-([A-z,0-9]*)\)';
navigate_paths = regexp(colin_out, path_regex, 'tokens');

%% Compute and plot linear trajectories

lin_trajectories = {};
if exist('all_line_plots') & ishandle(all_line_plots), delete(all_line_plots), end;
all_line_plots = []
for i=1:length(navigate_paths)
    region_name1 = navigate_paths{i}{1};
    region_name2 = navigate_paths{i}{2};
    fprintf('Path from %s to %s.\n', region_name1, region_name2);
    region1 = interest_regions(region_name1);
    region2 = interest_regions(region_name2);
    safe_reg_path = extractAPSPpath(paths, interest_to_safe(region_name1), interest_to_safe(region_name2));
    [ytraj, cost, time] = findLinearTrajectory(safe_regions(safe_reg_path), region1, region2, MAX_VEL);
    tmax = ytraj.tspan(2);
    t = 0:0.01:tmax;
    xtr = ytraj.eval(t);
    lin_trajectories{i} = xtr;
    hold on
    all_line_plots(i) = plot(xtr(1,:), xtr(2,:), 'g.', 'LineWidth', 0.5);

%     pause();
%     delete(line_plot);
end

%% Compute smooth bezier trajectories for each travel activity
bez_trajectories = {};
if exist('all_bez_plots') & ishandle(all_bez_plots), delete(all_bez_plots), end;
all_bez_plots = [];
bez_times = [];
for i=1:length(navigate_paths)
    region_name1 = navigate_paths{i}{1};
    region_name2 = navigate_paths{i}{2};
    fprintf('Bezier Path from %s to %s.\n', region_name1, region_name2);
    region1 = interest_regions(region_name1);
    region2 = interest_regions(region_name2);
    reg1c = mean(region1.V);
    reg2c = mean(region2.V);
    safe_reg_path = extractAPSPpath(paths, interest_to_safe(region_name1), interest_to_safe(region_name2));
    tic
    bezPart = findBezierTrajectory(safe_regions(safe_reg_path), reg1c', reg2c', MAX_VEL);
%         plot(Psol{i}(:,1), Psol{i}(:,2), '+r')    
    bez_times(i) = toc;
    t=0:0.0001:1;
    bez_t = bezPart.evaluate(t);
    if i>1
        set(all_bez_plots(i-1), 'Color',[0.2 0.2 0.2])
        set(all_bez_plots(i-1), 'LineWidth',0.3)
        set(all_bez_plots(i-1), 'LineStyle','--')
    end
    all_bez_plots(i) = plot(bez_t(:,1), bez_t(:,2),'b-', 'LineWidth', 0.5);
    bez_trajectories{i} = bez_t;
    hold on
    
%     export_fig(sprintf('figures/bezier_paths/bezier_%02i.pdf', i))

    pause(0.1);
%     delete(bez_plot);
end

%% Show bezier paths
bez_plot = [];
writerObj = VideoWriter('videos/all_bez_paths.mpeg');
writerObj.FrameRate = 1;
open(writerObj);
for i=1:length(bez_trajectories)
    xtr = bez_trajectories{i};
    bez_plot(i) = plot(xtr(:,1), xtr(:,2), 'm-','LineWidth', 3.5)
    figinfo = hardcopy(figure(1),'-dzbuffer','-r0');
    writeVideo(writerObj, im2frame(figinfo));
    pause()
    delete(bez_plot(i))
end
close(writerObj);

%% Compute smooth SOS poly trajectories for each travel activity
sos_trajectories = {};
sos_times = [];
for i=1:length(navigate_paths)
    region_name1 = navigate_paths{i}{1};
    region_name2 = navigate_paths{i}{2};
    fprintf('SOS Path from %s to %s.\n', region_name1, region_name2);
    region1 = interest_regions(region_name1);
    region2 = interest_regions(region_name2);
    reg1c = mean(region1.V);
    reg2c = mean(region2.V);
    
    safe_reg_path = extractAPSPpath(paths, interest_to_safe(region_name1), interest_to_safe(region_name2));
    
    poly_deg = 3;
    num_seg = 10;
    tic
    ytraj = findMixedTrajectory(safe_regions(safe_reg_path), reg1c', reg2c', lb, ub, [], poly_deg, num_seg, [])
    sos_times(i) = toc;
    tmax = ytraj.tspan(2);
    t = 0:0.01:tmax;
    xtr = ytraj.eval(t);
    hold on
    sos_plot = plot(xtr(1,:), xtr(2,:), 'm')
    sos_trajectories{i} = xtr;
    
    

    pause(0.1);
    delete(sos_plot);
end

%% Load all SOS paths
% delete(sos_plot)
all_sos_plots = [];
for i=1:length(sos_trajectories)
    xtr = sos_trajectories{i};
    all_sos_plots(i) = plot(xtr(1,:), xtr(2,:), 'b--', 'LineWidth', 0.5)    
    pause()
%     delete(sos_plot(i))
end

%% SOS video 
% delete(sos_plot)
sos_plots = [];
writerObj = VideoWriter('videos/all_sos_paths.mpeg');
writerObj.FrameRate = 1;
open(writerObj);
for i=1:length(sos_trajectories)
    xtr = sos_trajectories{i};
    sos_plots(i) = plot(xtr(1,:), xtr(2,:),  'm-','LineWidth', 3.5)
    figinfo = hardcopy(figure(1),'-dzbuffer','-r0');
    writeVideo(writerObj, im2frame(figinfo));
    pause()
    delete(sos_plots(i))
end
close(writerObj);