classdef BezierPartTrajectory
    properties
        P = {[0 0; 2 2; 4 2; 6 1; 8 2] ; [8 2; 10 0; 14 1; 16 5]};
%         P{1} = [0 0; 2 2; 4 2; 6 1; 8 2];
%         P{2} = [8 2; 10 0; 14 1; 16 5];        
        M = 2; % Number of bezier parts        
    end
    methods
        
        function obj = BezierPartTrajectory(P)
            if  nargin > 0
                obj.P = P;
            end
        end
        
        function y = applyFunction(obj, func_handle, t)
            % Assumes t is an ORDERED vector between [0,1]
            % Evaluates the trajectory in that vector
            n = size(obj.P{1},1) - 1;
%             m = size(obj.P{1},2);
            M = length(obj.P);
%             num_p = length(t);
            i = 1:M;
            increment = 1/M;
            stops = [0 i*increment];
            
            % From MATLAB's ppval fun
            sizet = size(t); lt = numel(t); t = reshape(t,1,lt);
            %  if XX is row vector, suppress its first dimension
            if length(sizet)==2&&sizet(1)==1, sizet(1) = []; end

            % take apart PP
%             [b,c,l,k,dd]=unmkpp(pp);

            % for each evaluation site, compute its breakpoint interval
            % (mindful of the possibility that xx might be empty)
            if lt, [~,index] = histc(t,[-inf,stops(2:M),inf]);
            else index = ones(1,lt);
            end

            % adjust for troubles, like evaluation sites that are NaN or +-inf
            inft = find(t==inf); if ~isempty(inft), index(inft) = M; end
            nogoodt = find(index==0);
            if ~isempty(nogoodt), t(nogoodt) = NaN; index(nogoodt) = 1; end

            % now go to local coordinates ...
            t = (t-stops(index)) / increment;
            
            unique_idx = unique(index);
%             y = zeros(num_p, m);
            
            for j=1:length(unique_idx)
                part_j = unique_idx(j);
                bez_j = BezierTrajectory(obj.P{part_j});
%                 y(index == part_j, :) = bez_j.(func_name)(t(index == part_j))
                y(index == part_j, :) = func_handle(bez_j, t(index == part_j));
            end
        end
        
        function traj = evaluate(obj, t)
            % Assumes t is an ORDERED vector between [0,1]
            % Evaluates the trajectory in that vector
            n = size(obj.P{1},1) - 1;
            m = size(obj.P{1},2);
            M = length(obj.P);
            num_p = length(t);
            i = 1:M;
            increment = 1/M;
            stops = [0 i*increment];
            
            % From MATLAB's ppval fun
            sizet = size(t); lt = numel(t); t = reshape(t,1,lt);
            %  if XX is row vector, suppress its first dimension
            if length(sizet)==2&&sizet(1)==1, sizet(1) = []; end

            % take apart PP
%             [b,c,l,k,dd]=unmkpp(pp);

            % for each evaluation site, compute its breakpoint interval
            % (mindful of the possibility that xx might be empty)
            if lt, [~,index] = histc(t,[-inf,stops(2:M),inf]);
            else index = ones(1,lt);
            end

            % adjust for troubles, like evaluation sites that are NaN or +-inf
            inft = find(t==inf); if ~isempty(inft), index(inft) = M; end
            nogoodt = find(index==0);
            if ~isempty(nogoodt), t(nogoodt) = NaN; index(nogoodt) = 1; end

            % now go to local coordinates ...
            t = (t-stops(index)) / increment;
            
            unique_idx = unique(index);
            traj = zeros(num_p, m);
            
            for j=1:length(unique_idx)
                part_j = unique_idx(j);
                bez_j = BezierTrajectory(obj.P{part_j});
                traj(index == part_j, :) = bez_j.evaluate(t(index == part_j))
            end
            
        end
        
        function y = curvature(obj, t)
            y = obj.applyFunction(@(bez, t) bez.curvature(t), t);
        end
        
        function y = getDerivative(obj, d, t)
            y = obj.applyFunction(@(bez, t) bez.getDerivative(d,t), t);
        end
        
        function plot(obj)
            t = 0:0.01:1;            
            traj = obj.evaluate(t);
            figure(1); hold on;
            % Control points
            for j=1:length(obj.P)
                Pj = obj.P{j};
                plot(Pj(:,1), Pj(:,2), '+-r')
            end
            plot(traj(:,1), traj(:,2), 'b')
        end
        
        
    end
    
    
end
