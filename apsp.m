function [C, P] = apsp(G)
% Floyd-Warshall implementation of all pairs shortest paths
% returns C, matrix of costs between nodes,
% and
% P, matrix of paths between nodes. Need to recursively extract the path
% from P

num_nodes = size(G,1);

if size(G,1) ~= size(G,2)
    error('Matrix G needs to be square.')
end

% C = sparse(num_nodes, num_nodes);
% Set initial cost matrix to initial graph edge costs
C = inf(num_nodes, num_nodes);
C(find(G)) = G(find(G));
C(logical(eye(num_nodes))) = zeros(num_nodes,1);

% Set path to initial edges
P = repmat(1:num_nodes, num_nodes,1);
P = P .* logical(full(G));
P(logical(eye(num_nodes))) = 1:num_nodes;

for k=1:num_nodes
    for i=1:num_nodes
        for j=1:num_nodes
            d = C(i,k) + C(k,j);
            if d < C(i,j)
                C(i,j) = d;
                P(i,j) = P(i,k);
            end
        end
    end
end

end