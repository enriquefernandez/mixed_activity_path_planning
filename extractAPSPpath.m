function sol_path = extractAPSPpath(P, start_idx, end_idx)



if P(start_idx, end_idx) == 0
    sol_path = [];
else
    sol_path = [start_idx];
    n = P(start_idx, end_idx);    
    while n~=end_idx
        sol_path = [sol_path n];        
        n = P(n, end_idx);
    end
    sol_path = [sol_path end_idx];
    
end


end