% addpath(fullfile('/Users/efernan/underactuated/drake-distro-wind/'))
if exist('/Users/efernan/underactuated/drake-distro-wind/')
    cd('/Users/efernan/underactuated/drake-distro-wind/')
else
    cd('/Users/efernan/underactuated/drake-distro/')
end
addpath_pods
addpath_iris
addpath_gurobi
addpath_mosek
addpath_snopt
addpath_drake
mpt_init