function [X_mesh, Y_mesh, Z_mesh, reach_mesh, min_limits, max_limits] = voxelizeWorkspace(X_ee_array, Y_ee_array, Z_ee_array, voxelsPerDim)

    % Getting the lower and upper limits for axes based on the workspace
    X_lower_lim = min(X_ee_array(:))-0.1; % in meter
    X_upper_lim = max(X_ee_array(:))+0.1; % in meter
    
    Y_lower_lim = min(Y_ee_array(:))-0.1; % in meter
    Y_upper_lim = max(Y_ee_array(:))+0.1; % in meter
    
    Z_lower_lim = min(Z_ee_array(:))-0.1; % in meter
    Z_upper_lim = max(Z_ee_array(:))+0.1; % in meter
    
    min_limits = [X_lower_lim, Y_lower_lim, Z_lower_lim];
    max_limits = [X_upper_lim, Y_upper_lim, Z_upper_lim];
    
    num_row = voxelsPerDim; % Number of voxels in X dim
    num_col = voxelsPerDim; % Number of voxels in Y dim
    num_depth = voxelsPerDim; % Number of voxels in Z dim
%     total_voxels = num_col * num_row * num_depth;
    
    [X_mesh, Y_mesh, Z_mesh] = ndgrid(linspace(X_lower_lim, X_upper_lim, num_row), linspace(Y_lower_lim, Y_upper_lim, num_col), linspace(Z_lower_lim, Z_upper_lim, num_depth));
    reach_mesh = zeros(size(X_mesh));
    
    count = 0;
    total_points = length(X_ee_array);
%     disp(["total: ", num2str(total_points)]);
%     input('Enter to continue');
    
    for i=1:total_points             
        count = count + 1;
        clc
        fprintf("Point %d out of %d\n", count, total_points);
        fprintf("Progress %2.2f%%\n", (count/total_points)*100);
        X_bin_index = round((X_ee_array(i)-X_lower_lim)/(X_upper_lim-X_lower_lim)*num_row);
        Y_bin_index = round((Y_ee_array(i)-Y_lower_lim)/(Y_upper_lim-Y_lower_lim)*num_col);
        Z_bin_index = round((Z_ee_array(i)-Z_lower_lim)/(Z_upper_lim-Z_lower_lim)*num_depth);
        reach_mesh(X_bin_index + 1, Y_bin_index + 1, Z_bin_index + 1) = reach_mesh(X_bin_index + 1, Y_bin_index + 1, Z_bin_index + 1) + 1;
    end
    
%     R_mesh = reach_mesh;
    reach_mesh(reach_mesh~=0) = 1;
    reach_mesh(reach_mesh==0) = NaN;

end