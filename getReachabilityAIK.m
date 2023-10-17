function [R_index] = getReachabilityAIK(robot, min_limits, max_limits, voxelsPerDim, R_mesh, theta_nums, phi_nums, aik_mfile_name, both_halves)

    theta = linspace(0, pi  , theta_nums);
    phi   = linspace(0, 2*pi, phi_nums + 1);
    
    ik = inverseKinematics();
    ik.RigidBodyTree = robot;
    
    % ik_weights = [0.25 0.25 0.25 1 1 1];
    
    % initialguess = robot.homeConfiguration;
    
    [i_end, j_end, k_end] = size(R_mesh);
    R_index = zeros(size(R_mesh));
    total_valid_points = length(R_mesh(~isnan(R_mesh)));
    % total_valid_points = i_end*j_end/2*k_end;
    loop_counter = 0;
    for i=1:i_end
        for j=1:floor(j_end/2)+1 % Only right half of the workspace (symmetrical)
            for k=1:k_end
                if (~isnan(R_mesh(i,j,k)))
                    loop_counter = loop_counter + 1;
                    fprintf("Point %d out of %d\n", loop_counter, total_valid_points);
                    pose_target = [1.0000    0.0000    0.0000    min_limits(1) + (i-1)*(max_limits(1)-min_limits(1))/voxelsPerDim
                                   0.0000    1.0000    0.0000    min_limits(2) + (j-1)*(max_limits(2)-min_limits(2))/voxelsPerDim
                                   0.0000    0.0000    1.0000    min_limits(3) + (k-1)*(max_limits(3)-min_limits(3))/voxelsPerDim
                                   0.0000    0.0000    0.0000     1.0000];
                    for i_theta=1:length(theta)
                        for j_phi=1:length(phi)-1
                            pose_target(1:3,1:3) = eul2rotm([theta(i_theta) 0 phi(j_phi)], 'XYZ');
                            % fprintf("AIK File Name: %s.mat\n", aik_mfile_name)
                            if checkReachabilityAIK(robot, aik_mfile_name, pose_target)
                                R_index(i, j, k) = R_index(i, j, k) + 1;
                            end
                        end
                    end
                    fprintf("Reachability %d / %d\n", R_index(i, j, k), theta_nums * phi_nums);
                end
    
            end
        end
    end
    
    % copying the values from the right half to the left half
    if both_halves
        for i=1:i_end
            for j=floor(j_end/2)+2:j_end 
                for k=1:k_end
                    if (~isnan(R_mesh(i,j,k)))
                        loop_counter = loop_counter + 1;
                        fprintf("Point %d out of %d\n", loop_counter, total_valid_points);
                        R_index(i, j, k) = R_index(i, floor(j_end/2)-(j-floor(j_end/2)-2), k);
                        fprintf("Reachability %d / %d\n", R_index(i, j, k), theta_nums*(phi_nums-1));
                    end
                end
            end
        end
    end
    
    R_index(R_index==0) = NaN; % assign NaN to zero arrays to hide them in visualization

end