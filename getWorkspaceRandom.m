function [X_ee_array, Y_ee_array, Z_ee_array] = getWorkspaceRandom(robot, lower_limits, upper_limits, total_points, base_link_name, ee_link_name)

    count = 0;
    
    X_ee_array = zeros([1, total_points]);
    Y_ee_array = zeros([1, total_points]);
    Z_ee_array = zeros([1, total_points]);
    
    % fprintf("Total points: %d\n", total_points);
    % disp(["total: ", num2str(total_points)]);
    % input('Enter to continue');
    % tic;
    
    for i=1:total_points
        
        count = count + 1;
        clc
        fprintf("Point %d out of %d\n", count, total_points);
        fprintf("Progress %2.2f%%\n", (count/total_points)*100);
%         message = sprintf("Workspace generation in progress %2.2f%%\n", (count/total_points)*100);
%         app.StatusTextArea.Value = message;
    %     elapsed_time=toc;
    %     fprintf("Elapsed time: %dd %dh %dm %ds\n", floor(elapsed_time/(24*3600)), floor(rem(elapsed_time,24*3600)/3600), floor(rem(rem(elapsed_time,24*3600), 3600)/60), floor(rem(rem(rem(elapsed_time,24*3600), 3600),60)));
    %     fprintf("Estimated remaining time: %dd %dh %dm %ds\n", floor((elapsed_time/count)*(total_points-count)/(24*3600)), floor(rem((elapsed_time/count)*(total_points-count),(24*3600))/3600), floor(rem(rem((elapsed_time/count)*(total_points-count),(24*3600)),3600)/60), floor(rem(rem(rem((elapsed_time/count)*(total_points-count),(24*3600)),3600),60)));
        
        joint_rand = rand(size(lower_limits), "double");
        joint_rand = lower_limits + joint_rand .* (upper_limits - lower_limits);
    %     joint_rand = joint_rand * pi/180;
    
        ee_pose = getTransform(robot, joint_rand', ee_link_name, base_link_name);
        
        X_ee_array(count) = ee_pose(1, 4);
        Y_ee_array(count) = ee_pose(2, 4);
        Z_ee_array(count) = ee_pose(3, 4);
        
    end
end