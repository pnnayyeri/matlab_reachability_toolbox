function [isReachable] = checkReachabilityAIK(robot, ikmfile, ee_pose)

    % [configSol, solInfo] = eval(ikmfile , '(tool0, ' , ee_pose, ', ' , ik_weights, ', ' , initial_guesses ,')');
    configSol = feval(ikmfile, ee_pose, true);
    
    if size(configSol) == [0,6] % if there is no solution found (default format is [x,6])
        isReachable = 0; % no solution found so the pose is not reachable
    elseif size(configSol) == [1,6] % if there is only one solution (default format is [x,6])
        configSol = configSol';
        [isColliding, ~, ~] = robot.checkCollision(configSol, SkippedSelfCollisions="parent");
        if ~isColliding(1) % ignore self-collision "~isColliding(1) &&" and collision with objects "~isColliding(2) &&"
            isReachable = 1; % the only solution is valid
        else
            isReachable = 0; % the only solution is invalid, the pose not reachable
        end
    elseif size(configSol, 1) >= 2 % if there are more than one solution
        configSol = configSol'; % transpose so it complies with the rest of the code
        for i=1:size(configSol, 2) % iterate over the solutions
            [isColliding, ~, ~] = robot.checkCollision(configSol(:, i)); % check one-by-one
            if ~isColliding(1) % ignore self-collision "~isColliding(1) &&" and collision with objects "~isColliding(2) &&"
                isReachable = 1; 
                break % as soon as one solution is valid there is no need to continue iteration
            else
                isReachable = 0; % keep isReachable false
            end
        end
    end
end