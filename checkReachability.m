function [isReachable] = checkReachability(robot, ikSolver, ik_weights, initial_guesses, ee_pose)

    [configSol, solInfo] = ikSolver('tool0', ee_pose, ik_weights, initial_guesses);
    
    [isColliding, ~, ~] = robot.checkCollision(configSol);
    
    
    if (~isColliding(1) && solInfo.ExitFlag == 1) % ignore self-collision "~isColliding(1) &&" and collision with objects "~isColliding(2) &&"
        isReachable = 1;
    else
        isReachable = 0;
    end

end