function kinematics_sim()
    % configuration
    theta_start = [0,0,0,0];
    theta_end = [pi/2,pi/2,pi/2,pi/2];
    phi = [0,0,0,0];
    Ci_1 = 0.2; 
    R = 0.05;
    
    % time setting
    dt = 0.1;
    t_end = 10;
    
    % Initialize the storage for end-effector positions
    end_effector_positions = [];
    
    % simulation
    for t = 0:dt:t_end
        theta = theta_start + (t/t_end)*(theta_end - theta_start);  % linear interpolation from start to end pose
        
        % Call plot_robot to plot the robot
        Node = plot_robot(theta, phi, Ci_1, R);
        
        % Get the end effector position and add it to the storage
        end_effector_position = Node(:, 2*length(theta_start)+1);
        end_effector_positions = [end_effector_positions, end_effector_position];
        
        % Plot the trajectory of the end effector
        plot3(end_effector_positions(1, :), end_effector_positions(2, :), end_effector_positions(3, :), 'r-');
        
        % Pause for a short duration to simulate real-time behavior
        pause(dt);
        
        % Clear the figure for next iteration
        cla;
    end
end
