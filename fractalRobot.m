function fractalRobot(i, n, C0, R, direction, init_dir)
% i: number of joints
% n: number of sides for the regular polygon
% C0: initial straight link length
% R: radius of the arc joint
% direction: direction of the fractal iteration, -1 for clockwise, +1 for counterclockwise
% init_dir: initial direction, represented by a 1*3 vector

theta = 2*pi/n; % arc angle in radians
unit_num = i+1; % number of units

% Check if the direction vector is valid
if length(init_dir)~=3
    error('Invalid direction vector. It must be a 1*3 vector.')
end

% Normalize the direction vector
init_dir = init_dir / norm(init_dir);

% Initialize the current direction and position
curr_dir = init_dir;
curr_pos = zeros(1,3);

% Start plotting
figure;
hold on;

for j=1:unit_num
    % Draw a straight link
    next_pos = curr_pos + C0*curr_dir;
    plot3([curr_pos(1), next_pos(1)], [curr_pos(2), next_pos(2)], [curr_pos(3), next_pos(3)], 'b','LineWidth', 1);
    
    % Draw an arc joint
    angle_range = linspace(0, theta, 100);
    for angle = angle_range
        %rotated_dir = rotate_vector(curr_dir, angle*direction);
        %arc_pos = next_pos + R*rotated_dir;
        %plot3(arc_pos(1), arc_pos(2), arc_pos(3), 'k.');
        %next_dir = rotate_vector(curr_dir,theta*direction);
        %cross_product = cross(curr_dir,next_dir);
        %cross_product = cross_product/norm(cross_product);
        %arc_initial_dir = cross(curr_dir,cross_product);
        %arc_initial_dir = arc_initial_dir/norm(arc_initial_dir);
        arc_initial_pos = rotate_vector(curr_dir,-direction*pi/2);
        arc_initial_dir = arc_initial_pos/norm(arc_initial_pos);
        rotated_dir = rotate_vector(arc_initial_pos, angle*direction);
        %rotated_dir = arc_initial_dir*cos(theta) + cross(cross_product,arc_initial_dir)*sin(theta) + cross_product*dot(cross_product,arc_initial_dir)*(1-cos(theta));
        arc_curr_pos = next_pos - R*arc_initial_dir;
        arc_pos = arc_curr_pos + R*rotated_dir;
        plot3(arc_pos(1), arc_pos(2), arc_pos(3), 'k.');
    end
    text_pos = arc_curr_pos ; % adjust this to move the text position
    text(text_pos(1), text_pos(2), text_pos(3), ['T', num2str(j)]);

    % Update the current position and direction
    curr_pos = arc_pos;
    curr_dir = rotate_vector(curr_dir, theta*direction);
    
    % Update the length of the straight link for the next iteration
    %C0 = C0 - R*tan(theta/2);
    if  mod(j-1,n-2) == 0 
        C0 = C0 - R*tan(theta/2);
    end
end

hold off;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
end

function rotated_vector = rotate_vector(vector, angle)
% This function rotates a vector around the z-axis by a specified angle
rot_matrix = [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0, 0, 1];
rotated_vector = (rot_matrix*vector')';
end


