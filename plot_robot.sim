function Node = plot_robot (theta, phi, Ci_1, R)
%% This script is used to generate the simulation pattern of the planar fractal root, also known as the forward kinematics
    % theta :storing theta size: 1 * i, eg. [0,pi/2,pi/4,pi/6] will
    % generate a robot with 4 links, each has a bending angle corresponding
    % to the configuration
    % phi: the twisting angle 
    %figure;
    %hold on;
    %num = 2*length(theta) + 1 % this is the number of nodes
    clf;
    hold on;
    Node = cal_node(theta,phi,Ci_1,R);
    disp(Node);
    for i = 1 : length(theta)
        p1 = Node(:,2*i - 1);
        p2 = Node(:,2*i);
        p3 = Node(:,2*i + 1);
        % draw  straight line
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)],'LineWidth', R);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        grid on; hold on;
        %draw arc
        angle_range = linspace(0,theta(i),100);
        for angle = angle_range
            v1 = p2 - p1;
            v2 = p3 - p2;
            n1 = cross(v1,v2);
            n1 = n1 / norm(n1);
            arc_rotate = cross(v1,n1);
            arc_rotate = arc_rotate/norm(arc_rotate);
        
            rotated_dir = rotateVector(arc_rotate,n1,-angle);
            arc_curr_pos = p2 - R*arc_rotate;
            arc_pos = arc_curr_pos + R*rotated_dir;
            plot3(arc_pos(1), arc_pos(2), arc_pos(3), 'k.');
        end
    end
end
    
%% this function will output a 4*4*(2*num+1) space matrix, 
%  each 4*4 matrix inside indicates a homogeneous transfrom from the base  
%  frame of the corresponding node
function A = cal_homo(theta,phi,Ci_1,R) 
    num = length(theta);
    a = zeros(4,4,6);
    A = zeros(4,4,2*num+1);
    homo_matrix = eye(4);
    j = 1;
    for i = 1: num
        di = 2*R*sin(theta(i)/2);
        a(:, :, 1) = matrixA(0,0,0,-pi/2);
        a(:, :, 2) = matrixA(phi(i),Ci_1,0,0);
        a(:, :, 3) = matrixA(0,0,0,pi/2);
        a(:, :, 4) = matrixA(theta(i)/2,0,0,-pi/2);
        a(:, :, 5) = matrixA(0,di,0,pi/2);
        a(:, :, 6) = matrixA(theta(i)/2,0,0,0);
        if  mod(i-1,2) == 0 
            Ci_1 = Ci_1 - R*tan(pi/4);
        end
        while j < 2*num + 1
            A(:,:,j) =   homo_matrix ;
            A(:,:,j+1) = A(:,:,j) * a(:,:,1) * a(:,:,2)*a(:,:,3);
            A(:,:,j+2) = A(:,:,j+1)*a(:,:,4)*a(:,:,5)*a(:,:,6);
            break;
        end
        j = j + 2;
        homo_matrix = A(:,:,j);
    end 
end
%% this function will map the homogeneous matrix to its corresponding node
function Node = cal_node(theta,phi,Ci_1,R) % Node is a 3*(2*num+1) matrix
    A_homo = cal_homo(theta,phi,Ci_1,R);
    Node = zeros(3, 2*length(theta)+1);
    for idx = 1:(2*length(theta)+1)
        Node(:, idx) = A_homo(1:3, 4, idx);
    end
end
%% 
function mA = matrixA(thetai, di, ai, alphai)
    % TODO: you may want to define geometric parameters here that will be
    % useful in computing the forward kinematics. The data you will need
    % is provided in the lab handout
    mA = [cos(thetai), -sin(thetai)*cos(alphai), sin(thetai)*sin(alphai), ai*cos(thetai);
          sin(thetai), cos(thetai)*cos(alphai), -cos(thetai)*sin(alphai), ai*sin(thetai);
          0, sin(alphai), cos(alphai), di;
          0, 0, 0, 1];
end

