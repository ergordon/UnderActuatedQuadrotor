clc
close all

record_video = 1;

if(record_video)
    aviobj = VideoWriter('newfile.avi');
    open(aviobj);
end

[x,y,z] = sphere;
[fs,vs,cs] = surf2patch(obst_s*x,obst_s*y,obst_s*z,obst_s*z);
L = size(vs,1);
% Define "mocap" as a 7-by-N matrix, where 7 is the number of rows
%         in the mocap message object, and N is the number of sample 
%         measurements over time.

% Row 1: Time vector
% Row 2: X position vector
% Row 3: Y position vector
% Row 4: Z position vector
% Row 5: Theta_x vector
% Row 6: Theta_y vector
% Row 7: Theta_z vector

t = mocap(1,:);
dt = mocap(1,2:end) - mocap(1,1:end-1);
x_pos = mocap(2,:); y_pos = mocap(3,:); z_pos = mocap(4,:);
theta_x = mocap(5,:); theta_y = mocap(6,:); theta_z = mocap(7,:);

%Use these mat files from Lab1
% CREATE WORLD OBJECTS
load('world.mat');  % this will load variables:   w0  wsz  wcolors
w0(3,:) = -w0(3,:);

% CREATE A QUADROTOR
load('quadmodel.mat');  % this will load variables:  p1   faces   colors
p1(1:2,:) = (0.6/1.62)*p1(1:2,:);
p1(3,:) = -p1(3,:);

R02 = [1  0  0;0  1  0; 0  0 1];

% SETUP THE PLOT
clf;
set(gcf,'Renderer','zbuffer');
axis([-4 4 -4 4 -1 3.5]);
axis equal;
hold on;


R = [cos(theta_y(1))*cos(theta_z(1)), -cos(theta_y(1))*sin(theta_z(1)), sin(theta_y(1));
    cos(theta_x(1))*sin(theta_z(1))+cos(theta_z(1))*sin(theta_x(1))*sin(theta_y(1)), cos(theta_x(1))*cos(theta_z(1))-sin(theta_x(1))*sin(theta_y(1))*sin(theta_z(1)), -cos(theta_y(1))*sin(theta_x(1));
    sin(theta_x(1))*sin(theta_z(1))-cos(theta_x(1))*cos(theta_z(1))*sin(theta_y(1)), cos(theta_z(1))*sin(theta_x(1))+cos(theta_x(1))*sin(theta_y(1))*sin(theta_z(1)), cos(theta_x(1))*cos(theta_y(1))];

% ROTATE FROM BODY FRAME TO MATLAB PLOT FRAME
%%%%%
%Enter your code here:
%%%%%
p2 = R02*(R*p1+repmat([x_pos(1),y_pos(1),z_pos(1)].',1,294));

% ROTATE FROM WORLD FRAME TO MATLAB PLOT FRAME
w2 = R02*w0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scatter3(w2(1,:), w2(2,:), w2(3,:), wsz, wcolors,'filled');
plot3(0,0,0,'k.','markersize',16);
view(-10,25);

v_s = vs + repmat(obst_all(1,:),L,1);
s = patch('Vertices',v_s,'Faces',fs,...
          'CData',cs,'FaceColor','flat');
hold on;

h = patch('Vertices',p2','Faces',faces,...
          'CData',colors,'FaceColor','flat');
hTitle = title(sprintf('t = %4.2f',0));
lighting flat
light('Position',[0 -2 -1])
light('Position',[0 -2 1])
xlabel('x'); ylabel('y'); zlabel('z');
drawnow;
pause(0.5);

% ANIMATE THE RESULTS
i = 1;
tic;
while (i<length(t)-1)
    if (toc > dt(i))
        tic;
        i = i+1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % YOUR CODE HERE TO COMPUTE p0
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        R = [cos(theta_y(i))*cos(theta_z(i)), -cos(theta_y(i))*sin(theta_z(i)), sin(theta_y(i));
             cos(theta_x(i))*sin(theta_z(i))+cos(theta_z(i))*sin(theta_x(i))*sin(theta_y(i)), cos(theta_x(i))*cos(theta_z(i))-sin(theta_x(i))*sin(theta_y(i))*sin(theta_z(i)), -cos(theta_y(i))*sin(theta_x(i));
             sin(theta_x(i))*sin(theta_z(i))-cos(theta_x(i))*cos(theta_z(i))*sin(theta_y(i)), cos(theta_z(i))*sin(theta_x(i))+cos(theta_x(i))*sin(theta_y(i))*sin(theta_z(i)), cos(theta_x(i))*cos(theta_y(i))];
        
        % TRANSFORM FROM BODY TO WORLD FRAME
        %%%%%
        %Enter your code here:
        %%%%%
        p0 = R*p1 + repmat([x_pos(i),y_pos(i),z_pos(i)].',1,294);
        
        % TRANSFORM FROM WORLD FRAME TO MATLAB DISPLAY FRAME
        p2 = R02*p0;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v_s = vs + repmat(obst_all(i,:),L,1);
        
        % UPDATE GRAPHICS OBJECT VERTICES
        set(s,'Vertices',v_s);
        set(h,'Vertices',p2');
        set(hTitle,'string',sprintf('t = %4.2f',t(i)));
        if(record_video)
            F = getframe(gcf);
            writeVideo(aviobj,F);
        end
        drawnow;
    end
end
if(record_video)
    close(aviobj);
end