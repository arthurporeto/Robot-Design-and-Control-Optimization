v = VideoWriter('animation.mp4', 'MPEG-4'); % Create a VideoWriter object
v.FrameRate = 30; % Set frames per second
open(v); % Open the video file for writing

th1 = z(1,:);
th2 = z(2,:);
%read l1 and l2 from the previous code
l1 = init.l1;
l2 = init.l2;
plot_bounds = l1+l2+0.1;

%The coordinates of the links of each of the manipulator
x1 = l1*cos(th1);
y1 = l1*sin(th1);
x2 = l1*cos(th1)+l2*cos(th1+th2);
y2 = l1*sin(th1)+l2*sin(th1+th2);
figHandle = figure();
for i=1:int16(init.N/100):init.N
    A = [0 x1(i)]; 
    B = [0 y1(i)]; 
    %   subplot(1,3,1);
    plot(A,B,'*')
    axis([-plot_bounds plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A,B)
    hold on
    A2 = [x1(i) x2(i)]; 
    B2 = [y1(i) y2(i)];
    %   subplot(1,3,1);
    plot(A2,B2,'*')
    axis([-plot_bounds plot_bounds -plot_bounds plot_bounds])
    hold on
    line(A2,B2,'Color','red')
    pause(0.03);
    % Capture the frame
    frame = getframe(gcf); % Get current figure frame
    writeVideo(v, frame); % Write frame to video

end

close(v); % Finalize and save the video

