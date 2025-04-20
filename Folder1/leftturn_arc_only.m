function turtleleftturn_arc_only()
% Only performs a fixed arc left turn using odometry yaw tracking

% ROS setup
Node = ros2node('/lanenode');
imageTopic = '/camera/image_raw';
PubVel = ros2publisher(Node, '/cmd_vel', 'geometry_msgs/Twist');
velmsg = ros2message(PubVel);
imageSub = ros2subscriber(Node, imageTopic, "sensor_msgs/Image");
lanePub = ros2publisher(Node, "/camera/camera_lanes", "sensor_msgs/Image");
laneMsg = ros2message("sensor_msgs/Image");
odomNode = ros2node('/odomnode');
odomSub = ros2subscriber(odomNode, '/odom', 'nav_msgs/Odometry');
stopSub = ros2subscriber(Node, "/stop", "std_msgs/Bool");


sobel_x = [1, 0, -1; 2, 0, -2; 1, 0, -1]; % Sobel filter
CropValue = 270; % Crop top part of the image
prevCTE = 0;
intCTE = 0;
dt = 0.1;
startTime = tic; % Start the turn timer
killNode = false;


% Arc control params
R = 6.5;                     % Radius of arc (meters)
Lvel = 0.5;                  % Linear speed (m/s)
Avel = Lvel / R;             % Angular velocity (rad/s)
YawTarget = pi/2 - 0.05;     % ~90° turn (adjust for tighter or looser arc)
dt = 0.1;                    % Control loop delay

% Get starting yaw
odom = receive(odomSub, 10);
initial_yaw = getYaw(odom);

disp("Starting arc left turn...");

while ~killNode
% Check if this node need to be killed
stopMsg = stopSub.LatestMessage;
if ~isempty(stopMsg)
killNode = stopMsg.data;
end
elapsedTime = toc(startTime);
% Receive Image Data
[scanData, status, ~] = receive(imageSub, 10);
if ~status
disp('Failed to receive image.');
continue;
end
laneMsg=scanData;
msgOut = rosReadImage(scanData, "Encoding", 'bgr8');
croppedImage=msgOut(CropValue:end,:,:);
Image_grey = rgb2gray(croppedImage);
% Yellow mask + gradient
[BinaryYellowMask, ~] = createMask(croppedImage);
yellowGrad = imfilter(BinaryYellowMask, sobel_x, 'replicate', 'same', 'conv');
GradientFrame = imfilter(Image_grey, sobel_x, 'replicate', 'same', 'conv');
GradientFrame(GradientFrame <= 55) = 0; % Thresholding
%add yellow gradient to original
GradientFrame=logical(GradientFrame);
GradientFrame=GradientFrame+yellowGrad;
% Step 3: Hough Transform for Line Detection
[H, t, r] = hough(GradientFrame, 'Theta', -80:80);
P = houghpeaks(H, 2, 'threshold', ceil(0.2 * max(H(:))), 'NHoodSize', [275, 71], 'Theta', t);
lines = houghlines(GradientFrame, t, r, P, 'FillGap', 200, 'MinLength', 20);
% Initialize overlay image
overlayImage = msgOut;
figure(1); imshow(overlayImage);

%elapsedTime
%length(lines)
numLines = length(lines);
if numLines >= 2 && elapsedTime > 10 % wait at least 3 seconds before checking for return
disp("Two or more lines detected, right lane following.");
run(fullfile('/home/nuc7/turtlebot3_ws', 'turtlerightturn.m'));
return;
elseif numLines == 1
disp("One line detected: continuing right line following...");
elseif numLines == 0
disp("No lines detected.");
end
    odom = receive(odomSub, 10);
    current_yaw = getYaw(odom);
    delta_yaw = wrapToPi(current_yaw - initial_yaw);

    if abs(delta_yaw) < YawTarget
        velmsg.linear.x = Lvel;
        velmsg.angular.z = Avel;
        send(PubVel, velmsg);
        disp(["Yaw: ", num2str(rad2deg(delta_yaw)), " deg"]);
    else
        disp("Left turn complete.");
        velmsg.linear.x = 0;
        velmsg.angular.z = 0;
        send(PubVel, velmsg);
        break;
    end

    pause(dt);
end
end

function yaw = getYaw(odom)
q = odom.pose.pose.orientation;
angles = quat2eul([q.w q.x q.y q.z]);
yaw = angles(1);
end
function [BW, maskedRGBImage] = createMask(RGB)
I = rgb2hsv(RGB);
channel1Min = 0.100; channel1Max = 0.300;
channel2Min = 0.401; channel2Max = 1.000;
channel3Min = 0.702; channel3Max = 1.000;
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
maskedRGBImage = RGB;
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end