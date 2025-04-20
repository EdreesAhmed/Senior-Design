function turtlerightturn()
% TURTLERIGHTTURN - A right-turn lane following controller using ROS 2.
% This function connects to ROS 2 topics, processes camera input to detect
% lane lines (especially the rightmost line), and uses PID control to follow
% the line. It is intended for use with TurtleBot3 or similar robots.

% ------------------------- ROS 2 INITIALIZATION -------------------------

% Define camera image topic
imageTopic = '/camera/image_raw';

% Create the ROS 2 node
Node = ros2node('/lanenode');

% Subscribe to camera image topic
imageSub = ros2subscriber(Node, imageTopic, "sensor_msgs/Image");

% Publisher to send the processed overlayed image
lanePub = ros2publisher(Node, "/camera/camera_lanes", "sensor_msgs/Image");
laneMsg = ros2message("sensor_msgs/Image");

% Publisher for velocity commands
PubVel = ros2publisher(Node,'/cmd_vel','geometry_msgs/Twist');
velmsg = ros2message(PubVel);

% Subscriber to listen for external stop signal
stopSub = ros2subscriber(Node, "/stop", "std_msgs/Bool");

% Define Sobel filter for edge detection
sobel_x = [1, 0, -1; 2, 0, -2; 1, 0, -1];

% Set crop height to remove top part of the image (focus on road)
CropValue = 270;

% PID controller variables
prevCTE = 0;
intCTE = 0;
dt = 0.1;  % time step for control loop

% Start timer to measure how long we've been turning
startTime = tic;

% Flag to break loop when stop signal is received
killNode = false;

% ------------------------- PID SLIDER UI -------------------------

% Create a UI window for live PID tuning
ui = uifigure('Name', 'PID Tuning', 'Position', [100 100 300 300]);

% Proportional gain slider
P_slider = uislider(ui, 'Position', [75 240 150 3], 'Limits', [0 0.02], 'Value', 0.0025);
P_label = uilabel(ui, 'Position', [75 260 150 15], 'Text', 'P Gain');

% Integral gain slider
I_slider = uislider(ui, 'Position', [75 160 150 3], 'Limits', [0 0.005], 'Value', 0.0002);
I_label = uilabel(ui, 'Position', [75 180 150 15], 'Text', 'I Gain');

% Derivative gain slider
D_slider = uislider(ui, 'Position', [75 80 150 3], 'Limits', [0 0.02], 'Value', 0.0005);
D_label = uilabel(ui, 'Position', [75 100 150 15], 'Text', 'D Gain');

% ------------------------- MAIN LOOP -------------------------
while ~killNode

    % Check if stop signal was received
    stopMsg = stopSub.LatestMessage;
    if ~isempty(stopMsg)
        killNode = stopMsg.data;
    end

    elapsedTime = toc(startTime);  % measure elapsed time

    % Receive an image from camera
    [scanData, status, ~] = receive(imageSub, 10);
    if ~status
        disp('Failed to receive image.');
        continue;
    end

    % Convert image message to MATLAB image format
    laneMsg = scanData;
    msgOut = rosReadImage(scanData, "Encoding", 'bgr8');

    % Crop the image to remove sky/ceiling and focus on road
    croppedImage = msgOut(CropValue:end,:,:);
    Image_grey = rgb2gray(croppedImage);

    % Apply yellow color mask and Sobel gradient
    [BinaryYellowMask, ~] = createMask(croppedImage);
    yellowGrad = imfilter(BinaryYellowMask, sobel_x, 'replicate', 'same', 'conv');

    % Apply Sobel filter to grayscale image and threshold it
    GradientFrame = imfilter(Image_grey, sobel_x, 'replicate', 'same', 'conv');
    GradientFrame(GradientFrame <= 55) = 0;
    GradientFrame = logical(GradientFrame);
    GradientFrame = GradientFrame + yellowGrad;

    % Use Hough transform to detect lines
    [H, t, r] = hough(GradientFrame, 'Theta', -80:80);
    P = houghpeaks(H, 2, 'threshold', ceil(0.2 * max(H(:))), ...
        'NHoodSize', [275, 71], 'Theta', t);
    lines = houghlines(GradientFrame, t, r, P, 'FillGap', 200, 'MinLength', 20);

    % Copy image for drawing lines
    overlayImage = msgOut;
    numLines = length(lines);

    % If enough lines are detected after turn, return to normal following
    if numLines >= 2 && elapsedTime > 10
        disp("Two or more lines detected after turn duration: returning to full lane following.");
        run(fullfile('/home/nuc7/turtlebot3_ws', 'IntersectionDetection.m'));
        return;
    elseif numLines == 1
        disp("One line detected: continuing right line following...");
    elseif numLines == 0
        disp("No lines detected.");
    end

    % --------------------- FIND RIGHTMOST LINE ---------------------
    maxX = -inf;
    rightmostLine = [];

    % Loop through lines to find the one furthest to the right
    for k = 1:length(lines)
        % Re-adjust Y-coordinates since image was cropped
        lines(k).point1(2) = lines(k).point1(2) + CropValue - 1;
        lines(k).point2(2) = lines(k).point2(2) + CropValue - 1;
        avgX = mean([lines(k).point1(1), lines(k).point2(1)]);
        if avgX > maxX
            maxX = avgX;
            rightmostLine = lines(k);
        end
    end

    % --------------------- DRAW RIGHTMOST LINE ---------------------
    overlayImage = msgOut;
    if ~isempty(rightmostLine)
        xy = [rightmostLine.point1; rightmostLine.point2];

        % Draw the detected line
        overlayImage = insertShape(overlayImage, 'Line', [xy(1, :) xy(2, :)], ...
            'Color', [255 0 0], 'LineWidth', 3);

        % Interpolate line points and draw them
        N = 50;
        x_points = linspace(xy(1, 1), xy(2, 1), N);
        y_points = linspace(xy(1, 2), xy(2, 2), N);
        points = round([x_points; y_points]);

        % Draw circles along the line and compute average x-position
        xTotal = 0;
        for p = 1:N
            overlayImage = insertShape(overlayImage, 'Circle', ...
                [points(1, p), points(2, p), 3], 'Color', [0 0 255], 'LineWidth', 1);
            xTotal = xTotal + points(1, p);
        end
        xAvg = xTotal / N;

        % --------------------- PID CONTROL ---------------------
        offset = 200;  % Bias away from right edge
        CTE = (xAvg - offset) - 320;
        fprintf("Raw CTE: %.2f\n", CTE);
        if abs(CTE) < 15
            CTE = 0;  % Deadband to ignore small noise
        end
        fprintf("Filtered CTE: %.2f\n", CTE);

        % Read PID gain values from sliders
        P = P_slider.Value;
        I = I_slider.Value;
        D = D_slider.Value;

        % Compute PID control terms
        P_term = P * CTE;
        intCTE = intCTE + CTE * dt;
        I_term = I * intCTE;
        dCTE = (CTE - prevCTE) / dt;
        D_term = D * dCTE;
        Avel = -(P_term + I_term + D_term);
        prevCTE = CTE;

        % Clamp angular velocity
        fprintf("Unclamped Avel: %.4f\n", Avel);
        Avel = max(min(Avel, 0.5), -0.5);
        Lvel = 0.4;  % constant forward velocity

        % Print PID breakdown
        fprintf("P_term: %.4f, I_term: %.4f, D_term: %.4f, Total Avel: %.4f\n", ...
            P_term, I_term, D_term, Avel);

        % Set velocity command
        velmsg.linear.x = double(Lvel);
        velmsg.angular.z = double(Avel);
        send(PubVel, velmsg);
    else
        % No lines found - stop robot
        disp("No lines found, stopping.");
        velmsg.linear.x = 0.0;
        velmsg.angular.z = 0.0;
        send(PubVel, velmsg);
    end

    % --------------------- PUBLISH VISUALIZATION ---------------------
    figure(1); imshow(overlayImage);
    laneMsg = rosWriteImage(laneMsg, overlayImage, "Encoding", "rgb8");
    send(lanePub, laneMsg);
    pause(dt);  % Wait for next cycle
end
end

% --------------------- HELPER FUNCTION: YELLOW MASK ---------------------
function [BW, maskedRGBImage] = createMask(RGB)
% CREATEMASK - Returns binary mask for yellow regions in RGB image using HSV filtering

% Convert image to HSV space
I = rgb2hsv(RGB);

% Define HSV range for yellow
channel1Min = 0.100; channel1Max = 0.300;
channel2Min = 0.401; channel2Max = 1.000;
channel3Min = 0.702; channel3Max = 1.000;

% Create mask
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
           (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
           (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Output binary mask
BW = sliderBW;

% Mask the original image
maskedRGBImage = RGB;
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end
