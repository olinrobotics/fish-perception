%% Take a picture from the camera and process it to detect the laser beam

%% Capture the image
img = snapshot(cam);

%% Quickly display the raw capture
figure(1); clf; imagesc(imrotate(img, -91.5)); title("Raw Image"); axis equal; colormap gray;

%% Store the image for later testing
imwrite(img, sprintf("testimgs/testimg-%1.0f.png", round(now() * 100000)));

%% Clean up the image
img = undistortImage(img, camParams);
img = imrotate(img, -91.5);
img = img(:, :, 2);  % green channel only
img = img > 75;  % turns each pixel into a boolean depending on brightness

%% Find laser

%% Some Constants

LASER_ANGLE = 87.457; % theta, deg (this angle is from the CAD)
LASER_ZERO_PIXEL_ROW = 331; % Y_0, experimentally determined
ZERO_POINT_DISTANCE = 4000000;
DISTANCE_SCALAR = -0.01535; % alpha, experimentally determined 


%% Setup
num_cols = size(img, 2);
num_rows = size(img, 1);

positions = NaN(1, num_cols);
distances = NaN(1, num_cols);
ren3d = NaN(3, num_cols);

%% Find the largest streak in each column
for col=1:num_cols
    on_streak = false;
    max_streak_idx = 0;
    max_streak_len = 0;
    cur_streak_idx = 0;
    cur_streak_len = 0;
    
    for row=1:num_rows
        if img(row, col)
            if ~on_streak
                on_streak = true;
                cur_streak_idx = row;
                cur_streak_len = 0;
            end
            
            cur_streak_len = cur_streak_len + 1;
            
            if cur_streak_len > max_streak_len
                max_streak_idx = cur_streak_idx;
                max_streak_len = cur_streak_len;
            end
        else
            on_streak = false;
            cur_streak_idx = 0;
            cur_streak_len = 0;
        end
    end
    
    %% Find the center of the largest streak
    position = max_streak_idx + round(max_streak_len / 2);
     
    if max_streak_len < 1
        position = NaN;
    end
    
    %% Calculate the physical distance
    % Old version: distance =  ZERO_POINT_DISTANCE - ((LASER_ZERO_PIXEL_ROW - position) * METERS_OFFSET_PER_PIXEL_ROW);
    distance = ((LASER_ZERO_PIXEL_ROW - position) * atand(LASER_ANGLE)) * DISTANCE_SCALAR;

    %% Store results
    positions(col) = position;
    distances(col) = distance;
    ren3d(:, col) = [col; position; distance];
end
figure(2); clf;
 
imagesc(img(:, :)); hold on; axis equal; colormap gray; % plot the image
plot(positions, 'LineWidth', 2); % plot the laser centerline
yline(LASER_ZERO_PIXEL_ROW, "r", "Zero Point"); % plot the zero line

title("Processed Frame");
legend("Laser Centerline");

%% Dead Code, no longer used
% laser_dy = positions(end) - positions(1)
% laser_dx = size(positions, 2)
% laser_angle = atan(laser_dx / laser_dy)
% 
% mean_pos = mean(positions(~isnan(positions)));
% mean_pos_adj = mean_pos - LASER_ZERO_PIXEL_ROW;
% mean_dist = mean(distances(~isnan(distances)));
% 
% min_dist = min(distances);
% max_dist = max(distances);
% d_dist = max_dist - min_dist;
% 
% min_pos = min(positions);
% max_pos = max(positions);
% d_pos = max_pos - min_pos;


%% 3D Plot
return % currently disabled
figure(2);

xs = movmean(ren3d(1, :), 15);
ys = movmean(ren3d(3, :), 15);
zs = movmean(ren3d(2, :), 15);

plot3(xs, ys, zs)
xlabel("Column (px)"); ylabel("Distance (m)"); zlabel("Row (px)");
grid on;