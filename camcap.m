%% Capture, display, and store a fresh image from the camera.

img = snapshot(cam);
imwrite(img, sprintf("capimgs/testimg-%1.0f.png", round(now() * 100000)));

imagesc(img); hold on; axis equal;