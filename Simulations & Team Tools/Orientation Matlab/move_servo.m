function [servo1_mag, servo2_mag] = move_servo(delta, servo1_offset, servo2_offset)
%This function finds the difference between phi and gamma

servo1_mag = cos((servo1_offset-delta)*(pi/180));
servo2_mag = cos((servo2_offset-delta)*(pi/180));


end