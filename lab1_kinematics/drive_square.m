function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.

% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive
[str_l,str_r] = inverse_kinematics(sideLength/4,0);


[turn_wl,turn_wr] = inverse_kinematics(0,pi/4);

% disp([str_l,str_r],[turn_wl,turn_wr]);

for i=1:4
    pb.setVelocity([str_l,str_r],4);
    pb.setVelocity([turn_wl,turn_wr],2);
end



end