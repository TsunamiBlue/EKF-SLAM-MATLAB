function drive_circle(pb, radius)
% DRIVE_CIRCLE send a sequence of commands to the pibot to make it drive in a circle.

% pb is the pibot instance to send commands
% radius is the radius of the circle to drive

q = 2*pi/40;

u = 2*radius*pi/40;

[wl,wr] = inverse_kinematics(u,q);
for i=1:4
pb.setVelocity([wl,wr],10);

end