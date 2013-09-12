%Compare the Vicon and PTAM outputs in the NED reference frame.

close all;
clear all;
clc;

%Grab the log data from file
viconData = bag_load_nedvicon('vicon_ned',0);
ptamData = bag_load_nedptam('vslam_ned',0);


%Create new time axes
viconTime = (viconData.time - viconData.time(1))./1000000000;
ptamTime =  (ptamData.time - viconData.time(1)) ./1000000000;

%Plot the axes independently
h1 = figure('name','Vicon v Ptam Pos');
ax1 = subplot(3,1,1);
plot(viconTime,viconData.tx,'-r');hold on;
plot(ptamTime,ptamData.tx,'-b');
ylabel('x pos. (m)');
ax2 = subplot(3,1,2);
plot(viconTime,viconData.ty,'-r');hold on;
plot(ptamTime,ptamData.ty,'-b');
ylabel('y pos. (m)');
ax3 = subplot(3,1,3);
plot(viconTime,viconData.tz,'-r');hold on;
plot(ptamTime,ptamData.tz,'-b');
ylabel('z pos. (m)');
xlabel('time (s)');
linkaxes([ax1 ax2 ax3],'x');

%Plot the axes independently
h1 = figure('name','Vicon v Ptam Rot');
ax1 = subplot(3,1,1);
plot(viconTime,(180/pi)*viconData.rx,'-r');hold on;
plot(ptamTime,(180/pi)*ptamData.rx,'-b');
ylabel('x rot. (deg)');
ylim([-8 8]);
ax2 = subplot(3,1,2);
plot(viconTime,(180/pi)*viconData.ry,'-r');hold on;
plot(ptamTime,(180/pi)*ptamData.ry,'-b');
ylabel('y rot. (deg)');
ylim([-8 8]);
ax3 = subplot(3,1,3);
plot(viconTime,(180/pi)*viconData.rz,'-r');hold on;
plot(ptamTime,(180/pi)*ptamData.rz,'-b');
ylabel('z rot. (deg)');
xlabel('time (s)');
linkaxes([ax1 ax2 ax3],'x');



