%Compare the Vicon and PTAM outputs in the NED reference frame.

close all;
clear all;
clc;

%Grab the log data from file
viconData = bag_load_nedvicon('vicon_ned',0);
cmdvelData = bag_load_nedcmdvel('cmd_vel',0);

%Pick Vicon, pos, rot axes. 1=x, 2=y, 3=z
%Pick cmdvel axis  1=x, 2=y, 3=z
viconPosAxis = 2;
viconRotAxis = 2;
cmdvelPosAxis = 1;

if viconPosAxis==1
    plotPosData = viconData.tx;
elseif viconPosAxis==2
    plotPosData = viconData.ty;
elseif viconPosAxis==3
    plotPosData = viconData.tz;
end

if viconRotAxis==1
    plotRotData = viconData.rx;
elseif viconRotAxis==2
    plotRotData = viconData.ry;
elseif viconRotAxis==3
    plotRotData = viconData.rz;
end

if cmdvelPosAxis==1
    plotInputData = cmdvelData.tx;
elseif cmdvelPosAxis==2
    plotInputData = cmdvelData.ty;
elseif cmdvelPosAxis==3
    plotInputData = cmdvelData.tz;
end

%Create new time axes
viconTime = (viconData.time - viconData.time(1))/1000000000;
cmdvelTime = (cmdvelData.time - viconData.time(1))/1000000000;

%Plot the axes independently
h1 = figure('name','Vicon vs cmdvel');
hd(1) = subplot(3,1,1);
plot(viconTime,plotPosData,'-r');hold on;
ylabel('Pos. (m)');
hd(2) = subplot(3,1,2);
plot(viconTime,plotRotData,'-r');hold on;
ylabel('Rot. (m)');
hd(3) = subplot(3,1,3);
plot(cmdvelTime,plotInputData,'-r');hold on;
ylabel('Inp.');
xlabel('time (s)');
pause(2);
linkaxes([hd(1) hd(2) hd(3)],'x');



