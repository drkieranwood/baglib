%Compare the Vicon and PTAM outputs in the NED reference frame.

close all;
clear all;
clc;

%Grab the log data from file
viconData = bag_load_nedvicon('vicon_ned',0);
ptamData = bag_load_nedptam('vslam_ned',0);
refer_on = 0;
if (exist('refer_ned','file'))
    referData = bag_load_nedrefer('refer_ned',0);
    refer_on = 1;
end


%Create new time axes
viconTime = (viconData.time - viconData.time(1))./1000000000;
ptamTime =  (ptamData.time - viconData.time(1)) ./1000000000;
if (refer_on)
    referTime =  (referData.time - viconData.time(1)) ./1000000000;
end

%Plot the axes independently
h1 = figure('name','Vicon v Ptam Pos');
ax1 = subplot(3,1,1);
plot(viconTime,viconData.tx,'-k');hold on;
plot(ptamTime,ptamData.tx,'--k');
ylabel('X pos. (m)');
ax2 = subplot(3,1,2);
plot(viconTime,viconData.ty,'-k');hold on;
plot(ptamTime,ptamData.ty,'--k');
ylabel('Y pos. (m)');
ax3 = subplot(3,1,3);
plot(viconTime,viconData.tz,'-k');hold on;
plot(ptamTime,ptamData.tz,'--k');
ylabel('Z pos. (m)');
xlabel('Time (s)');
linkaxes([ax1 ax2 ax3],'x');
legend('Vicon','PTAM');

%Plot the axes independently
h1 = figure('name','Vicon v Ptam Rot');
ax1 = subplot(3,1,1);
plot(viconTime,(180/pi)*viconData.rx,'-k');hold on;
plot(ptamTime,(180/pi)*ptamData.rx,'--k');
ylabel('x rot. (deg)');
ylim([-8 8]);
ax2 = subplot(3,1,2);
plot(viconTime,(180/pi)*viconData.ry,'-k');hold on;
plot(ptamTime,(180/pi)*ptamData.ry,'--k');
ylabel('y rot. (deg)');
ylim([-8 8]);
ax3 = subplot(3,1,3);
plot(viconTime,(180/pi)*viconData.rz,'-k');hold on;
plot(ptamTime,(180/pi)*ptamData.rz,'--k');
ylabel('z rot. (deg)');
xlabel('time (s)');
linkaxes([ax1 ax2 ax3],'x');
legend('Vicon','PTAM');

h1=figure('name','XYZ Plot Vicon');
downSampleCount = 20;
tempL = kl(viconData.tx);
for ii=1:1:tempL
    normsTemp(ii) = norm([viconData.tx(ii) viconData.ty(ii) viconData.tz(ii)]);
end
viconSpeed = diff(normsTemp);
viconSpeed(end+1) = viconSpeed(end);
viconSpeed = viconSpeed/0.1;
viconSpeed = viconSpeed.*(abs(viconSpeed)<0.1);
viconSpeed = abs(viconSpeed);
scatter3(viconData.tx(1:downSampleCount:tempL),viconData.ty(1:downSampleCount:tempL),viconData.tz(1:downSampleCount:tempL),1,viconTime(1:downSampleCount:tempL));
view(0,-90)
xlabel('X pos. (m)');
ylabel('Y pos. (m)');
zlabel('Z pos. (m)');
hold on;

for ii=1:1:tempL
    normsTemp(ii) = norm([ptamData.tx(ii) ptamData.ty(ii) ptamData.tz(ii)]);
end
ptamSpeed = diff(normsTemp);
ptamSpeed(end+1) = ptamSpeed(end);
ptamSpeed = ptamSpeed/0.1;
ptamSpeed = smooth(ptamSpeed,10);
ptamSpeed = ptamSpeed.*(abs(ptamSpeed)<0.1);
ptamSpeed = abs(ptamSpeed);
scatter3(ptamData.tx(1:downSampleCount:tempL),ptamData.ty(1:downSampleCount:tempL),ptamData.tz(1:downSampleCount:tempL),5,ptamTime(1:downSampleCount:tempL));
if (refer_on)
    plot3(referData.tx(1:downSampleCount:tempL),referData.ty(1:downSampleCount:tempL),referData.tz(1:downSampleCount:tempL),'-k');
end
colorbar;

if 1
    h1=figure('name','XY Plot Vicon');
    hold on;
    startIdx = 5500;
    midIdx = 15920;
    endIdx = 27000;
    plot(ptamData.ty(startIdx:midIdx),ptamData.tx(startIdx:midIdx),'--k');
    plot(ptamData.ty(midIdx:endIdx),ptamData.tx(midIdx:endIdx),'-k');
    plot(ptamData.ty(startIdx),ptamData.tx(startIdx),'xk');
    plot(ptamData.ty(midIdx),ptamData.tx(midIdx),'sk');
    plot(ptamData.ty(endIdx),ptamData.tx(endIdx),'ok');
    legend('Exploration','Return','Start','Turn-Around','End','location','nw');
    xlabel('$y_{I}$ position (m)');
    ylabel('$x_{I}$ position (m)');
    axis square;
end


