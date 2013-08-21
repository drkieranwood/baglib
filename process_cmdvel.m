%This script loads the cmd_vel inputs and creates some extra comparison
%plots with the Vicon and PTAM data

%Clear up
close all;
%clear all;
clc;

%Clear variables that are set by this script
clear cmdvelRaw;

%Set the debug flags
debug_on = 1;

%Load the required data
cmdvelRaw = ros_load_cmdvel('cmd_vel',0);

%Check if the Vicon and PTAM data is available
viconPresent = ~isempty(viconRaw);
vslamPresent = ~isempty(vslamRaw);
cmdvelPresent = ~isempty(cmdvelRaw);


if (cmdvelPresent)
    %There is no transform that needs to be applied to the cmdvel data
    %If a plot of the raw input is wanted then use
    % ros_load_cmdvel('file',1)
else
    %The cmdvel file was empty
    warning('ROSLIB:PROCESSCMDVEL','This script requires cmdvel data.');
end %cmdvelPresent



if (cmdvelPresent && viconPresent)
    %Adjust the timings to match the Vicon times
    cmdvelRaw.time(:) = (cmdvelRaw.time(:)-initTime)/1000000000;
        
    %Plot 
    if debug_on
        h1 = figure('name','DEBUG: Vicon IO X'); hold on;
        plot(cmdvelRaw.time,cmdvelRaw.tx(:),'-r');
        plot(viconRaw.time,viconRaw.ty(:),'-g');
        ylabel('X IO');
        xlabel('Time (s)');
        legend('Input','Output');
        ylim([-10 10]);
    end
        
    if debug_on
        h1 = figure('name','DEBUG: Vicon IO Rot.'); hold on;
        plot(cmdvelRaw.time,cmdvelRaw.tx(:),'-r');
        plot(viconRaw.time,viconRaw.rw(:),'-g');
        plot(viconRaw.time,viconRaw.rx(:),'-g');
        plot(viconRaw.time,viconRaw.ry(:),'-g');
        plot(viconRaw.time,viconRaw.rz(:),'-g');
        ylabel('X IO');
        xlabel('Time (s)');
        legend('Input','Output');
        ylim([-10 10]);
    end
    
else
    %If either the Vicon or cmdvel data file was empty
    warning('ROSLIB:PROCESSCMDVEL','This script requires both Vicon and cmdvel data.');
    
end %cmdvelPresent && viconPresent



%Add the input command data
if (cmdvelPresent && vslamPresent)
    %If PTAM is present Vicon must also be present and the times have
    %already been adjusted.
    
    
    %Plot 
    if debug_on
        h1 = figure('name','DEBUG: PTAM IO X'); hold on;
        plot(cmdvelRaw.time,cmdvelRaw.tx(:),'-r');
        plot(vslamTr.time,vslamTr.ty(:),'-g');
        ylabel('X IO');
        xlabel('Time (s)');
        legend('Input','Output');
        ylim([-10 10]);
    end
    
    if debug_on
        h1 = figure('name','DEBUG: PTAM IO Rot.'); hold on;
        plot(cmdvelRaw.time,cmdvelRaw.rz(:),'-r');
        plot(vslamTr.time,vslamRaw.rw(:),'-g');
        plot(vslamTr.time,vslamRaw.rx(:),'-g');
        plot(vslamTr.time,vslamRaw.ry(:),'-g');
        plot(vslamTr.time,vslamRaw.rz(:),'-g');
        ylabel('X IO');
        xlabel('Time (s)');
        legend('Input','Output');
        ylim([-10 10]);
    end
    
else
    %If either the PTAM or cmdvel data file was empty
    warning('ROSLIB:PROCESSCMDVEL','This script requires both PTAM and cmdvel data.');
    
end %cmdvelPresent && vslamPresent

    
    
%At the end of this script there will exist the following variables
% cmdvelRaw      - unaltered cmdvel values from file (no time adjustment)

