function [ output_args ] = bag_load_nedcmdvel( input_args ,  plot_on )
%ROS_LOAD_NEDPTAM Loads the NED transformed CMD_VEL (drone input) data output by ROSbag files.
%   This script converts the csv file output from ROS into a MATLAB array
%   with field names, time-stamps. It is for a single Vicon pose
%   history expressed in the NED reference frame.
%
%   The first input must be the file name of the csv extracted using the
%   'extractBag_v3.sh' script.
%
%   The second optional input specifies if the data should be plotted.
%   1=plot, 0=do not plot.
%
%   The current MATLAB directory must contain the csv file. 

%Check if the plot argument has been supplied, if not then set to zero.
if nargin < 2
    plot_on = 0;
end

%If no file name use default
if nargin < 1
    plot_on = 1;
    input_args = 'cmd_vel';
end

%Check the csv file exists
if ~exist(input_args,'file')
    error('Input file not found.');
end

%Load the file using the standard MATLAB function
%NOTE: this step may cause bugs depending on how well MATLAB interprets the
%data.
temp = importdata(input_args,',',1);

%Check if any data was loaded
if ~isempty(temp)
        
    %Create an empty array to store the extracted data
    output_args = struct;
    
    %Add the time column and sequence number
    output_args = setfield(output_args,'time',temp.data(:,1));             %Add the sample times

    %Add the translational data
    output_args = setfield(output_args,'tx',temp.data(:,2));               %Add the x input to the field 'tx'
    output_args = setfield(output_args,'ty',temp.data(:,3));               %Add the y input to the field 'ty'
    output_args = setfield(output_args,'tz',temp.data(:,4));               %Add the z input to the field 'tz'

    %Add the rotational data
    output_args = setfield(output_args,'rx',temp.data(:,5));               %Add the x(phi) input to the field 'rx'
    output_args = setfield(output_args,'ry',temp.data(:,6));               %Add the y(theta) input to the field 'ry'
    output_args = setfield(output_args,'rz',temp.data(:,7));               %Add the z(psi) input to the field 'rz'
    
    %This completes the data loading

    if plot_on
        h1 = figure('name','Cmd_Vel: Time: Pos'); hold on;
        temp_time = ( output_args.time - output_args.time(1) )/1000000000;
        plot(temp_time,output_args.tx,'-r');
        plot(temp_time,output_args.ty,'-g');
        plot(temp_time,output_args.tz,'-b');
        legend('x','y','z');
        xlabel('Time (s)');
        ylabel('Pos. (m)');

        h2 = figure('name','Cmd_Vel: Time: Rot'); hold on;
        temp_time = ( output_args.time - output_args.time(1) )/1000000000;
        plot(temp_time,output_args.rx,'-r');
        plot(temp_time,output_args.ry,'-g');
        plot(temp_time,output_args.rz,'-b');
        legend('x','y','z');
        xlabel('Time (s)');
        ylabel('Rot. (Euler rads)');
    end
    
else
    %If the file exists but is empty then return 
    warning('ROSLIB:LOAD_NEDCMDVEL','Input file is empty');
    output_args = [];
end

end %ros_load_nedcmdvel


