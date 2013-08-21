function [ output_args ] = ros_load_cmdvel( input_args ,  plot_on )
%LOADVICON Loads the ardrone cmd_vel data output by ROSbag files.
%   This script converts the csv file output from ROS into a MATLAB array
%   with field names, time-stamps. It is for a single ardrone command
%   input.
%
%   The first input must be the file name of the csv extracted using the
%   'extractBag.sh' script.
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
%data. Currently (2013-05-14 R2011b) the impoirt data function fails to
%fully recognise the data format hence some re-arranging is needed.
temp = importdata(input_args);

%Check if any data was loaded
if ~isempty(temp)
    
    %Create an empty array to store the extracted data
    output_args = struct;
    
    %Add the time column
    output_args.time = temp(:,1);                  %Add the data as text to the field 'time'

    %Add the translational input data
    output_args = setfield(output_args,'tx',temp(:,2));                    %Add the x position to the field 'tx'
    output_args = setfield(output_args,'ty',temp(:,3));                    %Add the y position to the field 'ty'
    output_args = setfield(output_args,'tz',temp(:,4));                    %Add the z position to the field 'tz'

    %Add the rotational data
    output_args = setfield(output_args,'rx',temp(:,5));                    %Add the x rotation to the field 'rx'
    output_args = setfield(output_args,'ry',temp(:,6));                    %Add the y rotation to the field 'ry'
    output_args = setfield(output_args,'rz',temp(:,7));                    %Add the z rotation to the field 'rz'

    %This completes the data loading
    
    if plot_on
        h1 = figure('name','Cmdvel: Time: T_Input'); hold on;
        temp_time = ( output_args.time - output_args.time(1) )/1000000000;
        plot(temp_time,output_args.tx,'-r');
        plot(temp_time,output_args.ty,'-g');
        plot(temp_time,output_args.tz,'-b');
        legend('x','y','z');
        xlabel('Time (s)');
        ylabel('Translational Input');

        h2 = figure('name','Cmdvel: Time: R_Input'); hold on;
        temp_time = ( output_args.time - output_args.time(1) )/1000000000;
        plot(temp_time,output_args.rx,'-r');
        plot(temp_time,output_args.ry,'-g');
        plot(temp_time,output_args.rz,'-b');
        legend('x','y','z','w');
        xlabel('Time (s)');
        ylabel('Roational Input');
    end
else
    %If the file exists but is empty then return 
    warning('ROSLIB:LOADCMDVEL','Input file is empty');
    output_args = [];
end

end %ros_load_cmdvel
