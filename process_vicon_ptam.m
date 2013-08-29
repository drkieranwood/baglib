%This script loads the vicon and PTAM pose time histories and compares
%them.
%
%The data is pre-processed the data into a common time-frame using linear 
%interpolation. 
%
%Since the PTAM reference frame is at an arbitrary orientation a best fit
%between the PTAM and vicon is found. All data is converted to use the NED 
%(North-East-Down) reference frame.

%Clear up and disable rounding warnings.
close all;
clc;
warning('off','KROTLIB:rounding');

%Clear variables that are set by this script
clear viconRaw vslamRaw viconRe vslamTr vslamNoise initTime tr fitQuality;
clear procrustes_on debug_on plot_on;

%Set the debug flags to control plotting
debug_on = 1;
plot_on = 1;

%Which VSLAM orietation method to use
% 1 - use procrustes
% 0 - use initial frame
procrustes_on = 0;

%Load the required data
viconRaw = ros_load_vicon('vicon_archie',0);
vslamRaw = ros_load_ptam('ptam_pose_world',0);

%Check if the Vicon and PTAM data is available
viconPresent = ~isempty(viconRaw);
vslamPresent = ~isempty(vslamRaw);
if (viconPresent && vslamPresent)

    %======================================================================
    % TIME ADJUSTMENT
    %======================================================================
    %Change the time fields to be in seconds and have a common zero.
    initTime = viconRaw.time(1);
    viconRaw.time(:,1) = (viconRaw.time(:,1)-initTime)/1000000000;
    vslamRaw.time(:,1) = (vslamRaw.time(:,1)-initTime)/1000000000;
    
    %======================================================================
    % VICON: INTO NED REFERENCE FRAME
    %======================================================================
    %NOTE: PERNAMENT CHANGE OF REFERENCE FRAME
    %Change the Vicon axes directions. The X position and orientation is
    %correct so leave that the same. The Y and Z axes are backwards so swap
    %their sign.

    %Just swap the axes sign
    viconRaw.tx = viconRaw.tx;
    viconRaw.ty = -viconRaw.ty;
    viconRaw.tz = -viconRaw.tz;

    %Just swap the axes sign
    viconRaw.rw = viconRaw.rw;
    viconRaw.rx = viconRaw.rx;
    viconRaw.ry = -viconRaw.ry;
    viconRaw.rz = -viconRaw.rz; 
    
    %Add Euler angles for ease
    for ii=1:1:length(viconRaw.time)
        temp = r_q_to_e([viconRaw.rw(ii);viconRaw.rx(ii);viconRaw.ry(ii);viconRaw.rz(ii)]);
        viconRaw.rphi(ii,1)   = temp(1);
        viconRaw.rtheta(ii,1) = temp(2);
        viconRaw.rpsi(ii,1)   = temp(3);
    end
    clear ii temp;

    %======================================================================
    % VICON: CAMERA OFFSET
    %======================================================================
    %Add an artificial camera offset effect. It is a body offset of [0.195;0;0] 
    %therefore needs rotating into the world frame, by applying the current
    %orientation inverse (the effect is quite small for pitching motions
    %but can become more pronounced for larger yaw motions).
if 0
    for ii=1:1:length(viconRaw.time)
        temp = r_apply_q([0.195;0;0],r_inv_q([viconRaw.rw(ii,1);viconRaw.rx(ii,1);viconRaw.ry(ii,1);viconRaw.rz(ii,1)]));
        viconRaw.tx(ii,1) = viconRaw.tx(ii,1) + temp(1);
        viconRaw.ty(ii,1) = viconRaw.ty(ii,1) + temp(2);
        viconRaw.tz(ii,1) = viconRaw.tz(ii,1) + temp(3);
    end
    clear temp;
end
       
    %======================================================================
    % VICON: PLOTS
    %======================================================================
    if debug_on
        %Plot the original Vicon data after its tranformation into NED
        h1 = figure('name','DEBUG: Vicon in NED: Translation'); hold on;
        plot(viconRaw.time,viconRaw.tx,'-r');
        plot(viconRaw.time,viconRaw.ty,'-g');
        plot(viconRaw.time,viconRaw.tz,'-b');
        xlabel('Time (s)');
        ylabel('Pos. (m)');
        legend('Vicon x','Vicon y','Vicon z');
        
        h1 = figure('name','DEBUG: Vicon in NED: Rotation'); hold on;
        plot(viconRaw.time,(180/pi)*viconRaw.rphi,'-r');
        plot(viconRaw.time,(180/pi)*viconRaw.rtheta,'-g');
        plot(viconRaw.time,(180/pi)*viconRaw.rpsi,'-b');
        xlabel('Time (s)');
        ylabel('Rot. (Degs.)');
        legend('Vicon \phi','Vicon \theta','Vicon \psi');
        
        h1 = figure('name','DEBUG: Vicon in NED: 3D Trace'); hold on;
        plot3(viconRaw.tx,viconRaw.ty,viconRaw.tz);
        xlabel('x (m)');
        ylabel('y (m)');
        zlabel('z (m)');
        view(3);
    end
    
    %At this point we have the Vicon data converted into the NED reference
    %frame. Additional Euler rotations have been added to the data set for ease
    %of understanding [phi;theta;psi]

    
    %======================================================================
    % VICON: INTO VSLAM TIME FRAME
    %======================================================================
    %The data needs to have common sample times for an accurate comaprison. The
    %Vicon data can be interpolated to provide samples at the same times as the
    %PTAM samples. The interp1() function is used for this.
    viconRe.time = vslamRaw.time;
    viconRe.tx = interp1(viconRaw.time(:),viconRaw.tx(:),vslamRaw.time(:));
    viconRe.ty = interp1(viconRaw.time(:),viconRaw.ty(:),vslamRaw.time(:));
    viconRe.tz = interp1(viconRaw.time(:),viconRaw.tz(:),vslamRaw.time(:));

    viconRe.rw = interp1(viconRaw.time(:),viconRaw.rw(:),vslamRaw.time(:));
    viconRe.rx = interp1(viconRaw.time(:),viconRaw.rx(:),vslamRaw.time(:));
    viconRe.ry = interp1(viconRaw.time(:),viconRaw.ry(:),vslamRaw.time(:));
    viconRe.rz = interp1(viconRaw.time(:),viconRaw.rz(:),vslamRaw.time(:));

    viconRe.rphi   = interp1(viconRaw.time(:),viconRaw.rphi(:)  ,vslamRaw.time(:));
    viconRe.rtheta = interp1(viconRaw.time(:),viconRaw.rtheta(:),vslamRaw.time(:));
    viconRe.rpsi   = interp1(viconRaw.time(:),viconRaw.rpsi(:)  ,vslamRaw.time(:));
    
    %Re-normalise the quarternions. This is not a perfect way to
    %interpolate quarternions but it is close for fast sampling. See the
    %'slerp' algorithm or 'quarternion interpolation'
    for ii = 1:1:length(viconRe.time);
        temp = r_norm_q([viconRe.rw(ii);viconRe.rx(ii);viconRe.ry(ii);viconRe.rz(ii)]);
        viconRe.rw(ii,1) = temp(1);
        viconRe.rx(ii,1) = temp(2);
        viconRe.ry(ii,1) = temp(3);
        viconRe.rz(ii,1) = temp(4);
    end
    clear ii temp;
    
    if debug_on
        %Plot the raw PTAM tx values against the re-sampled Vicon tx values 
        h1 = figure('name','DEBUG: Resampling Comparison: TX'); hold on;
        n_samples = 100;
        
        %Find the times that relate to the first n samples
        pt_low = vslamRaw.time(1);
        pt_high = vslamRaw.time(n_samples);
        [~, temp] = min(abs(viconRaw.time(:)-pt_low));
        vidx_low = temp;
        [~, temp] = min(abs(viconRaw.time(:)-pt_high));
        vidx_high = temp;
        
        %Plot over those ranges
        plot(viconRaw.time(vidx_low:vidx_high),viconRaw.tx(vidx_low:vidx_high),'-r');
        plot(vslamRaw.time(1:n_samples),viconRe.tx(1:n_samples),'-sg');
        plot(vslamRaw.time(1:n_samples),vslamRaw.tx(1:n_samples),'-ob');
        legend('Vicon Raw','Vicon Resamp.','PTAM Raw');
        xlabel('Time (s)');
        ylabel('Pos. (m)');

        %Clear unwanted junk
        clear n_samples pt_low pt_high vidx_low vidx_high a temp ii;
    end
    
    %======================================================================
    % VSLAM: INTO NED REFERENCE FRAME
    %======================================================================
    if ~procrustes_on
        
        %Create output structure
        vslamTr = struct;
        vslamTr = setfield(vslamTr,'time',vslamRaw.time);
        
        %Temporarily use procrustes to get the scale
        if 1
            %Choose the range in seconds to perform the data fit.
            t_low = vslamRaw.time(1);
            t_high = t_low+65;
            %Find the nearest index values from the vslam time vector
            [~,temp] = min(abs(vslamRaw.time(:)-t_low));
            idx_low = temp;
            [~,temp] = min(abs(vslamRaw.time(:)-t_high));
            idx_high = temp;
            %Create the input and output 3D vectors [x(:) y(:) z(:)]
            temp_vicon_points = [viconRe.tx(idx_low:idx_high)    viconRe.ty(idx_low:idx_high)    viconRe.tz(idx_low:idx_high) ];
            temp_vslam_points = [vslamRaw.tx(idx_low:idx_high)   vslamRaw.ty(idx_low:idx_high)   vslamRaw.tz(idx_low:idx_high)];

            %The function to apply the transform is: Z = b*Y*T + c
            [fitQuality,~,tr] = procrustes(temp_vicon_points,temp_vslam_points);
            if fitQuality > 0.2
                disp('The procrustes fit was poor');
                return;
            end
        end
        clear t_low t_high a idx_low idx_high fitQuality temp temp_vicon_points temp_vslam_points;
           
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Invert affine
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Firstly invert all of the output from VSLAM
        %Inverse orientation (just make x,y,z parts negative)
        vslamTr.rw(:,1) = vslamRaw.rw(:,1);
        vslamTr.rx(:,1) = -vslamRaw.rx(:,1);
        vslamTr.ry(:,1) = -vslamRaw.ry(:,1);
        vslamTr.rz(:,1) = -vslamRaw.rz(:,1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Convert Position
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Apply PTAM rotation to each SCALED position vector. This basically
        %projects the position into the world frame coordinate system.
        for ii = 1:1:length(vslamTr.time)
            temp = (-1)*r_apply_q([vslamRaw.tx(ii,1)*tr.b; vslamRaw.ty(ii,1)*tr.b; vslamRaw.tz(ii,1)*tr.b],[vslamTr.rw(ii,1); -vslamTr.rx(ii,1); -vslamTr.ry(ii,1); -vslamTr.rz(ii,1)]);
            vslamTr.tx(ii,1) = temp(1);
            vslamTr.ty(ii,1) = temp(2);
            vslamTr.tz(ii,1) = temp(3);         
        end
        clear temp;

        %Remove initial position from all following positions
        initpos = [vslamTr.tx(1,1);vslamTr.ty(1,1);vslamTr.tz(1,1)];
        for ii = 1:1:length(vslamTr.time)
            vslamTr.tx(ii,1) = vslamTr.tx(ii,1) - initpos(1);
            vslamTr.ty(ii,1) = vslamTr.ty(ii,1) - initpos(2);
            vslamTr.tz(ii,1) = vslamTr.tz(ii,1) - initpos(3);
        end
        clear initpos;
 
        %Unrotate all relative positions by the initial orientation to get relative
        %movements in the camera axes. Swap the axes order (x=z,y=x,z=y)
        %into NED. Then rotate by the initial Vicon orientation and add the
        %initial Vicon position.
        for ii = 1:1:length(vslamTr.time)
            temp =  (-1)*r_apply_q([vslamTr.tx(ii,1); vslamTr.ty(ii,1); vslamTr.tz(ii,1)],[vslamTr.rw(1,1);vslamTr.rx(1,1);vslamTr.ry(1,1);vslamTr.rz(1,1)]);
            temp = [temp(3);temp(1);temp(2)];
            temp2 = (-1)*r_apply_q([temp(1);temp(2);temp(3)],r_inv_q([viconRe.rw(1,1);viconRe.rx(1,1);viconRe.ry(1,1);viconRe.rz(1,1)]));
            vslamTr.tx(ii,1) = temp2(1) + viconRe.tx(1,1);
            vslamTr.ty(ii,1) = temp2(2) + viconRe.ty(1,1);
            vslamTr.tz(ii,1) = temp2(3) + viconRe.tz(1,1);
        end
        clear temp;
       
       
      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Convert Orientation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Subtract the initial orientation from all subsequent orientations
        %to get a relative orientation. Then convert from the vision
        %coordinate system to the Vicon NED system (w=w,x=z,y=x,z=y),
        %finally apply the initial Vicon offset.
        meanphi = 0;
        meantheta = 0;
        jj=0;
        initq = [vslamTr.rw(1,1); vslamTr.rx(1,1); vslamTr.ry(1,1); vslamTr.rz(1,1)];
        for ii = 1:1:length(vslamTr.time)
            %1) Find relative quaternion, qt = q0^-1 * qi
            temp  = r_multi_q(r_inv_q(initq),[vslamTr.rw(ii,1); vslamTr.rx(ii,1); vslamTr.ry(ii,1); vslamTr.rz(ii,1)]);
            
            %2) Swap axes to convert from vision(v) to NED(n) (wn=wv,xn=zv,yn=xv,zn=yv)
            temp2 = [temp(1) ; temp(4) ; temp(2) ; temp(3)];
            
            if 1
                %Find euler
                tempeuler = r_q_to_e(temp2);
                %Find means
                jj=jj+1;
                meanphi   = ((meanphi*(jj-1))  +tempeuler(1))/jj;
                meantheta = ((meantheta*(jj-1))+tempeuler(2))/jj;
                %Reconstruct into quaternion
                tempq = r_e_to_q([meanphi;meanphi;0]);
                temp3 = r_multi_q(r_inv_q(tempq),temp2);
            else
                %3) Add on the initial vicon orientation(qv0) offset, q = qv0 * qi 
                temp3 = r_multi_q([viconRe.rw(1,1); viconRe.rx(1,1); viconRe.ry(1,1); viconRe.rz(1,1)],temp2);
            end
            
            vslamTr.rw(ii,1) = temp3(1);
            vslamTr.rx(ii,1) = temp3(2);
            vslamTr.ry(ii,1) = temp3(3);
            vslamTr.rz(ii,1) = temp3(4);
        end
        clear temp temp2 temp3 tempeuler tempq;
         
        %Add Euler angles for ease
        for ii=1:1:length(vslamTr.time)
            warning off;
            temp = r_q_to_e([vslamTr.rw(ii);vslamTr.rx(ii);vslamTr.ry(ii);vslamTr.rz(ii)]);
            vslamTr.rphi(ii,1)   = temp(1);
            vslamTr.rtheta(ii,1) = temp(2);
            vslamTr.rpsi(ii,1)   = temp(3);
            warning on;
        end 
        clear temp;
        
        
if 1
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Correct for camera offset (post)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Basically the camera is 0.195m further back in the body x axis than reported. Hence
        %all measurements need 0.195 removing in the (rotated)x direction.
        %This uses the current orientation in the NED frame hence must
        %occur after the other transforms
        for ii=1:1:length(vslamTr.time)
%             rotdiff(ii,1) = viconRe.rw(ii,1) - vslamTr.rw(ii,1);
%             rotdiff(ii,2) = viconRe.rx(ii,1) - vslamTr.rx(ii,1);
%             rotdiff(ii,3) = viconRe.ry(ii,1) - vslamTr.ry(ii,1);
%             rotdiff(ii,4) = viconRe.rz(ii,1) - vslamTr.rz(ii,1);
            temp = r_apply_q([0.195;0;0],r_inv_q([vslamTr.rw(ii,1);vslamTr.rx(ii,1);vslamTr.ry(ii,1);vslamTr.rz(ii,1)]));
            if ii==1
                initoffset = temp;
            end
            vslamTr.tx(ii,1) = vslamTr.tx(ii,1) - (temp(1) - initoffset(1));
            vslamTr.ty(ii,1) = vslamTr.ty(ii,1) - (temp(2) - initoffset(2));
            vslamTr.tz(ii,1) = vslamTr.tz(ii,1) - (temp(3) - initoffset(3));
        end
        clear temp;
end
figure;hold on;
% plot(vslamTr.time,rotdiff(:,1),'-r');
% plot(vslamTr.time,rotdiff(:,2),'-g');
% plot(vslamTr.time,rotdiff(:,3),'-b');
% plot(vslamTr.time,rotdiff(:,4),'-m');
    


    else    
        
        
        %===============================================
        %At this point we have [tx,ty,tz],[rw,rx,ry,rz] for the Vicon and PTAM
        %time histories with common samples at the vslamRaw.time() moments.
        %
        %The PTAM data is now transformed to best fit the Vicon data over a
        %specified range.
        %===============================================
        %Remove initialVSLAM position
        vslamTr = struct;
        vslamTr = setfield(vslamTr,'time',vslamRaw.time);
%         vslamTr.tx(:,1) = vslamRaw.tx(:,1) - vslamRaw.tx(1);
%         vslamTr.ty(:,1) = vslamRaw.ty(:,1) - vslamRaw.ty(1);
%         vslamTr.tz(:,1) = vslamRaw.tz(:,1) - vslamRaw.tz(1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Invert affine
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Firstly invert all of the output from VSLAM
        %Inverse orientation (just make x,y,z parts negative)
        vslamTr.rw(:,1) = vslamRaw.rw(:,1);
        vslamTr.rx(:,1) = -vslamRaw.rx(:,1);
        vslamTr.ry(:,1) = -vslamRaw.ry(:,1);
        vslamTr.rz(:,1) = -vslamRaw.rz(:,1);
        
        %Apply inverse rotation to each position vector.
        %Also swap the axes to be correct
        for ii = 1:1:length(vslamTr.time)
            temp = (-1)*r_apply_q([vslamRaw.tx(ii,1); vslamRaw.ty(ii,1); vslamRaw.tz(ii,1)],[vslamTr.rw(ii,1); vslamTr.rx(ii,1); vslamTr.ry(ii,1); vslamTr.rz(ii,1)]);
            vslamTr.tx(ii,1) = temp(1);
            vslamTr.ty(ii,1) = temp(2);
            vslamTr.tz(ii,1) = temp(3);
        end



        %Choose the range in seconds to perform the data fit.
        t_low = vslamTr.time(1);
        t_high = t_low+65;
        %Find the nearest index values from the vslam time vector
        [a temp] = min(abs(vslamTr.time(:)-t_low));
        idx_low = temp;
        [a temp] = min(abs(vslamTr.time(:)-t_high));
        idx_high = temp;
        %Create the input and output 3D vectors [x(:) y(:) z(:)]
        temp_vicon_points = [viconRe.tx(idx_low:idx_high)    viconRe.ty(idx_low:idx_high)    viconRe.tz(idx_low:idx_high) ];
        temp_vslam_points = [vslamTr.tx(idx_low:idx_high)    vslamTr.ty(idx_low:idx_high)    vslamTr.tz(idx_low:idx_high) ];

        %Clear unwanted junk
        clear t_low t_high a idx_low idx_high;

        %Solve for scale, tanslation, and orientatrion
        %d=quality of least-square fit
        %Z=the input Y with the calculated transform applied
        %tr.c=translation
        %tr.T=rotation
        %tr.b=scale
        %The function to apply the transform is: Z = b*Y*T + c
        [fitQuality,~,tr] = procrustes(temp_vicon_points,temp_vslam_points);

        %Clear unwanted junk
        clear temp_vicon_points temp_vslam_points;

        %Check the quality of the fit
        disp(['=================================']);
        disp(['Procrustes fit quality = ' num2str(fitQuality)]);
        disp(['=================================']);

        if debug_on
            %Display the transform
            disp('Scale');
            disp(tr.b);
            disp('Rotation');
            disp(tr.T);
            disp('Translation');
            disp(tr.c(1,:));
        end

        %Now apply the transform to the whole vslam vector
        temp_vslam_points = [vslamTr.tx vslamTr.ty vslamTr.tz];
        temp = tr.b*temp_vslam_points*tr.T;
        for ii = 1:1:length(temp)
            temp(ii,1:3) = temp(ii,1:3) + tr.c(1,:);
        end
        vslamTr.tx = temp(:,1);
        vslamTr.ty = temp(:,2);
        vslamTr.tz = temp(:,3);
        
        vslamTr.rw = vslamRaw.rw;
        vslamTr.rx = vslamRaw.rx;
        vslamTr.ry = vslamRaw.ry;
        vslamTr.rz = vslamRaw.rz;
        %Add Euler angles for ease
        for ii=1:1:length(vslamTr.time)
            warning off;
            temp = r_q_to_e([vslamTr.rw(ii);vslamTr.rx(ii);vslamTr.ry(ii);vslamTr.rz(ii)]);
            vslamTr.rphi(ii,1)   = temp(1);
            vslamTr.rtheta(ii,1) = temp(2);
            vslamTr.rpsi(ii,1)   = temp(3);
            warning on;
        end    
    
    end %procrustes_on
    %======================================================================
   
    
    if debug_on
        %Plot the vslam data that should now be aligned in the NED
        %reference frame
        h1 = figure('name','DEBUG: Vslam in NED: Translation'); hold on;
        plot(vslamTr.time,vslamTr.tx,'-r');
        plot(vslamTr.time,vslamTr.ty,'-g');
        plot(vslamTr.time,vslamTr.tz,'-b');
        xlabel('Time (s)');
        ylabel('Pos. (m)');
        legend('Vslam x','Vslam y','Vslam z');
        
        h1 = figure('name','DEBUG: Vslam in NED: Rotation'); hold on;
        plot(vslamTr.time,(180/pi)*vslamTr.rphi,'-r');
        plot(vslamTr.time,(180/pi)*vslamTr.rtheta,'-g');
        plot(vslamTr.time,(180/pi)*vslamTr.rpsi,'-b');
        xlabel('Time (s)');
        ylabel('Rot. (Degs.)');
        legend('Vslam \phi','Vslam \theta','Vslam \psi');
    end
    
    if plot_on
        %Plot the original Vicon data against the transformed Vslam data
        h1 = figure('name','PLOT: Transform Comparison: Translation'); hold on;
        plot(viconRaw.time,viconRaw.tx,'-r');
        plot(viconRaw.time,viconRaw.ty,'-g');
        plot(viconRaw.time,viconRaw.tz,'-b');
        plot(vslamRaw.time,vslamTr.tx,'xr');
        plot(vslamRaw.time,vslamTr.ty,'xg');
        plot(vslamRaw.time,vslamTr.tz,'xb');

        xlabel('Time (s)');
        ylabel('Pos. (m)');
        legend('ViconX','ViconY','ViconZ','VslamX','VslamY','VslamZ');
        
        %Plot the original Vicon data against the transformed Vslam data
        %Euler angles
        h1 = figure('name','PLOT: Transform Comparison: Rotation'); hold on;
        plot(viconRaw.time,viconRaw.rphi,'-r');
        plot(viconRaw.time,viconRaw.rtheta,'-g');
        plot(viconRaw.time,viconRaw.rpsi,'-b');
        plot(vslamRaw.time,vslamTr.rphi,'xr');
        plot(vslamRaw.time,vslamTr.rtheta,'xg');
        plot(vslamRaw.time,vslamTr.rpsi,'xb');
        
        plot(vslamRaw.time,mean(viconRaw.rphi),'-r');
        plot(vslamRaw.time,mean(viconRaw.rtheta),'-g');
        xlabel('Time (s)');
        ylabel('Rot. (rads)');
        legend('Vicon \phi','Vicon \theta','Vicon \psi','Vslam \phi','Vslam \theta','Vslam \psi');
        
        if 1
            %Plot the original Vicon data against the transformed Vslam
            %data Quarternion
            h1 = figure('name','PLOT: Transform Comparison: Rotation2'); hold on;
            plot(viconRaw.time,viconRaw.rw,'-y');
            plot(viconRaw.time,viconRaw.rx,'-r');
            plot(viconRaw.time,viconRaw.ry,'-g');
            plot(viconRaw.time,viconRaw.rz,'-b');
            plot(vslamRaw.time,vslamTr.rw,'xy');
            plot(vslamRaw.time,vslamTr.rx,'xr');
            plot(vslamRaw.time,vslamTr.ry,'xg');
            plot(vslamRaw.time,vslamTr.rz,'xb');
            xlabel('Time (s)');
        ylabel('Rot. (rads)');
        legend('ViconW','ViconX','ViconY','ViconZ','VslamW','VslamX','VslamY','VslamZ');
        end
    end 
    
    %Clear unwanted junk
    clear temp_vslam_points temp ii;
   
    
    %======================================================================
    % VSLAM NOISE
    %======================================================================
    %Work on the rotation matrix
    %The rotation is applied by Z = YT which is backwards to the normal
    %rotation matrix which pre-miltiplies.
    %This is becasue it is rotating row vectors of XYZ data rather than column
    %vectors.
    %
    %To find the common column vector rotation matrix take the transpose
    %   Z = YT        - transormation provided by procrustes
    %   Z' = (YT)'    - transpose both sides to get data as column vectors
    %   Z' = T'Y'     - apply matrix rules to find the pre-multiply rotation matrix
    %Hence T' is the actual direction cosine matrix being applied matrix.
    %   viconVector = (T')*vslamvector
    %Hence T' = R_{slam->vicon}
    %So anything in the vslam frame (vectors etc..) needs to be multipled by
    %the rotation before use.

     %Find the noise vectors that show the deviation of PTAM from the ground
     %truth Vicon.
     vslamNoise = struct;
     vslamNoise = setfield(vslamNoise,'time',vslamRaw.time);
     vslamNoise = setfield(vslamNoise,'tx',viconRe.tx-vslamTr.tx);
     vslamNoise = setfield(vslamNoise,'ty',viconRe.ty-vslamTr.ty);
     vslamNoise = setfield(vslamNoise,'tz',viconRe.tz-vslamTr.tz);

    if debug_on
        %Plot the noise vectors (the difference between the Vicon and
        %transformed PTAM
        h1 = figure('name','DEBUG: PTAM noise: Translation'); hold on;
        subplot(3,1,1);
        temp(1) = plot(vslamNoise.time,vslamNoise.tx);
        ylabel('X noise (m)');
        ylim([-0.2 0.2]);

        subplot(3,1,2);
        temp(2) = plot(vslamNoise.time,vslamNoise.ty);
        ylabel('Y noise (m)');
        ylim([-0.2 0.2]);

        subplot(3,1,3);
        temp(3) = plot(vslamNoise.time,vslamNoise.tz);
        ylabel('Z noise (m)');
        ylim([-0.2 0.2]);
        
        xlabel('Time (s)');

        %Clear unwanted junk
        clear temp maxYlim ii;
    end

    if plot_on
        %Plot the XY motion
        h1 = figure('name','PLOT: Position: XY'); hold on;
        plot(viconRe.ty,viconRe.tx,'-r');
        plot(vslamTr.ty,vslamTr.tx,'-g');
        xlabel('Y Pos. (m)');
        ylabel('X Pos. (m)');
        axis equal;
        legend('Vicon','PTAM');
    end
  
else
    %If either the Vicon or PTAM data file was empty
    error('This script requires both Vicon and PTAM data.');
    
end %viconPresent && vslamPresent

%At the end of this script there will exist the following variables
% viconRaw       - unaltered vicon data from file except times
% vslamRaw       - unaltered vslam data from file except times
% viconRe        - vicon data resampled into the vslam sample times
% vslamTr        - the vslam data transformed to best for the vicon data
% vslamNoise     - the difference between the each element in viconRe and vslamTr
% initTime       - the earliest time stamp in the vicon data
% tr.b,tr.T,tr.c - the scale, rotation, and translation mapping vslam to vicon
% fitQuality     - the quality of the Procrustes fit