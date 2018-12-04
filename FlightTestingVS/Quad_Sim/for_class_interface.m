close all
clear all
clc

% This file requires unpack.m

% Setup motion capture input from groundstation
n_vars=97*4;
socket=java.net.DatagramSocket(50000);
socket.setSoTimeout(2000);
socket.setReuseAddress(1);
packet = java.net.DatagramPacket(zeros(1,n_vars,'int8'),n_vars);

% Setup serial communication with Hummingbird (via XBee)
% You may need to check Device Manager and change which COM port number is
% assigned to the device.
try
    s = serial('COM8');
    set(s,'BaudRate',57600);
    fopen(s);
    msgid = typecast(uint16(11),'uint8');
    msgsz = typecast(uint16(20),'uint8');
catch err
    socket.close
    rethrow(err);
end

% Initialize things. Change maxcycles if you want to run only a few
% iterations. flag is used to stop MATLAB safely when you flip the
% Hummingbird from the High-Level Board to the Low-Level Board (incoming 
% messages come at 1Hz, not 50Hz). Please do not use Ctrl-C to stop MATLAB. 
% If you do so, you will need to close the UDP socket and the serial port
% manually (and failure to do so may result in having to restart MATLAB.

% These you might want to touch
maxcycles=1e7;
quad_id=2;      % This is the same number as on your XBee device

% These you don't
flag=0;
xhist = [];
uhist = [];
t1=0; t2=0; t3=0;

try
    for cycles=1:maxcycles
        
        tic
        % MEASUREMENT (You shouldn't have to touch this. x is the state
        % vector from MoCap: [x;y;z;vx;vy;vz;phi;theta;psi;p;q;r])
        socket.receive(packet);
        x=unpack(packet.getData,quad_id);
        t1=t1+toc;
        
        tic
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%% EDIT THIS CODE %%%%%%%%%%%%%%%%%%
        % FEEDFORWARD CONTROL (Control vector is [phi;theta;psi;thrust]
        u_ff=zeros(4,1);
        
        % FEEDBACK CONTROL
        u_fb=zeros(4,1);
        
        u=u_ff+u_fb
        %%%%%%%%%%% EDIT THAT CODE%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % STORE X AND U
        xhist = [xhist x];
        uhist = [uhist u];
        t2=t2+toc;

        % Package and send control over serial port. You shouldn't have to
        % touch this.
        tic
        crap=[msgid msgsz typecast(single(u(1)),'uint8') typecast(single(u(2)),'uint8') typecast(single(u(3)),'uint8') typecast(single(u(4)),'uint8')];
        fwrite(s,crap,'uint8');
        a=s.BytesAvailable;
        if a>=10
            [A,count]=fread(s,a-mod(a,10));
            if A(6)==1  % 1 signals the high level controller.
                if flag==0
                    flag=1;
                end
            else
                if flag==1
                    break
                end
            end
        end
        t3=t3+toc;
    end 
catch err
    socket.close
    fclose(s);
    delete(s)
    clear s
    rethrow(err);
end
socket.close
fclose(s);
delete(s)
clear s

% t1
% t2
% t3
% This rate estimate probably overestimates the true datarate, as ticking
% and tocking probably takes some non-zero time.
rate=cycles/(t1+t2+t3);
disp(['Datarate was approximately ',num2str(round(rate*10)/10),' Hz'])


% SAVE XHIST AND UHIST TO MATLAB DATA FILE
filename_t = now;
filename_tvec = datevec(now);
filename_tstr = datestr(filename_tvec, 'yyyymmdd-HHMMSS');
filename = strcat(filename_tstr,'-xhist.mat');
save(filename, 'xhist', 'uhist');






