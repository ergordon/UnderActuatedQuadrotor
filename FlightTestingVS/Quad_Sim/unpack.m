function varargout=unpack(msg,quad_id)

% This function unpacks the UDP message from MoCap. It requires two inputs:
% msg - the actual UDP packet
% quad_id - quadrotor identifier. This is the same number as the XBee
% identifier

% We return either one or two outputs. If you only take one output, it
% returns the quadrotor MoCap state. If you take a second output, it is a
% status message indicating if the MoCap frame was dropped. It is a zero if
% all is well, one if a frame is dropped.

% Key: [2;3;5;6] is [green;yellow;purple;white]
key=[2;3;5;6];

n=length(msg);
if n~=97*4
    error('Message length assumption violated in unpack')
end

start=97*(find(key==quad_id,1)-1);
state=zeros(12,1);
for i=1:12
    state(i)=typecast(msg(start+((i-1)*8+1:i*8)),'double');
end
varargout{1}=state;
if nargout==2;
    varargout{2}=msg(start+97);
end