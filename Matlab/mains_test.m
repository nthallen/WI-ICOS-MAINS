%%
cd C:\Users\nort\Documents\Documents\Exp\Boards\WI-ICOS-Mains\WI-ICOS-MAINS\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM9');
set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is an FCC
BdID = read_subbus(s, 2);
if BdID ~= 13
  error('Expected BdID 13 for WI-ICOS Mains. Reported %d', BdID);
end
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);
fprintf(1, 'Attached to WI-ICOS Mains S/N %d Build # %d\n', SerialNo, Build);
%%
rm_obj = read_multi_prep([8,40,9,0]);
[vals,ack] = read_multi(s,rm_obj);
even = mod(vals(2:end),256);
odd = floor(vals(2:end)/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
desc = char(il);
fprintf(1,'Description is: %s\n', desc);
%%
rm_obj = read_multi_prep([16,1,27]);
% FS = full scale voltage including the inverse of attached divider
% circuits.
FS = [ 4.096, 4.096, 4.096, 4.096, 4.096, 4.096, 6.144*104.99/4.99, 0.125, 4.096*11, 4.096*11 ]';
N = 0;
%%
stop = 0;
while stop == 0
  %%
  [vals,ack] = read_multi(s,rm_obj);
  %%
  fprintf(1,'---------\n');
  fprintf(1,'I2C Status: %04X  ADC Status: %04X  N: %8d\n', vals(1),vals(2),N);
  N = N+1;
  adc = vals(3:end);
  sadc = adc - (adc>2^15)*2^16;
  %%
  vadc = FS .* sadc / (2^15);
  for i=1:length(sadc)
    fprintf(1,'%04X %8d %10f V\n', adc(i), sadc(i), vadc(i));
  end
  %%
  pause(1);
end
%%
[value,ack] = read_subbus(s,39); % General read register
fprintf(1,'ack=%d value=%04X\n', ack,value);
