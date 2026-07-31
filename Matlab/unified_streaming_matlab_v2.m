function unified_streaming_matlab_v2()
% - Two Giga CVBS cameras (serial) + IMU (serial)
% - OptiTrack mocap (UDP unicast)
% - Displays both camera feeds, IMU orientation (bottom‑left),
%   and mocap orientation (bottom‑right). All data are recorded
%   and saved when streaming stops.


% ========================== CONFIGURATION ================================
% ---- Serial (Giga) ----
approx_fps = 60;                % camera frames/sec
record_duration = 300;          % max 5 minutes
baud_rate = 5000000;            % 5 Mbps
serial_timeout = 0.1;           % serial timeout duration

% for Mac
port_names = {'/dev/tty.usbmodem101', '/dev/tty.usbmodem2101'}; % use serialportlist("available") to find potential ports connected to Arduinos
save_folder = '/Users/**username**/Desktop'; % rename to your username

% e.g. for Windows
% port_names = {'COM3', 'COM4'};
% save_folder = 'C:\Users\**username**\Desktop';

cam1_min = 75;                  % figure black-level
cam1_max = 175;                 % figure white-level
cam1_crop_x = 20;
cam2_min = 75;  
cam2_max = 175;  
cam2_crop_x = 40;

% ---- NatNet (Mocap) ----
approx_datarate = 100;          % mocap packets/sec
default_server_ip = '192.168.0.106';
command_port = 1510;
listen_port  = 1511;
default_rb_id = 1;              % rigid body streaming ID
default_rb_name = 'ShrewSpecs';      % rigid body name
udp_timeout = 0.1;              % udp timeout duration

% ---- Visualization settings ----
live_view = true;
update_rate = 5;                % GUI plot updates/sec
images_to_bin = 1;


% ========================== FIXED SETTINGS ================================
% ---- Packet markers (from Giga firmware) ----
packet_start_marker = uint8(0xFE);
cam0_marker = uint8(0x01);
cam1_marker = uint8(0x02);
imu_marker = uint8(0x03);
settings_marker = uint8(0x04);
frame_marker = uint8(0x05);
text_marker = uint8(0x06);
cmd_status = uint8(0x20);
cmd_start_frame = uint8(0x21);
cmd_start_imu = uint8(0x22);
cmd_start_tracking = uint8(0x23);
stop_marker = uint8(0x11);

% image settings
crop_frame_w = 80;
crop_frame_h = 80;
frame_size = crop_frame_w * crop_frame_h;
last_display_time = 0;

% Pre‑allocate data
max_frames_per_cam = approx_fps * record_duration;
video_frames_cam1 = uint8(ones(crop_frame_h, crop_frame_w, max_frames_per_cam));
video_frames_cam2 = uint8(ones(crop_frame_h, crop_frame_w, max_frames_per_cam));
frame_samples_cam1 = nan(1, max_frames_per_cam);
frame_timestamps_cam1 = nan(1, max_frames_per_cam);
frame_readtimes_cam1 = nan(1, max_frames_per_cam);
frame_count_cam1 = 0;

frame_samples_cam2 = nan(1, max_frames_per_cam);
frame_timestamps_cam2 = nan(1, max_frames_per_cam);
frame_readtimes_cam2 = nan(1, max_frames_per_cam);
frame_count_cam2 = 0;

imu_max_count = max_frames_per_cam;
imu_samples = nan(1, imu_max_count);
imu_timestamps = nan(1, imu_max_count);
imu_readtimes = nan(1, imu_max_count);
imu_count = 0;
q_w = nan(1, imu_max_count);
q_x = nan(1, imu_max_count);
q_y = nan(1, imu_max_count);
q_z = nan(1, imu_max_count);
linacc_x = nan(1, imu_max_count);
linacc_y = nan(1, imu_max_count);
linacc_z = nan(1, imu_max_count);

% Pre‑allocate Mocap data
mocap_max_samples = approx_datarate * record_duration; 
mocap_q = nan(mocap_max_samples, 4);      % [qx qy qz qw]
mocap_pos = nan(mocap_max_samples, 3);    % [x y z]
mocap_frame_num = nan(1, mocap_max_samples);
mocap_is_recording = nan(1, mocap_max_samples);
mocap_timestamps = nan(1, mocap_max_samples);
mocap_tracking_error = nan(1, mocap_max_samples);
mocap_valid = false(1, mocap_max_samples);
mocap_readtime = nan(1, mocap_max_samples);
mocap_count = 0;

% Streaming control
streaming_active = false;
streaming_start_time = 0;
exit_flag = false;
data_saved_for_session = false;
prev_streaming_active = false;
stream_tic = [];          % for elapsed time

% UDP / NatNet state
udp_obj = [];
mocap_subscribed = false;
last_keepalive_time = 0;

% ========================== BUILD GUI ====================================
font_size_1 = 9;
font_size_2 = 11;

fig = figure('Position', [50 50 1200 1000], ...
    'Name', 'GigaCVBS + NatNet Viewer', ...
    'NumberTitle', 'off', ...
    'CloseRequestFcn', @(src,evt) onClose());

% ---- Subplots ----
% Top‑Left: Cam1
axes('OuterPosition', [0 0.6 0.4 0.4]);
i1 = imshow(zeros(500, 500));
t1_a = text(0, 0, ' ', 'HorizontalAlignment','left','VerticalAlignment','top','Color','w');
t1_b = text(500, 0, ' ', 'HorizontalAlignment','right','VerticalAlignment','top','Color','w');
title('Cam1', 'FontSize', font_size_2);

% Top‑Right: Cam2
axes('OuterPosition', [0.4 0.6 0.4 0.4]);
i2 = imshow(zeros(500, 500));
t2_a = text(0, 0, ' ', 'HorizontalAlignment','left','VerticalAlignment','top','Color','w');
t2_b = text(500, 0, ' ', 'HorizontalAlignment','right','VerticalAlignment','top','Color','w');
title('Cam2', 'FontSize', font_size_2);

% Bottom‑Left: IMU Orientation
ax_IMU = axes('OuterPosition', [0 0.2 0.4 0.4]);
hold(ax_IMU, 'on');
IMU_x = quiver3(0,0,0,0,0,0, 'r', 'LineWidth', 2, 'DisplayName', 'forward');
IMU_y = quiver3(0,0,0,0,0,0, 'g', 'LineWidth', 1, 'DisplayName', 'left');
IMU_z = quiver3(0,0,0,0,0,0, 'b', 'LineWidth', 1, 'DisplayName', 'up');
legend(ax_IMU, 'Location', 'best');
title('IMU Orientation', 'FontSize', font_size_2);
xlabel('east'); ylabel('north'); zlabel('vertical');
grid on; axis equal; view(3);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
hold(ax_IMU, 'off');

% Bottom‑Right: Mocap Orientation
ax_Mocap = axes('OuterPosition', [0.4 0.2 0.4 0.4]);
hold(ax_Mocap, 'on');
Mocap_x = quiver3(0,0,0,0,0,0, 'r', 'LineWidth', 2, 'DisplayName', 'X');
Mocap_y = quiver3(0,0,0,0,0,0, 'g', 'LineWidth', 1, 'DisplayName', 'Y');
Mocap_z = quiver3(0,0,0,0,0,0, 'b', 'LineWidth', 1, 'DisplayName', 'Z');
legend(ax_Mocap, 'Location', 'best');
title('Mocap Orientation', 'FontSize', font_size_2);
xlabel('east'); ylabel('north'); zlabel('vertical');
grid on; axis equal; view(3);
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
hold(ax_Mocap, 'off');

% ---- Right Panel Controls (re‑arranged) ----
% ---- NatNet fields
uicontrol('Style','text','Units','normalized','Position',[0.82 0.97 0.15 0.02],...
    'String','NatNet','HorizontalAlignment','left','FontSize',font_size_2);

uicontrol('Style','text','Units','normalized','Position',[0.82 0.94 0.15 0.02],...
    'String','Server IP:','HorizontalAlignment','left','FontSize',font_size_1);
hServerIP = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.91 0.15 0.03],...
    'String',default_server_ip,'FontSize',font_size_1);

uicontrol('Style','text','Units','normalized','Position',[0.82 0.88 0.15 0.02],...
    'String','RB Name:','HorizontalAlignment','left','FontSize',font_size_1);
hRBName = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.85 0.15 0.03],...
    'String',default_rb_name,'FontSize',font_size_1);

uicontrol('Style','text','Units','normalized','Position',[0.82 0.82 0.15 0.02],...
    'String','RB ID:','HorizontalAlignment','left','FontSize',font_size_1);
hRBID = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.79 0.15 0.03],...
    'String',num2str(default_rb_id),'FontSize',font_size_1);


% ---- Camera fields
uicontrol('Style','text','Units','normalized','Position',[0.82 0.73 0.15 0.02],...
    'String','Cameras','HorizontalAlignment','left','FontSize',font_size_2);

% Brightness scaling
uicontrol('Style','text','Units','normalized','Position',[0.82 0.70 0.15 0.02],...
    'String','Cam1 Min/Max','HorizontalAlignment','left','FontSize',font_size_1);
edit_c1_min = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.67 0.07 0.03],...
    'String',num2str(cam1_min));
edit_c1_max = uicontrol('Style','edit','Units','normalized','Position',[0.90 0.67 0.07 0.03],...
    'String',num2str(cam1_max));

uicontrol('Style','text','Units','normalized','Position',[0.82 0.64 0.15 0.02],...
    'String','Cam2 Min/Max','HorizontalAlignment','left','FontSize',font_size_1);
edit_c2_min = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.61 0.07 0.03],...
    'String',num2str(cam2_min));
edit_c2_max = uicontrol('Style','edit','Units','normalized','Position',[0.90 0.61 0.07 0.03],...
    'String',num2str(cam2_max));

% Camera crop settings
uicontrol('Style','text','Units','normalized','Position',[0.82 0.58 0.15 0.02],...
    'String','Cam1 Crop X/Y','HorizontalAlignment','left','FontSize',font_size_1);
edit_c1_cx = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.55 0.07 0.03],...
    'String',num2str(cam1_crop_x));
edit_c1_cy = uicontrol('Style','edit','Units','normalized','Position',[0.90 0.55 0.07 0.03],...
    'String','0');

uicontrol('Style','text','Units','normalized','Position',[0.82 0.52 0.15 0.02],...
    'String','Cam2 Crop X/Y','HorizontalAlignment','left','FontSize',font_size_1);
edit_c2_cx = uicontrol('Style','edit','Units','normalized','Position',[0.82 0.49 0.07 0.03],...
    'String',num2str(cam2_crop_x));
edit_c2_cy = uicontrol('Style','edit','Units','normalized','Position',[0.90 0.49 0.07 0.03],...
    'String','0');

btn_send_crop = uicontrol('Style','pushbutton','Units','normalized','Position',[0.82 0.46 0.15 0.02],...
    'String','Send Crop Settings','FontSize',font_size_1,...
    'Callback', @(src,evt) sendCropCallback(edit_c1_cx, edit_c1_cy, edit_c2_cx, edit_c2_cy));


% ---- Streaming fields
uicontrol('Style','text','Units','normalized','Position',[0.82 0.40 0.15 0.02],...
    'String','Streaming','HorizontalAlignment','left','FontSize',font_size_2);

% Streaming checkboxes
chk_mocap = uicontrol('Style','checkbox','Units','normalized','Position',[0.82 0.37 0.15 0.03],...
    'String','Stream Mocap','Value',1,'BackgroundColor',[0.9 0.9 0.9]);
chk_frame = uicontrol('Style','checkbox','Units','normalized','Position',[0.82 0.35 0.15 0.03],...
    'String','Stream Video','Value',1,'BackgroundColor',[0.9 0.9 0.9]);
chk_imu = uicontrol('Style','checkbox','Units','normalized','Position',[0.82 0.33 0.15 0.03],...
    'String','Stream IMU','Value',1,'BackgroundColor',[0.9 0.9 0.9]);
chk_tracking = uicontrol('Style','checkbox','Units','normalized','Position',[0.82 0.31 0.15 0.03],...
    'String','Stream Eye Tracking (TBD)','Value',0,'BackgroundColor',[0.9 0.9 0.9]);

% Start/Stop and Exit
btn_streaming = uicontrol('Style','pushbutton','Units','normalized','Position',[0.82 0.27 0.15 0.04],...
    'String','Start Streaming','FontSize',font_size_2,'FontWeight','bold','BackgroundColor',[0.2 0.8 0.2],...
    'Callback', @(src,evt) toggleStreamingCallback(src, chk_frame, chk_imu, chk_tracking, chk_mocap));

btn_exit = uicontrol('Style','pushbutton','Units','normalized','Position',[0.82 0.21 0.15 0.04],...
    'String','Exit','FontSize',font_size_2,'FontWeight','bold','BackgroundColor',[0.6 0.6 0.6],...
    'Callback', @(src,evt) exitScriptCallback());



% ---- Status windows (MATLAB and Boards)
uicontrol('Style','text','Units','normalized','Position',[0 0.185 0.49 0.015],...
    'String','MATLAB Status','FontWeight','bold','BackgroundColor',[0.9 0.9 0.9]);

status_messages = {'Matlab Status window initialized.'};
status_listbox = uicontrol('Style','listbox','Units','normalized','Position',[0 0 0.49 0.18],...
    'String',status_messages,'FontSize',9,'BackgroundColor','w','Max',2,'Value',[]);

uicontrol('Style','text','Units','normalized','Position',[0.51 0.185 0.49 0.015],...
    'String','Peripheral Status','FontWeight','bold','BackgroundColor',[0.9 0.9 0.9]);
message_log = {'Serial monitor initialized.'};
serial_monitor = uicontrol('Style','listbox','Units','normalized','Position',[0.51 0 0.49 0.18],...
    'String',message_log,'FontSize',9,'BackgroundColor','w','Max',2,'Value',[]);

drawnow;


% ========================== INITIALISE SERIAL PORTS =====================
serial_ports = cell(1, numel(port_names));
ports_opened = false;
for idx = 1:numel(port_names)
    try
        serial_ports{idx} = serialport(port_names{idx}, baud_rate, 'Timeout', serial_timeout);
        addStatusMessage(['Opened ' port_names{idx}]);
        ports_opened = true;
    catch ME
        warning('Could not open %s: %s', port_names{idx}, ME.message);
        addStatusMessage(sprintf('Could not open %s: %s', port_names{idx}, ME.message));
        serial_ports{idx} = [];
    end
end

if ~ports_opened
    addStatusMessage('No serial ports could be opened. Check port names and connections');
end

% Flush initial bytes
for idx = 1:numel(serial_ports)
    if ~isempty(serial_ports{idx})
        while serial_ports{idx}.NumBytesAvailable > 0
            read(serial_ports{idx}, serial_ports{idx}.NumBytesAvailable, 'uint8');
        end
    end
end

% Send status request to each board
addStatusMessage('Requesting status updates from serial ports for Giga CVBS boards');
for idx = 1:numel(serial_ports)
    if ~isempty(serial_ports{idx})
        write(serial_ports{idx}, cmd_status, 'uint8');
    end
    pause(0.25);
end

serial_buffers = cell(1, numel(serial_ports));
for idx = 1:numel(serial_ports)
    serial_buffers{idx} = uint8([]);
end

% ========================== INITIALISE UDP (NATNET) =====================
try
    udp_obj = udpport('datagram','IPV4', ...
                      'LocalHost','0.0.0.0', ...
                      'LocalPort', listen_port, ...
                      'EnablePortSharing', false, ...
                      'Timeout', udp_timeout);
    addStatusMessage('UDP port opened for NatNet');
catch ME
    addStatusMessage(sprintf('UDP open failed: %s', ME.message));
    udp_obj = [];
end




% ========================== ASSIGN VARIABLES TO WORKSPACE =====================
assignin('base','serial_ports',serial_ports);
assignin('base','serial_buffers',serial_buffers);
assignin('base','udp_obj',udp_obj);
assignin('base','mocap_subscribed',false);
assignin('base','last_keepalive_time',0);
assignin('base','streaming_active',false);
assignin('base','streaming_start_time',0);
assignin('base','exit_flag',false);
assignin('base','data_saved_for_session',false);
assignin('base','prev_streaming_active',false);
assignin('base','stream_tic',[]);
assignin('base','mocap_count',0);
assignin('base','mocap_q',mocap_q);
assignin('base','mocap_pos',mocap_pos);
assignin('base','mocap_frame_num',mocap_frame_num);
assignin('base','mocap_is_recording',mocap_is_recording);
assignin('base','mocap_timestamps',mocap_timestamps);
assignin('base','mocap_tracking_error',mocap_tracking_error);
assignin('base','mocap_valid',mocap_valid);
assignin('base','mocap_readtime',mocap_readtime);
% Also serial data arrays (already in base via script)
assignin('base','video_frames_cam1',video_frames_cam1);
assignin('base','frame_samples_cam1',frame_samples_cam1);
assignin('base','frame_timestamps_cam1',frame_timestamps_cam1);
assignin('base','frame_readtimes_cam1',frame_readtimes_cam1);
assignin('base','frame_count_cam1',frame_count_cam1);
assignin('base','video_frames_cam2',video_frames_cam2);
assignin('base','frame_samples_cam2',frame_samples_cam2);
assignin('base','frame_timestamps_cam2',frame_timestamps_cam2);
assignin('base','frame_readtimes_cam2',frame_readtimes_cam2);
assignin('base','frame_count_cam2',frame_count_cam2);
assignin('base','imu_samples',imu_samples);
assignin('base','imu_timestamps',imu_timestamps);
assignin('base','imu_readtimes',imu_readtimes);
assignin('base','imu_count',imu_count);
assignin('base','q_w',q_w);
assignin('base','q_x',q_x);
assignin('base','q_y',q_y);
assignin('base','q_z',q_z);
assignin('base','linacc_x',linacc_x);
assignin('base','linacc_y',linacc_y);
assignin('base','linacc_z',linacc_z);

% Store figure handles for callbacks
handles = struct('fig',fig, ...
    'i1',i1,'t1_a',t1_a,'t1_b',t1_b, ...
    'i2',i2,'t2_a',t2_a,'t2_b',t2_b, ...
    'ax_IMU',ax_IMU,'IMU_forward',IMU_x,'IMU_left',IMU_y,'IMU_up',IMU_z, ...
    'ax_Mocap',ax_Mocap,'hMocapX',Mocap_x,'hMocapY',Mocap_y,'hMocapZ',Mocap_z, ...
    'hServerIP',hServerIP,'hRBName',hRBName,'hRBID',hRBID, ...
    'chk_mocap',chk_mocap,'chk_frame',chk_frame,'chk_imu',chk_imu,'chk_tracking',chk_tracking, ...
    'btn_streaming',btn_streaming,'btn_exit',btn_exit, ...
    'edit_c1_min',edit_c1_min,'edit_c1_max',edit_c1_max, ...
    'edit_c2_min',edit_c2_min,'edit_c2_max',edit_c2_max, ...
    'edit_c1_cx',edit_c1_cx,'edit_c1_cy',edit_c1_cy, ...
    'edit_c2_cx',edit_c2_cx,'edit_c2_cy',edit_c2_cy, ...
    'btn_send_crop',btn_send_crop, ...
    'status_listbox',status_listbox,'serial_monitor',serial_monitor);
assignin('base','handles',handles);


% ========================== MAIN LOOP ===================================
addStatusMessage('Main loop started. Press Start Streaming to begin.');

while true
    % Check exit flag
    if exit_flag
        addStatusMessage('Exiting script...');
        break;
    end

    elapsed = [];
    if ~isempty(stream_tic)
        elapsed = toc(stream_tic);
    end

    % Retrieve current streaming state
    if streaming_active && ~prev_streaming_active
        last_display_time = 0;   % reset display timer on new stream
    end
    prev_streaming_active = streaming_active;

    % Check recording duration
    if streaming_active
        if ~isempty(elapsed) && elapsed >= record_duration
            addStatusMessage('Record duration reached. Stopping streaming and saving data...');
            % Send stop commands to serial ports
            for idx = 1:numel(serial_ports)
                if ~isempty(serial_ports{idx})
                    try
                        write(serial_ports{idx}, stop_marker, 'uint8');
                    catch
                    end
                end
            end
            % Also unsubscribe mocap if active
            if mocap_subscribed
                doMocapUnsubscribe();
            end
            streaming_active = false;
            set(btn_streaming,'String','Start Streaming','BackgroundColor',[0.2 0.8 0.2]);
            data_saved_for_session = false;  % allow save
        end
    end

    % ---- Read serial data ----
    for idx = 1:numel(serial_ports)
        s = serial_ports{idx};
        if isempty(s)
            continue;
        end
        if s.NumBytesAvailable > 0
            new_bytes = uint8(read(s, s.NumBytesAvailable, 'uint8'));
            serial_buffers{idx} = [serial_buffers{idx}; new_bytes(:)];
        end

        buf = serial_buffers{idx};
        while ~isempty(buf)
            if buf(1) ~= packet_start_marker
                buf(1) = [];
                continue;
            end
            if numel(buf) < 4
                break;
            end
            packet_type = buf(2);
            payload_len = double(buf(3)) + 256*double(buf(4));
            packet_len = 1+1+2+payload_len+1;
            if numel(buf) < packet_len
                break;
            end
            packet = buf(1:packet_len);
            buf(1:packet_len) = [];

            % Verify checksum
            checksum = packet(end);
            calc = double(packet(2)) + double(packet(3)) + double(packet(4));
            if payload_len > 0
                calc = calc + double(sum(uint8(packet(5:end-1))));
            end
            if mod(calc,256) ~= double(checksum)
                continue;   % bad packet, drop
            end

            payload = packet(5:end-1);
            % Process frame packets
            if packet_type == frame_marker
                cam_id = payload(1);
                frame_num = typecast(reshape(payload(2:3)',[],1), 'uint16');
                timestamp = typecast(reshape(payload(4:7)',[],1), 'uint32');
                image_data = payload(8:end);
                frame_size = reshape(image_data, [crop_frame_w, crop_frame_h])';

                if cam_id == cam0_marker
                    fc1 = frame_count_cam1;
                    if fc1 < max_frames_per_cam
                        fc1 = fc1 + 1;
                        frame_readtimes_cam1(fc1) = elapsed;
                        video_frames_cam1(:,:,fc1) = frame_size;
                        frame_samples_cam1(fc1) = frame_num;
                        frame_timestamps_cam1(fc1) = double(timestamp)/1000;
                        frame_count_cam1 = fc1;
                    elseif streaming_active
                        streaming_active = false;
                        set(btn_streaming,'String','Start Streaming','BackgroundColor',[0.2 0.8 0.2]);
                        addStatusMessage('Camera 1 buffer full. Stopping streaming.');
                    end
                elseif cam_id == cam1_marker
                    fc2 = frame_count_cam2;
                    if fc2 < max_frames_per_cam
                        fc2 = fc2 + 1;
                        frame_readtimes_cam2(fc2) = elapsed;
                        video_frames_cam2(:,:,fc2) = frame_size;
                        frame_samples_cam2(fc2) = frame_num;
                        frame_timestamps_cam2(fc2) = double(timestamp)/1000;
                        frame_count_cam2 = fc2;
                    elseif streaming_active
                        streaming_active = false;
                        set(btn_streaming,'String','Start Streaming','BackgroundColor',[0.2 0.8 0.2]);
                        addStatusMessage('Camera 2 buffer full. Stopping streaming.');
                    end
                end

            elseif packet_type == imu_marker
                sample_num = typecast(reshape(payload(1:2)',[],1), 'uint16');
                timestamp = typecast(reshape(payload(3:6)',[],1), 'uint32');
                imu_data = typecast(reshape(payload(7:end)',[],1), 'single');
                ic = imu_count;
                if ic < imu_max_count
                    ic = ic + 1;
                    imu_samples(ic) = sample_num;
                    imu_timestamps(ic) = double(timestamp)/1000;
                    imu_readtimes(ic) = elapsed;
                    q_w(ic) = imu_data(1);
                    q_x(ic) = imu_data(2);
                    q_y(ic) = imu_data(3);
                    q_z(ic) = imu_data(4);
                    linacc_x(ic) = imu_data(5);
                    linacc_y(ic) = imu_data(6);
                    linacc_z(ic) = imu_data(7);
                    imu_count = ic;
                end

            elseif packet_type == text_marker
                text_msg = char(payload');
                text_msg = strtrim(text_msg);
                full_msg = sprintf('%s: %s', port_names{idx}, text_msg);
                message_log = [message_log; {full_msg}];
                if numel(message_log) > 10
                    message_log = message_log(end-9:end);
                end
                set(serial_monitor,'String',message_log,'Value',[]);
                drawnow;
            end
        end
        serial_buffers{idx} = buf;
    end

    % ---- Read UDP (NatNet) ----
    if ~isempty(udp_obj) && isvalid(udp_obj) && streaming_active
        % Drain all datagrams
        while udp_obj.NumDatagramsAvailable > 0
            dg = read(udp_obj, 1, 'uint8');
            data = dg.Data;
            if numel(data) < 2
                continue;
            end
            msgID = data(1) + data(2)*256;
            if msgID ~= 7
                continue;   % skip non‑frame packets
            end
            % Parse frame
            rbID = str2double(get(hRBID,'String'));
            [q, pos, trackingErr, tsMs, frameNumber, isRecording, valid, ok] = parseFrame(data, rbID);

            if ok
                mc = mocap_count;
                if mc < mocap_max_samples
                    mc = mc + 1;
                    mocap_readtime(mc) = elapsed;
                    mocap_q(mc,:) = q;
                    mocap_pos(mc,:) = pos;
                    mocap_frame_num(mc) = frameNumber;
                    mocap_timestamps(mc) = tsMs;
                    mocap_tracking_error(mc) = trackingErr;
                    mocap_is_recording(mc) = isRecording;
                    mocap_valid(mc) = valid;
                    mocap_count = mc;
                end
            end
        end

        % Keepalive
        if mocap_subscribed
            if isempty(last_keepalive_time)
                last_keepalive_time = 0;
            end
            if elapsed - last_keepalive_time > 4
                serverIP = get(hServerIP,'String');
                sendKeepalive(udp_obj, serverIP, command_port);
                last_keepalive_time = elapsed;
            end
        end
    end

    % ---- Live display update ----
    if live_view && ~isempty(elapsed)
        if elapsed - last_display_time >= 1/update_rate
            last_display_time = elapsed;

            % Cam1
            fc1 = frame_count_cam1;
            if fc1 > 0
                if fc1 >= images_to_bin
                    I = mean(double(video_frames_cam1(:,:,fc1-images_to_bin+1:fc1)), 3, 'omitnan');
                else
                    I = double(video_frames_cam1(:,:,fc1));
                end
                c1_min = str2double(get(edit_c1_min,'String'));
                c1_max = str2double(get(edit_c1_max,'String'));
                I = (I - c1_min) ./ (c1_max - c1_min);
                I = max(0, min(1, I));
                I = imresize(I, [500 500], 'bilinear');
                set(i1, 'CData', I);
                set(t1_a, 'String', sprintf('cam1 frame %d', fc1));
                set(t1_b, 'String', sprintf('%.2f s', frame_timestamps_cam1(fc1)));
            end

            % Cam2
            fc2 = frame_count_cam2;
            if fc2 > 0
                if fc2 >= images_to_bin
                    I = mean(double(video_frames_cam2(:,:,fc2-images_to_bin+1:fc2)), 3, 'omitnan');
                else
                    I = double(video_frames_cam2(:,:,fc2));
                end
                c2_min = str2double(get(edit_c2_min,'String'));
                c2_max = str2double(get(edit_c2_max,'String'));
                I = (I - c2_min) ./ (c2_max - c2_min);
                I = max(0, min(1, I));
                I = imresize(I, [500 500], 'bilinear');
                set(i2, 'CData', I);
                set(t2_a, 'String', sprintf('cam2 frame %d', fc2));
                set(t2_b, 'String', sprintf('%.2f s', frame_timestamps_cam2(fc2)));
            end

            % IMU orientation (bottom‑left)
            ic = imu_count;
            if ic > 0
                qw = q_w(ic); qxv = q_x(ic); qyv = q_y(ic); qzv = q_z(ic);
                R = quat2rot([qxv qyv qzv qw]);   % [qx qy qz qw]
                main_vec = R * [1;0;0];
                side_vec1 = R * [0;1;0] * 0.5;
                side_vec2 = R * [0;0;1] * 0.5;
                set(IMU_x, 'UData', main_vec(1), 'VData', main_vec(2), 'WData', main_vec(3));
                set(IMU_y, 'UData', side_vec1(1), 'VData', side_vec1(2), 'WData', side_vec1(3));
                set(IMU_z, 'UData', side_vec2(1), 'VData', side_vec2(2), 'WData', side_vec2(3));
            end

            % Mocap orientation (bottom‑right)
            mc = mocap_count;
            if mc > 0
                qm = mocap_q(mc,:);   % [qx qy qz qw]
                Rm = quat2rot(qm);
                % Use the same axis convention as NatNet viewer: X (red), Y (green), Z (blue)
                set(Mocap_x, 'UData', Rm(1,1), 'VData', Rm(2,1), 'WData', Rm(3,1));
                set(Mocap_y, 'UData', Rm(1,2), 'VData', Rm(2,2), 'WData', Rm(3,2));
                set(Mocap_z, 'UData', Rm(1,3), 'VData', Rm(2,3), 'WData', Rm(3,3));
            end

            drawnow limitrate;
        end
    end

    % Check if streaming stopped and data needs saving
    if ~streaming_active && ~data_saved_for_session
        fc1 = frame_count_cam1;
        fc2 = frame_count_cam2;
        ic = imu_count;
        mc = mocap_count;
        if fc1>0 || fc2>0 || ic>0 || mc>0
            saveRecordedData();
            data_saved_for_session = true;
        end
    end
    if streaming_active && data_saved_for_session
        data_saved_for_session = false;
    end

    pause(0.001);
end


% ========================== CLEANUP ======================================
% Close serial ports
for idx = 1:numel(serial_ports)
    if ~isempty(serial_ports{idx})
        clear serial_ports{idx};
    end
end

% Unsubscribe and close UDP
if mocap_subscribed
    doMocapUnsubscribe();
end
if ~isempty(udp_obj) && isvalid(udp_obj)
    delete(udp_obj);
end

if isvalid(fig)
    close(fig);
end
disp('Script ended successfully.');


% ==================== NESTED CALLBACK FUNCTIONS =========================
    function toggleStreamingCallback(src, h_frame, h_imu, h_trk, h_mocap)
        if streaming_active
            % ---- STOP ----
            for idx = 1:numel(serial_ports)
                if ~isempty(serial_ports{idx})
                    try
                        write(serial_ports{idx}, stop_marker, 'uint8');
                    catch ME
                        addStatusMessage(sprintf('Failed to send stop to port %d: %s', idx, ME.message));
                    end
                end
            end
            if mocap_subscribed
                doMocapUnsubscribe();
            end
            streaming_active = false;
            set(src,'String','Start Streaming','BackgroundColor',[0.2 0.8 0.2]);
        else
            % ---- START ----
            do_frame = get(h_frame,'Value');
            do_imu = get(h_imu,'Value');
            do_trk = get(h_trk,'Value');
            do_mocap = get(h_mocap,'Value');

            % Reset data arrays (serial)
            frame_count_cam1 = 0;
            frame_count_cam2 = 0;
            imu_count = 0;
            mocap_count = 0;
            stream_tic = tic;
            streaming_start_time = datetime('now','Format','yy_MM_dd_HH_mm_SS');
            data_saved_for_session = false;

            % Subscribe to mocap if requested
            if do_mocap
                if ~isempty(udp_obj) && isvalid(udp_obj)
                    serverIP = get(hServerIP,'String');
                    rbName = get(hRBName,'String');
                    sendConnect(udp_obj, serverIP, command_port);
                    pause(0.1);
                    addStatusMessage('Sent NatNet connection request');
                    sendSubscribe(udp_obj, serverIP, command_port, rbName);
                    mocap_subscribed = true;
                    last_keepalive_time = 0;
                    addStatusMessage('Subscribed to mocap data');
                else
                    addStatusMessage('UDP not available – cannot subscribe to mocap');
                end
            end

            % Send start commands to serial boards
            for idx = 1:numel(serial_ports)
                if ~isempty(serial_ports{idx})
                    try
                        if do_frame
                            write(serial_ports{idx}, cmd_start_frame, 'uint8');
                        end
                        if do_imu
                            write(serial_ports{idx}, cmd_start_imu, 'uint8');
                        end
                        if do_trk
                            write(serial_ports{idx}, cmd_start_tracking, 'uint8');
                        end
                    catch ME
                        addStatusMessage(sprintf('Failed to send start to port %d: %s', idx, ME.message));
                    end
                end
            end

            streaming_active = true;
            set(src,'String','Stop Streaming','BackgroundColor',[0.8 0.2 0.2]);
        end
    end

    function exitScriptCallback()
        exit_flag = true;
        addStatusMessage('Exit requested...');
    end

    function sendCropCallback(h_c1x, h_c1y, h_c2x, h_c2y)
        c1x = uint8(str2double(get(h_c1x,'String')));
        c1y = uint8(str2double(get(h_c1y,'String')));
        c2x = uint8(str2double(get(h_c2x,'String')));
        c2y = uint8(str2double(get(h_c2y,'String')));
        if numel(serial_ports)>=1 && ~isempty(serial_ports{1})
            try
                write(serial_ports{1}, [settings_marker, c1x, c1y], 'uint8');
                addStatusMessage(sprintf('Sent crop to port 1: X=%d Y=%d', c1x, c1y));
            catch ME
                addStatusMessage(sprintf('Failed sending crop to port 1: %s', ME.message));
            end
        end
        if numel(serial_ports)>=2 && ~isempty(serial_ports{2})
            try
                write(serial_ports{2}, [settings_marker, c2x, c2y], 'uint8');
                addStatusMessage(sprintf('Sent crop to port 2: X=%d Y=%d', c2x, c2y));
            catch ME
                addStatusMessage(sprintf('Failed sending crop to port 2: %s', ME.message));
            end
        end
    end

    function doMocapUnsubscribe()
        if ~isempty(udp_obj) && isvalid(udp_obj)
            serverIP = get(hServerIP,'String');
            rbName = get(hRBName,'String');
            sendUnsubscribe(udp_obj, serverIP, command_port, rbName);
            addStatusMessage('Unsubscribed from mocap');
        end
        mocap_subscribed = false;
    end

    function onClose()
        exit_flag = true;
        drawnow;
        % Give main loop a moment to exit
        pause(0.5);
        % If figure is still open, force close it
        if isvalid(fig)
            delete(fig);
        end
    end

    function addStatusMessage(msg)
        msgs = [status_messages; {msg}];
        if numel(msgs) > 10
            msgs = msgs(end-9:end);
        end
        status_messages = msgs;
        set(status_listbox,'String',msgs,'Value',[]);
        drawnow;
    end

    function saveRecordedData()
        % Gather all data from local workspace
        fc1 = frame_count_cam1;
        fc2 = frame_count_cam2;
        ic = imu_count;
        mc = mocap_count;
        if fc1==0 && fc2==0 && ic==0 && mc==0
            addStatusMessage('No data to save.');
            return;
        end

        % Trim pre‑allocated arrays
        results.frame_samples_cam1 = frame_samples_cam1(1:fc1);
        results.frame_timestamps_cam1 = frame_timestamps_cam1(1:fc1);
        results.frame_readtimes_cam1 = frame_readtimes_cam1(1:fc1);
        results.frame_count_cam1 = fc1;
        results.video_frames_cam1 = video_frames_cam1(:,:,1:fc1);

        results.frame_samples_cam2 = frame_samples_cam2(1:fc2);
        results.frame_timestamps_cam2 = frame_timestamps_cam2(1:fc2);
        results.frame_readtimes_cam2 = frame_readtimes_cam2(1:fc2);
        results.frame_count_cam2 = fc2;
        results.video_frames_cam2 = video_frames_cam2(:,:,1:fc2);

        results.imu_samples = imu_samples(1:ic);
        results.imu_timestamps = imu_timestamps(1:ic);
        results.imu_readtimes = imu_readtimes(1:ic);
        results.imu_count = ic;
        results.q_w = q_w(1:ic);
        results.q_x = q_x(1:ic);
        results.q_y = q_y(1:ic);
        results.q_z = q_z(1:ic);
        results.linacc_x = linacc_x(1:ic);
        results.linacc_y = linacc_y(1:ic);
        results.linacc_z = linacc_z(1:ic);

        results.mocap_q = mocap_q(1:mc,:);
        results.mocap_pos = mocap_pos(1:mc,:);
        results.mocap_frame_num = mocap_frame_num(1:mc);
        results.mocap_timestamps = mocap_timestamps(1:mc);
        results.mocap_is_recording = mocap_is_recording(1:mc);
        results.mocap_tracking_error = mocap_tracking_error(1:mc);
        results.mocap_valid = mocap_valid(1:mc);
        results.mocap_readtime = mocap_readtime(1:mc);
        results.mocap_count = mc;

        filename = ['GigaCVBS_NatNet_' char(streaming_start_time) '.mat'];
        save(fullfile(save_folder, filename), 'results');
        addStatusMessage(sprintf('Data saved to %s\nCam1: %d frames, Cam2: %d frames, IMU: %d samples, Mocap: %d samples', ...
            filename, fc1, fc2, ic, mc));
    end

end % end main function


% ==================== NATNET HELPER FUNCTIONS ============================
function sendKeepalive(u, serverIP, cmdPort)
    pkt = zeros(1,4,'uint8');
    pkt(1)=10; pkt(2)=0; pkt(3)=0; pkt(4)=0;
    write(u, pkt, 'uint8', serverIP, cmdPort);
end

function sendConnect(u, serverIP, cmdPort)
    payload = zeros(1,270,'uint8');
    payload(1:4) = uint8('Ping');
    payload(265:268) = [4 5 0 0];   % version 4.5.0.0
    nBytes = 271;   % 270 payload + null terminator
    pkt = zeros(1, 2+2+270+1, 'uint8');
    pkt(1:2) = [0 0];               % msgID = 0
    pkt(3:4) = [bitand(nBytes,255), bitshift(nBytes,-8)];
    pkt(5:274) = payload;
    pkt(275) = 0;
    write(u, pkt, 'uint8', serverIP, cmdPort);
    fprintf('NatNet: sent NAT_CONNECT to %s:%d\n', serverIP, cmdPort);
end

function sendSubscribe(u, serverIP, cmdPort, rbName)
    cmd = ['SubscribeToData,RigidBody,' rbName];
    sendNatRequest(u, serverIP, cmdPort, cmd);
    %fprintf('NatNet: sent subscribe: %s\n', cmd);
end

function sendUnsubscribe(u, serverIP, cmdPort, rbName)
    cmd = ['SubscribeToData,RigidBody,' rbName ',None'];
    sendNatRequest(u, serverIP, cmdPort, cmd);
    %fprintf('NatNet: sent unsubscribe: %s\n', cmd);
end

function sendNatRequest(u, serverIP, cmdPort, cmdStr)
    cmdBytes = [uint8(cmdStr) 0];
    nBytes = numel(cmdBytes);
    pkt = zeros(1, 4+nBytes, 'uint8');
    pkt(1:2) = [2 0];
    pkt(3:4) = [bitand(nBytes,255), bitshift(nBytes,-8)];
    pkt(5:end) = cmdBytes;
    write(u, pkt, 'uint8', serverIP, cmdPort);
end


function [q, pos, err, tsMs, frameNumber, isRecording, valid, ok] = parseFrame(data, targetID)
% Parse a NAT_FRAMEOFDATA (msgID 7) packet: rigid body + trailing timing suffix.

    q   = [0 0 0 1];
    pos = [0 0 0];
    err = 0;
    tsMs = NaN;
    timecodeHMS = [NaN NaN NaN NaN];
    timingRaw = struct('timecodeRaw',NaN,'timestampRaw',NaN,'midExposure',NaN, ...
        'dataReceived',NaN,'transmit',NaN,'precSec',NaN,'precFrac',NaN,'param',NaN);
    frameNumber = NaN;
    isRecording = false;
    ok  = false;

    %check is message is long enough to have rigid body info; if expected
    %message ID
    %Could also check if targetID matches (not implemented yet)
    if numel(data) < 70; return; end
    msgID = readU16(data, 1);
    if msgID ~= 7; return; end

    nDataBytes = readU16(data, 3);
    frameNumber = readI32(data, 5);

    % ---- rigid body info ----
    rx  = readF32(data, 37);
    ry  = readF32(data, 41);
    rz  = readF32(data, 45);
    rqx = readF32(data, 49);
    rqy = readF32(data, 53);
    rqz = readF32(data, 57);
    rqw = readF32(data, 61);
    err = readF32(data, 65);
    valid = readU16(data, 69) ~= 0;   % convert to logical
    q   = [rqx rqy rqz rqw];
    pos = [rx ry rz];
    ok  = true;

    % ---- suffix (timing) ----
    suffixOffset = 54;
    packetEnd   = min(numel(data), 4 + nDataBytes);
    suffixStart = packetEnd - suffixOffset + 1;
    suffixFieldsWidth = 50;  % bytes actually needed to decode every field below

    % If suffix exists, look for timing info
    try
        if suffixStart >= 71 && (suffixStart + suffixFieldsWidth - 1) <= numel(data)
            idx = suffixStart;

            timecodeRaw  = readU32(data, idx); idx = idx + 4;
            timecodeSub  = readU32(data, idx); idx = idx + 4; 
            timestampRaw = readF64(data, idx); idx = idx + 8;
            midExposure  = readI64(data, idx); idx = idx + 8;
            dataReceived = readI64(data, idx); idx = idx + 8;
            transmit     = readI64(data, idx); idx = idx + 8;
            precSec      = readI32(data, idx); idx = idx + 4;
            precFrac     = readI32(data, idx); idx = idx + 4;
            param        = readI16(data, idx); idx = idx + 2;

            % --- Recording flag: bit 0 of param ---
            isRecording = bitand(param, 1) == 1;

            % [hh, mm, ss, ff] = decodeTimecode(timecodeRaw);
            % 
            % tsMs = timestampRaw * 1000;         % seconds -> ms
            % timecodeHMS = [hh mm ss ff];
            % timingRaw = struct('timecodeRaw',timecodeRaw,'timestampRaw',timestampRaw, ...
            %     'midExposure',midExposure,'dataReceived',dataReceived,'transmit',transmit, ...
            %     'precSec',precSec,'precFrac',precFrac,'param',double(param));
        end
    catch
        % suffixOffset doesn't land inside the packet; no timing info found
    end
end

function v = readU16(data, idx)
    v = double(uint16(data(idx)) + uint16(data(idx+1))*256);
end

function v = readI32(data, idx)
    b = data(idx:idx+3);
    v = double(typecast(uint8(b), 'int32'));
end

function v = readF32(data, idx)
    b = data(idx:idx+3);
    v = double(typecast(uint8(b), 'single'));
end

function R = quat2rot(q)
    % q = [qx qy qz qw]  (scalar‑last)
    x=q(1); y=q(2); z=q(3); w=q(4);
    R = [1-2*(y^2+z^2),   2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z), 1-2*(x^2+z^2),   2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x^2+y^2)];
end