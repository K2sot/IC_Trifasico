function stm32_capture_then_plot_AA()
% Capture-then-plot using AA+ID header (no sync/version/size preamble)
% Frame = 250 bytes:
%  [1]    0xAA
%  [2]    ID (01/02/03/11/12/13/1F/F0)
%  [3..6] cycle_start_sample (LE32)
%  [7]    overflow (1B)
%  [8..11]cycle_start_timestamp (LE32)
%  [12..203] 192B packed samples (128 x 12-bit, 2-in-3)
%  [204..207]freq float (LE)     <-- we use this
%  [208..211]phase float (LE)    <-- and this
%  [212..244]reserved (33B)
%  [245..246]seq (LE16)
%  [247] chID (echo)
%  [248] reserved
%  [249..250] CRC16-CCITT (False) BIG-endian

%% --- User settings ---
portName   = 'COM3';     % <— set your port
baudRate   = 1382400;
Fs         = 7680;
Vref       = 3.3;
adc_bits   = 12; adc_full = 2^adc_bits - 1;
adc_offset = 1.0;        % adjust for your AFE mid-rail
plot_norm  = false;      % true = [-1..1], false = volts
CAP_SECONDS = 10.0;      % capture duration (no plotting during capture)

FRAME_BYTES = 250;

% IDs
HDR = struct( ...
  'VA', uint8(hex2dec('01')), 'VB', uint8(hex2dec('02')), 'VC', uint8(hex2dec('03')), ...
  'IA', uint8(hex2dec('11')), 'IB', uint8(hex2dec('12')), 'IC', uint8(hex2dec('13')), ...
  'IN', uint8(hex2dec('1F')), 'FINFO', uint8(hex2dec('F0')));

chList = {'VA','VB','VC','IA','IB','IC','IN'};
idList = uint8([HDR.VA HDR.VB HDR.VC HDR.IA HDR.IB HDR.IC HDR.IN]); % same order as chList

% Byte positions (MATLAB 1-based)
ID_AT      = 2;
RAW192_AT  = 12;                % 12..203
FREQ_AT    = [204 207];         % LE float
PHASE_AT   = [208 211];         % LE float
SEQ_AT     = [245 246];         % LE16
CHID_AT    = 247; %#ok<NASGU>
CRC_AT     = [249 250];         % BE16

%% --- Open serial (raw binary) ---
fprintf('Opening %s @ %d baud ...\n', portName, baudRate);
sp = serialport(portName, baudRate, 'Timeout', 0.05);
sp.ByteOrder = "little-endian";
try sp.InputBufferSize = 2e6; catch, end
flush(sp);

%% --- Capture (no plotting) ---
rx = uint8([]);           % column buffer
i  = 1;                   % scan pointer (1-based)
t0 = tic; framesOK=0; framesBad=0; framesUsed=0;

% per-channel stores
Y = struct();  Seqs = struct();
% NEW: per-frame freq/phase stores
F = struct();  Phi  = struct();

for k=1:numel(chList)
    Y.(chList{k})    = uint16([]);  %#ok<AGROW>
    Seqs.(chList{k}) = uint16([]);  %#ok<AGROW>
    F.(chList{k})    = [];          %#ok<AGROW>
    Phi.(chList{k})  = [];          %#ok<AGROW>
end

fprintf('Capturing for %.2f s ...\n', CAP_SECONDS);
while toc(t0) < CAP_SECONDS
    nb = sp.NumBytesAvailable;
    if nb > 0
        newb = read(sp, nb, 'uint8');
        rx   = [rx; newb(:)]; %#ok<AGROW>
    end

    % parse as many frames as possible
    while (numel(rx) - i + 1) >= FRAME_BYTES
        if rx(i) ~= uint8(170)   % 0xAA
            i = i + 1;
            continue;
        end
        if i + FRAME_BYTES - 1 > numel(rx)
            break;               % need more data
        end

        fr = rx(i:i+FRAME_BYTES-1);

        % CRC check (CRC over bytes 1..248, compare to BE16 at 249..250)
        crc_rx = uint16(fr(CRC_AT(1)))*256 + uint16(fr(CRC_AT(2)));
        if crc16_ccitt_false(fr(1:end-2)) == crc_rx
            framesOK = framesOK + 1;

            hdr = fr(ID_AT);
            idx = find(idList == hdr, 1, 'first');
            if ~isempty(idx)
                ch   = chList{idx};

                raw192 = fr(RAW192_AT:RAW192_AT+191);
                s128   = unpack12_2in3(raw192);         % uint16 [1x128]
                seqLE  = le16(fr(SEQ_AT(1):SEQ_AT(2))); % uint16

                % NEW: per-frame freq & phase (LE single -> double)
                freqLE = double(typecast(uint8(fr(FREQ_AT(1):FREQ_AT(2))), 'single'));
                phLE   = double(typecast(uint8(fr(PHASE_AT(1):PHASE_AT(2))), 'single'));

                Y.(ch)    = [Y.(ch), s128];     %#ok<AGROW>
                Seqs.(ch) = [Seqs.(ch), seqLE]; %#ok<AGROW>
                F.(ch)    = [F.(ch),   freqLE]; %#ok<AGROW>
                Phi.(ch)  = [Phi.(ch), phLE];   %#ok<AGROW>
                framesUsed = framesUsed + 1;
            end

            i = i + FRAME_BYTES;  % consume this frame
        else
            framesBad = framesBad + 1;
            i = i + 1;            % false AA inside payload; slide 1 byte
        end
    end

    % compact buffer sometimes
    if i > 10000
        rx = rx(i:end);
        i = 1;
    end
end
clear sp;
fprintf('Capture done. Frames OK=%d  BadCRC=%d  Used(ch)=%d\n', framesOK, framesBad, framesUsed);

%% --- Convert & plot all samples (concatenated per channel) ---
figure('Name','STM32 capture-then-plot (AA header)','NumberTitle','off');
tiledlayout(4,2,'Padding','compact','TileSpacing','compact');

for k = 1:numel(chList)
    ch = chList{k};
    yraw = Y.(ch);
    nexttile;
    if isempty(yraw)
        title(sprintf('%s (no frames)', ch)); axis off; continue;
    end

    if plot_norm
        y = (double(yraw) - 2048) / 2048; ylab = 'Normalized';
    else
        y = double(yraw)/double(adc_full)*Vref - adc_offset; ylab = 'Volts';
    end
    t = (0:numel(y)-1)/Fs;

    plot(t, y, 'LineWidth', 1.0); grid on; xlim([0 t(end)]);
    title(sprintf('%s  (%d cycles, %d samples)', ch, numel(yraw)/128, numel(yraw)));
    xlabel('Time [s]'); ylabel(ylab);
end

fprintf('Seq ranges per channel:\n');
for k=1:numel(chList)
    ch = chList{k};
    s  = Seqs.(ch);
    if isempty(s)
        fprintf('  %s: <none>\n', ch);
    else
        fprintf('  %s: %u .. %u (n=%d)\n', ch, s(1), s(end), numel(s));
    end
end

%% --- NEW: Mean frequency of phases (VA/VB/VC) from frame metadata ---
phases = {'VA','VB','VC'};
fprintf('\nMean frequency (from frame metadata):\n');
for k=1:numel(phases)
    ch = phases{k};
    fk = F.(ch);
    fk = fk(isfinite(fk) & fk>0);  % sanitize
    if isempty(fk)
        fprintf('  %s: <no freq field, fallback to FFT peak>\n', ch);
    else
        fprintf('  %s: mean = %.6f Hz,  std = %.6f,  n = %d\n', ch, mean(fk), std(fk), numel(fk));
    end
end

%% --- NEW: FFT (Welch PSD) for VA/VB/VC ---
figure('Name','Phase FFT (Welch PSD)','NumberTitle','off');
tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

Nseg   = min(8192, max(1024, 2^nextpow2(Fs)));   % reasonable segment
Nover  = round(0.5*Nseg);
Nfft   = max(4096, 2^nextpow2(Nseg));

for k=1:numel(phases)
    ch = phases{k};
    yraw = Y.(ch);
    nexttile;
    if isempty(yraw)
        title(sprintf('%s (no data)', ch)); grid on; continue;
    end
    % volts or normalized, consistent with above
    if plot_norm
        y = (double(yraw) - 2048) / 2048;
        ylab = 'Power/Frequency [dB]';
    else
        y = double(yraw)/double(adc_full)*Vref - adc_offset;
        ylab = 'Power/Frequency [dBV^2/Hz]';
    end

    % Welch PSD
    [Pxx,f] = pwelch(y, hanning(Nseg,'periodic'), Nover, Nfft, Fs, 'onesided');
    PdB = 10*log10(Pxx+eps);
    plot(f, PdB, 'LineWidth', 1.0); grid on; xlim([0, Fs/2]);
    xlabel('Frequency [Hz]'); ylabel(ylab);
    title(sprintf('%s Welch PSD (Nseg=%d, overlap=%d)', ch, Nseg, Nover));

    % Optional: mark peak (fundamental)
    [~,imax] = max(Pxx);
    fpk = f(imax);
    hold on; xline(fpk, '--');
    text(fpk, max(PdB), sprintf('  %.3f Hz', fpk), 'VerticalAlignment','bottom');
end

%% ===== helpers =====
function s128 = unpack12_2in3(bytes192)
    % bytes192: 192x1 uint8 → 1x128 uint16 (0..4095), 2-in-3 packing
    s128 = zeros(1,128,'uint16');
    bi = 1; si = 1;
    for kk = 1:64
        b0 = uint16(bytes192(bi+0));
        b1 = uint16(bytes192(bi+1));
        b2 = uint16(bytes192(bi+2));
        a = bitor(bitshift(b0,4), bitshift(bitand(b1,240),-4)); % (b0<<4)|(b1>>4)
        b = bitor(bitshift(bitand(b1,15),8), b2);               % ((b1&0x0F)<<8)|b2
        s128(si)   = bitand(a, 4095);
        s128(si+1) = bitand(b, 4095);
        bi = bi + 3; si = si + 2;
    end
end

function v = le16(b2)
    v = uint16(b2(1)) + bitshift(uint16(b2(2)),8);
end

function c = crc16_ccitt_false(data)
    % CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no xorout)
    data = uint8(data(:));
    poly = uint16(hex2dec('1021'));
    c    = uint16(hex2dec('FFFF'));
    for ii=1:numel(data)
        c = bitxor(c, bitshift(uint16(data(ii)),8));
        for b=1:8
            if bitand(c, hex2dec('8000'))
                c = bitxor(bitshift(c,1), poly);
            else
                c = bitshift(c,1);
            end
            c = bitand(c, hex2dec('FFFF'));
        end
    end
end
end
