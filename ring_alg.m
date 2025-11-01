%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main file to implement Fourier Ptychography reconstruction algorithm
% Lei Tian, et.al, Biomedical Optics Express 5, 2376-2389 (2014).
% last modified on 10/07/2015
% by Lei Tian, lei_tian@alum.mit.edu

F = @(x) fftshift(fft2(x));
Ft = @(x) ifft2(ifftshift(x));
row = @(x) x(:).';

filedir = '/Users/ajeems/Downloads/LU/phase shift files/final ver/sample_2';
out_dir = '/Users/ajeems/Downloads/LU/phase shift files/final ver/resultsdir';
addpath('/Users/ajeems/Downloads/LU/phase shift files/final ver/FP_Func');
addpath('/Users/ajeems/Downloads/LU/phase shift files/final ver/natsortfiles');

cd(filedir);

loadimages = 1;  % 1 = load all the input images.   0 = assume images are already loaded to save time when iterating parameters
imglist = dir('*.png');  % Generate the image list
%imglist = dir([filedir, '*.tif']);
N = natsortfiles({imglist.name});
numlit = 1;  % define number of LEDs used to capture each image


%%% ReLATED TO OPTICAL SETUP
n1 = 960; n2 = 1280;  % input image size

lambda = 0.62858;    % wavelength of illumination, assume monochromatic  HUB75 2mm pitch  R: 628.58nm  G: 519.5   B: 465.1
NA = 0.6;   % numerical aperture of the objective
mag = 40;    % !!!
dpix_c = 3.0;  % camera pixel size microns

%%% !!!
Np = 512;  % ? # of pixels at the output image patch. Each patch will assign a single k-vector, the image patch size cannot be too large to keep the single-k assumption holds

ds_led = 2.e3;   % Spacing between neighboring LEDs 8mm
decimation_led = 1;  % Use every LED (no decimation for full selection)
z_led = 2.8e+7;  % distance from the center LED to the object

%%% !!!
dia_led = 8.75;   % For ~70mm physical diameter (8.75 * 8mm)

Ibk_thresh = 3000; % background dark value

lit_cenv = 15;   % Centering for 32x32 grid
lit_cenh = 15;
vled = (0:31)-lit_cenv;
hled = (0:31)-lit_cenh;

pattern_type = 'ring';  % 'filled' or 'ring'

% Most user inputs and configuration settings above this line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[hhled,vvled] = meshgrid(hled,vled);
rrled = sqrt(hhled.^2+vvled.^2);

% Modified for pattern type
switch pattern_type
    case 'filled'
        LitCoord = rrled < dia_led/2;
    case 'ring'
        outer_radius = dia_led / 2;
        inner_radius = 3.217;  % Thin ring for exactly 24 LEDs
        LitCoord = (rrled <= outer_radius) & (rrled >= inner_radius);
end

LitCoord(:,mod(vled,decimation_led)~=0) = 0;  % decimation factor
LitCoord(mod(hled,decimation_led)~=0, :) = 0;  % decimation factor

% Remove exact center (included for consistency)
[~, center_idx] = min(rrled(:));
LitCoord(center_idx) = 0;


Nled = sum(LitCoord(:));  % total number of LEDs used in the experiment (24)
Litidx = find(LitCoord);   % index of LEDs used in the experiment



fidled = fopen(strcat("./", "LEDpattern.txt"), 'w');

fprintf(fidled,"int LEDpattern[%d][%d] = {\n", length(hled), length(vled));

for y = 1: length(vled)
    for x = 1: length(hled)
        if(x == length(hled) && y == length(vled))
          fprintf(fidled,"%d}};\n", LitCoord(y,x));
        elseif(x == 1)
          fprintf(fidled,"{%d,", LitCoord(y,x));
        elseif(x < length(hled))
          fprintf(fidled,"%d,", LitCoord(y,x));
        else
          fprintf(fidled,"%d},\n", LitCoord(y,x));
        end
    end
end

% Modified: Set CenterCoord to all zeros
CenterCoord = zeros(length(vled), length(hled));
fprintf(fidled,"\nint LEDcenter[%d][%d] = {\n", length(hled), length(vled));
for y = 1: length(vled)
    for x = 1: length(hled)
        if(x == length(hled) && y == length(vled))
          fprintf(fidled,"%d}};\n", CenterCoord(y,x));
        elseif(x == 1)
          fprintf(fidled,"{%d,", CenterCoord(y,x));
        elseif(x < length(hled))
          fprintf(fidled,"%d,", CenterCoord(y,x));
        else
          fprintf(fidled,"%d},\n", CenterCoord(y,x));
        end
    end
end

fclose(fidled);

um_m = NA/lambda;   % maximum spatial frequency set by NA
dx0 = 1/um_m/2;   % system resolution based on the NA
dpix_m = dpix_c/mag;  % effective image pixel size on the object plane
FoV = Np*dpix_m;   % FoV in the object space


if(loadimages == 1)
  fprintf('loading the images...\n');
  tic;
  Nimg = length(imglist);
  if Nimg ~= Nled
      error('Number of images (%d) must match Nled (%d). Ensure you have exactly %d PNG files in %s, captured in the order corresponding to sorted Litidx.', Nimg, Nled, Nled, filedir);
  end
  Iall = zeros(n1,n2,Nimg,'uint16');
  Ide = zeros(n1,n2,Nimg,'uint16');
  Ibk = zeros(Nimg,1);
  for m = 1:Nimg
      fn = [N{m}];
      disp(fn);
      Iall(:,:,m) = double(imread(fn));  %Read 16-bit monochrome TIFF
      Irgb(:,:,:) = demosaic(Iall(:,:,m),'gbrg');
      Ide(:,:,m) = Irgb(:,:,1);
      %Iall(:,:,m) = double(imread(fn))(:,:,1);  %Read red channel from 48-bit color TIFF.

      bk1 = mean2(double(Ide(1:10,1:10,m)));  % Specify background region coordinates
      bk2 = mean2(double(Ide(940:960,1260:1280,m)));

       %Ibk(m) = 1;
      Ibk(m) = mean([bk1,bk2]);
     % if Ibk(m)>Ibk_thresh  % If Ibk is larger than some threshold, it is not noise!
      %    Ibk(m) = Ibk(m-1);
    %  end
  end
  fprintf('\nFinished loading images\n');
  toc;
end



if mod(Np,2) == 1
    du = 1/dpix_m/(Np-1);  % sampling size at Fourier plane set by the image size (FoV)
else
    du = 1/FoV;  % sampling size at Fourier plane is always = 1/FoV
end

m = 1:Np;   % low-pass filter diameter set by the NA = bandwidth of a single measurment in index
[mm,nn] = meshgrid(m-round((Np+1)/2));
ridx = sqrt(mm.^2+nn.^2);
um_idx = um_m/du;  % assume a circular pupil function, lpf due to finite NA
w_NA = double(ridx<um_idx);  % generate cutoff window by NA
Ps_otf = double(ridx<2*um_idx);   % support of OTF is 2x of ATF(NA)
phC = ones(Np);
aberration = ones(Np);
pupil = w_NA.*phC.*aberration;
clear m mm nn Irgb



dd = sqrt((-hhled*ds_led).^2+(-vvled*ds_led).^2+z_led.^2);
sin_thetav = (-hhled*ds_led)./dd;  % corresponding angles for each LEDs
sin_thetah = (-vvled*ds_led)./dd;
illumination_na = sqrt(sin_thetav.^2+sin_thetah.^2);
illumination_na_used = illumination_na(LitCoord);
NBF = sum(illumination_na_used<NA);   % number of brightfield image

vled = sin_thetav/lambda;  % corresponding spatial freq for each LEDs
uled = sin_thetah/lambda;

idx_u = round(uled/du);  % spatial freq index for each plane wave relative to the center
idx_v = round(vled/du);

um_p = max(illumination_na_used)/lambda+um_m;  % maxium spatial frequency achievable based on the maximum illumination angle from the LED array and NA of the objective
dx0_p = 1/um_p/2;  % resolution achieved after freq post-processing
disp(['synthetic NA is ',num2str(um_p*lambda)]);

N_obj = round(2*um_p/du)*2; % assume the max spatial freq of the original object    um_obj>um_p    assume the # of pixels of the original object
N_obj = ceil(N_obj/Np)*Np;   % need to enforce N_obj/Np = integer to ensure no FT artifacts
um_obj = du*N_obj/2;  % max spatial freq of the original object
dx_obj = 1/um_obj/2;   % sampling size of the object (=pixel size of the test image)
[xp,yp] = meshgrid((-Np/2:Np/2-1)*dpix_m);
x0 = (-N_obj/2:N_obj/2/2-1)*dx_obj;
[xx0,yy0] = meshgrid(x0);
[u,v] = meshgrid(-um_obj:du:um_obj-du);   %% define propagation transfer function
z0=0;
H0 = exp(1i*2*pi/lambda*z0)*exp(-1i*pi*lambda*z0*(u.^2+v.^2));
% OR angular spectrum
% H0 = exp(1i*2*pi*sqrt((1/lambda^2-u.^2-v.^2).*double(sqrt(u.^2+v.^2)<1/lambda))*dz);

Imea = double(Ide(n1/2 - Np/2 : n1/2 + Np/2 - 1 ,n2/2 - Np/2 : n2/2 + Np/2 - 1,:)); %Use only a portion of the input images

ledidx = 1:Nled;
ledidx = reshape(ledidx,numlit,Nled);
lit = Litidx(ledidx);
lit = reshape(lit,numlit,Nled);

[dis_lit2,idx_led] = sort(reshape(illumination_na_used,1,Nled));  % reorder LED indices based on illumination NA

Nsh_lit = zeros(numlit,Nled);
Nsv_lit = zeros(numlit,Nled);

for m = 1:Nled
    lit0 = lit(:,m);   % corresponding index of spatial freq for the LEDs are lit
    Nsh_lit(:,m) = idx_u(lit0);
    Nsv_lit(:,m) = idx_v(lit0);
end

Ns = [];
Ns(:,:,1) = Nsv_lit;   % reorder the LED indices and intensity measurements according the previous
Ns(:,:,2) = Nsh_lit;

Imea_reorder = Imea(:,:,idx_led);
Ibk_reorder = Ibk(idx_led);

% pre-processing the data to DENOISING is IMPORTANT
% background subtraction
Ithresh_reorder = Imea_reorder;
for m = 1:Nled
    Itmp = Ithresh_reorder(:,:,m);
    Itmp = Itmp-Ibk_reorder(m);
%     Itmp = awgn(Itmp,0,'measured');
    Itmp(Itmp<0) = 0;
    Ithresh_reorder(:,:,m) = Itmp;
end

Ns_reorder = Ns(:,idx_led,:);
clear Imea; 

Nused = size(Imea_reorder(1,1,:),3);

idx_used = 1:Nled;
I = Ithresh_reorder(:,:,idx_used);
Ns2 = Ns_reorder(:,idx_used,:);

%% reconstruction algorithm options: opts
%   tol: maximum change of error allowed in two consecutive iterations
    %   maxIter: maximum iterations
    %   minIter: minimum iterations
    %   monotone (1, default): if monotone, error has to monotonically dropping
    %   when iters>minIter
%   display: display results (0: no (default) 1: yes)
    %   saveIterResult: save results at each step as images (0: no (default) 1: yes)
    %   mode: display in 'real' space or 'fourier' space.
    %   out_dir: saving directory
%   O0, P0: initial guesses for O and P
    %   OP_alpha: regularization parameter for O
    %   OP_beta: regularization parameter for P
%   scale: LED brightness map
%   H0: known portion of the aberration function,
        % e.g. sample with a known defocus induce a quardratic aberration
        % function can be defined here
%   poscalibrate: flag for LED position correction using
    % '0': no correction
    % 'sa': simulated annealing method
        % calbratetol: parameter in controlling error tolence in sa
    % 'ga': genetic algorithm
    % caution: takes consierably much longer time to compute a single iteration
%   F, Ft: operators of Fourier transform and inverse

%%%% VALUES FOR THE ALGORITHM
opts.tol = 0.1;
opts.maxIter = 10;
opts.minIter = 2;
opts.monotone = 1;
% 'full', display every subroutin,
% 'iter', display only results from outer loop
% 0, no display
opts.display = 'full';%0;%'iter';
upsamp = @(x) padarray(x,[(N_obj-Np)/2,(N_obj-Np)/2]);
opts.O0 = F(sqrt(I(:,:,1)));
opts.O0 = upsamp(opts.O0);
opts.P0 = w_NA;
opts.Ps = w_NA;
opts.iters = 1;
opts.mode = 'fourier'; %real
opts.scale = ones(Nused,1);
opts.OP_alpha = 1;
opts.OP_beta = 1e3 ;
opts.poscalibrate =0;
opts.calbratetol = 1e-1;
opts.F = F;
opts.Ft = Ft;
opts.StepSize = 0.1;

%% algorithm starts
%testc = evalc("[O,P,err_pc,c,Ns_cal] = AlterMin(I,[N_obj,N_obj],round(Ns2),opts);")

diary tempdiary

[O,P,err_pc,c,Ns_cal] = AlterMin(I,[N_obj,N_obj],round(Ns2),opts);
%%
% 

%

diary on
diaryfid = fopen("tempdiary");
diarytext = fscanf(diaryfid,"%c");
fclose(diaryfid);

%% save results
fn = ['RandLit-',num2str(numlit),'-',num2str(Nused)];
save([out_dir,'\',fn],'O','P','err_pc','c','Ns_cal');

%f1 = figure; imagesc(-angle(O),[-.6,1]); axis image; colormap gray; axis off

fprintf('processing complete\n');

%I = mat2gray(real(O));
%figure(2);imshow(I);

%figure(2);imshow(angle(O),[]);
figure(1);imshow(abs(O),[])
% figure(3);imagesc(-angle(O));colormap gray;
filenamebase = datestr(now(), 'yyyy-mm-dd_HH-MM-SS');


scalefactor = 65536 / max(max(abs(O)));
imwrite(uint16(abs(O).*scalefactor), strcat(out_dir,"/", filenamebase, ".tif"));

fidtxt = fopen(strcat(out_dir, "/", filenamebase, ".txt"), 'w');
fprintf(fidtxt, "%s = %d\n", 'n1', n1);
fprintf(fidtxt, "%s = %d\n", 'n2', n2);
fprintf(fidtxt, "%s = %d\n", 'lambda', lambda);
fprintf(fidtxt, "%s = %d\n", 'NA', NA);
fprintf(fidtxt, "%s = %d\n", 'mag', mag);
fprintf(fidtxt, "%s = %d\n", 'dpix_c', dpix_c);
fprintf(fidtxt, "%s = %d\n", 'Np', Np);
fprintf(fidtxt, "%s = %d\n", 'ds_led', ds_led);
fprintf(fidtxt, "%s = %d\n", 'decimation_led', decimation_led);
fprintf(fidtxt, "%s = %d\n", 'z_led', z_led);
fprintf(fidtxt, "%s = %.3f\n", 'dia_led', dia_led);
fprintf(fidtxt, "%s = %d\n", 'lit_cenv', lit_cenv);
fprintf(fidtxt, "%s = %d\n", 'lit_cenh', lit_cenh);
fprintf(fidtxt, "%s = %d\n", 'Nled', Nled);
fprintf(fidtxt, "%s = %s\n", 'Synthetic NA', num2str(um_p*lambda));
fprintf(fidtxt, "\n%s\n", 'Synthetic NA', diarytext);
fclose(fidtxt);