%trim model: dx/dt=Ax+Bu and y= Cx+Du u is input x is state y is output
%four states: beta,phi,yaw rate, roll rate
%two inputs:rudder and aieleron deflection

%coefficients(a for state matrix, b for control matrix

A = [-0.0558 -0.9968  0.0802 0.0415;
       0.598  -0.115 -0.0318      0;
       -3.05   0.388 -0.4650      0;
           0  0.0805       1      0];

B = [ 0.00729       0;
       -0.475 0.00775;
        0.153   0.143;
            0       0];

C = [0 1 0 0;
     0 0 0 1];

D = [0 0;
     0 0];

%with this code, i'll create a state-space model.

spacemodel = ss(A,B,C,D);

%labeling inputs outputs and states:
%rudder and aileron deflections affect yaw rate and bank angle

spacemodel.InputName =["rudder" "aileron"];
spacemodel.OutputName = ["yaw rate" "bank angle"];
spacemodel.StateName = ["beta" "yaw" "roll" "phi"];

%creating the map for dutch roll
%pzplot shows system poles and zeros on complex, it shows poles with x, and
%zeros with 0
%pole is a value makes a transfer functions denominator 0. poles tells a
%lot about systems stability, frequency etc.

%i will try to design a compensator to increase damping the poles.
%increasing damping of poles means reducing oscillation, and it means a
%more stable system.
pzp = pzplot(spacemodel);
pzp.FrequencyUnit = "rad/s";
grid off

%now lets determine a suitable control strategy. in order to do that check
%impulse response using impulseplot()
%impulse response means the systems response to a input signal. as it can
%seem, there is a big response to rudder input. there is an oscillation on
%bank angle is nonzero. for aileron, system looks like giving more stable
%response.

%yaw dampers uses yaw rate as sensed output and rudder as the input. Let's
%check frequency response. it means a systems response to inputs from
%different frequencies. let's choose it as a I/O pair(input/output pair)

%here, a bode plot will be used. a bode plot show the systems frequency
%response it has a magnitude plot which shows what frequency does the
%system change the signal. and it has a phase plot which shows the delay
%relative to input.
spacemodel11 = spacemodel("yaw","rudder");
bp = bodeplot(spacemodel);
bp.FrequencyUnit ="rad/s";
bp.MagnitudeUnit = "dB";
bp.PhaseUnit = "deg";

%as it can seem from plot, there is a authority around 1 rad/s which is
%dutch roll mode. this is the point we want to fix. the parameter here is
%damping ratio zeta > 0.35 and natural frequency Wn<1 rad/s.
%I will use a root locus technique. simplest compensator is gain. root
%locus technique gives you a reasonable gain value related to model11.
%Using positive feedback:

rp= rlocusplot(-spacemodel11);
rp.FrequencyUnit = "rad/s";

%from this plot we can see best value for gain is 2.85 for lowest damping
%possible which is 0.45

%we will setup a feedback loop with K and check the impulse response. siso
%means single input single output feedback loop.

%feedback code block creates a feedback closed loop transfer function

k = 2.85;
cl11 = feedback(spacemodel11, -k); %we create a transfer function with gain k

impulseplot(spacemodel11,"b--",cl11,"r") %b-- is blue dashed line, spacemodel11 is first system cl11 is second system. we are comparing.
legend("open-loop","closed-loop",Location="SouthEast");

%as we can see it works. lets close the loop to se also aileron response.
%since it's a MIMO(multiple input multiple output) we control what inputs
%are involved to the loop. you dont need these index for SISO

cloop=feedback(spacemodel,-k,1,1);
impulseplot(spacemodel,"b--",cloop,"r",20)

%it's well damped, but now bank angle is not normal. bank angle shows a
%spiral mode. spiral mode is a bank without input and it's a not wanted
%situation. it happens very slowly. Engineers use a washout filter to fix
%this. This filter blocks constant and slow-changed signals such as this
%spiral mode. this is (k*s)/(s+a). Forming a filter: (zpk forms a system as
%zero-pole-gain form

H = zpk(0,-0.2,1);
oloop = H * (-spacemodel11);
rp=rlocusplot(oloop); %to determine a gain for washout filter. 
grid

%choosing best gain from graph:

k=2.34;
wof = -k*H; %washout compensator
cloop = feedback(spacemodel,wof,1,1);
impulseplot(spacemodel,"b--",cloop,"r",20)