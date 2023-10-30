function predata()
pre = create();
save predata.mat
function pre = create()
pre.t0 = 0;
pre.dt = 0.01;
pre.v0 = 0;
pre.vn = 5;
pre.theta = [pi;0;0;pi];
pre.gamma = rad2deg(45);
pre.l = 0.1;
pre.Tag = {'AP1' 'AP2' 'AP3' 'AP4' 'AP5' 'AP6' 'AP7' 'AP8' 'AP9';
'WP1' 'WP2' 'WP3' 'WP4' 'WP5' 'WP6' 'WP7' 'WP8' 'WP9';
'TP1' 'TP2' 'TP3' 'TP4' 'TP5' 'TP6' 'TP7' 'TP8' 'TP9';
'RP1' 'RP2' 'RP3' 'RP4' 'RP5' 'RP6' 'RP7' 'RP8' 'RP9'
'EV1' 'EV2' 'EV3' 'EV4' 'EV5' 'EV6' 'EV7' 'EV8' 'EV9'
'WV1' 'WV2' 'WV3' 'WV4' 'WV5' 'WV6' 'WV7' 'WV8' 'WV9'};
