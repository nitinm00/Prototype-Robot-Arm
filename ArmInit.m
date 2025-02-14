Ts = 0.001;
tvec = linspace(0,10,11);
demo = [ 1.5 1.4 1.3 1.2 1.1 1 0.9 0.8 0.7 0.6 0.5;  1 1 1 1 1 1 1 1 1 1 1;  0.5 0.6 0.7 0.8 0.9 1 1.1 1.2 1.3 1.4 1.5].';
% Demo = load("pidTunePoint.mat");
% demo = [];
% for i=1:1:3
%     demo.add(Demo{i}.Data);
% end

[ArmTree, ArmInfo] = importrobot("./ArmModels/Arm3.slx");