%% Configuring tuner

tuned_blocks = {'PD Base', 'PD Joint 1', 'PD Joint 2x', ...
    'PD Joint 2z'};

outputs = {'ArmPIDControl/Arm/1', 'ArmPIDControl/Arm/2',... 
    'ArmPIDControl/Arm/3', 'ArmPIDControl/Arm/4'};

reference_signals = {'ArmPIDControl/Demux/1', 'ArmPIDControl/Demux/2',...
    'ArmPIDControl/Demux/3', 'ArmPIDControl/Demux/4'}; 

tuner = slTuner('ArmPIDControl', tuned_blocks);

addPoint(tuner, tuned_blocks);

addPoint(tuner, outputs);

addPoint(tuner, reference_signals)

%% Defining input and tuning system

controls = tuned_blocks;
measurements = outputs;
tau = 0.01;
options = looptuneOptions('RandomStart',20 ,"UseParallel",false);
tr = TuningGoal.StepTracking(reference_signals, measurements, tau, 0);
st1 = looptune(tuner,controls,measurements,tr,options);

writeBlockValue(tuner);