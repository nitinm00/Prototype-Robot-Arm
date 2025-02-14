    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% rtP.fromWS_Signal1_Time0
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.fromWS_Signal1_Data0
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 5;

                    ;% rtP.FromWorkspace_Time0
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 10;

                    ;% rtP.FromWorkspace_Data0
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 15;

                    ;% rtP.FromWorkspace1_Time0
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 20;

                    ;% rtP.FromWorkspace1_Data0
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 25;

                    ;% rtP.weight_Value
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 30;

                    ;% rtP.init_Value
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 36;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% rtB.TmpSignalConversionAtCoordinate
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.weight
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 3;

                    ;% rtB.init
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 9;

                    ;% rtB.INPUT_4_1_1
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 13;

                    ;% rtB.INPUT_1_1_1
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 17;

                    ;% rtB.INPUT_2_1_1
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 21;

                    ;% rtB.INPUT_3_1_1
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 25;

                    ;% rtB.MATLABSystem
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 29;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 8;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.obj
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.obj_l
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 11;
            section.data(11)  = dumData; %prealloc

                    ;% rtDW.INPUT_4_1_1_Discrete
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.INPUT_4_1_1_FirstOutput
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.INPUT_1_1_1_Discrete
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.INPUT_1_1_1_FirstOutput
                    section.data(4).logicalSrcIdx = 5;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.INPUT_2_1_1_Discrete
                    section.data(5).logicalSrcIdx = 6;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.INPUT_2_1_1_FirstOutput
                    section.data(6).logicalSrcIdx = 7;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.INPUT_3_1_1_Discrete
                    section.data(7).logicalSrcIdx = 8;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.INPUT_3_1_1_FirstOutput
                    section.data(8).logicalSrcIdx = 9;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.OUTPUT_1_0_Discrete
                    section.data(9).logicalSrcIdx = 10;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.STATE_1_Discrete
                    section.data(10).logicalSrcIdx = 11;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.freq
                    section.data(11).logicalSrcIdx = 12;
                    section.data(11).dtTransOffset = 10;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 17;
            section.data(17)  = dumData; %prealloc

                    ;% rtDW.fromWS_Signal1_PWORK.TimePtr
                    section.data(1).logicalSrcIdx = 13;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.FromWorkspace_PWORK.TimePtr
                    section.data(2).logicalSrcIdx = 14;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.FromWorkspace1_PWORK.TimePtr
                    section.data(3).logicalSrcIdx = 15;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.OUTPUT_1_0_Simulator
                    section.data(4).logicalSrcIdx = 16;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.OUTPUT_1_0_SimData
                    section.data(5).logicalSrcIdx = 17;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.OUTPUT_1_0_DiagMgr
                    section.data(6).logicalSrcIdx = 18;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.OUTPUT_1_0_ZcLogger
                    section.data(7).logicalSrcIdx = 19;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.OUTPUT_1_0_TsInfo
                    section.data(8).logicalSrcIdx = 20;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.Scope_PWORK.LoggedData
                    section.data(9).logicalSrcIdx = 21;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.SINK_1_RtwLogger
                    section.data(10).logicalSrcIdx = 22;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.SINK_1_RtwLogBuffer
                    section.data(11).logicalSrcIdx = 23;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.SINK_1_RtwLogFcnManager
                    section.data(12).logicalSrcIdx = 24;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.STATE_1_Simulator
                    section.data(13).logicalSrcIdx = 25;
                    section.data(13).dtTransOffset = 12;

                    ;% rtDW.STATE_1_SimData
                    section.data(14).logicalSrcIdx = 26;
                    section.data(14).dtTransOffset = 13;

                    ;% rtDW.STATE_1_DiagMgr
                    section.data(15).logicalSrcIdx = 27;
                    section.data(15).dtTransOffset = 14;

                    ;% rtDW.STATE_1_ZcLogger
                    section.data(16).logicalSrcIdx = 28;
                    section.data(16).dtTransOffset = 15;

                    ;% rtDW.STATE_1_TsInfo
                    section.data(17).logicalSrcIdx = 29;
                    section.data(17).dtTransOffset = 16;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 10;
            section.data(10)  = dumData; %prealloc

                    ;% rtDW.state
                    section.data(1).logicalSrcIdx = 30;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.state_l
                    section.data(2).logicalSrcIdx = 31;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.state_i
                    section.data(3).logicalSrcIdx = 32;
                    section.data(3).dtTransOffset = 3;

                    ;% rtDW.method
                    section.data(4).logicalSrcIdx = 33;
                    section.data(4).dtTransOffset = 628;

                    ;% rtDW.method_d
                    section.data(5).logicalSrcIdx = 34;
                    section.data(5).dtTransOffset = 629;

                    ;% rtDW.state_k
                    section.data(6).logicalSrcIdx = 35;
                    section.data(6).dtTransOffset = 630;

                    ;% rtDW.state_h
                    section.data(7).logicalSrcIdx = 36;
                    section.data(7).dtTransOffset = 632;

                    ;% rtDW.state_n
                    section.data(8).logicalSrcIdx = 37;
                    section.data(8).dtTransOffset = 633;

                    ;% rtDW.state_j
                    section.data(9).logicalSrcIdx = 38;
                    section.data(9).dtTransOffset = 635;

                    ;% rtDW.method_e
                    section.data(10).logicalSrcIdx = 39;
                    section.data(10).dtTransOffset = 1260;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.obj_ln
                    section.data(1).logicalSrcIdx = 40;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.obj_p
                    section.data(2).logicalSrcIdx = 41;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% rtDW.fromWS_Signal1_IWORK.PrevIndex
                    section.data(1).logicalSrcIdx = 42;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.FromWorkspace_IWORK.PrevIndex
                    section.data(2).logicalSrcIdx = 43;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.FromWorkspace1_IWORK.PrevIndex
                    section.data(3).logicalSrcIdx = 44;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.OUTPUT_1_0_Modes
                    section.data(4).logicalSrcIdx = 45;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.STATE_1_Modes
                    section.data(5).logicalSrcIdx = 46;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section

            section.nData     = 17;
            section.data(17)  = dumData; %prealloc

                    ;% rtDW.OUTPUT_1_0_FirstOutput
                    section.data(1).logicalSrcIdx = 47;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.STATE_1_FirstOutput
                    section.data(2).logicalSrcIdx = 48;
                    section.data(2).dtTransOffset = 1;

                    ;% rtDW.objisempty
                    section.data(3).logicalSrcIdx = 49;
                    section.data(3).dtTransOffset = 2;

                    ;% rtDW.state_not_empty
                    section.data(4).logicalSrcIdx = 50;
                    section.data(4).dtTransOffset = 3;

                    ;% rtDW.state_not_empty_k
                    section.data(5).logicalSrcIdx = 51;
                    section.data(5).dtTransOffset = 4;

                    ;% rtDW.state_not_empty_d
                    section.data(6).logicalSrcIdx = 52;
                    section.data(6).dtTransOffset = 5;

                    ;% rtDW.method_not_empty
                    section.data(7).logicalSrcIdx = 53;
                    section.data(7).dtTransOffset = 6;

                    ;% rtDW.freq_not_empty
                    section.data(8).logicalSrcIdx = 54;
                    section.data(8).dtTransOffset = 7;

                    ;% rtDW.method_not_empty_m
                    section.data(9).logicalSrcIdx = 55;
                    section.data(9).dtTransOffset = 8;

                    ;% rtDW.state_not_empty_j
                    section.data(10).logicalSrcIdx = 56;
                    section.data(10).dtTransOffset = 9;

                    ;% rtDW.objisempty_p
                    section.data(11).logicalSrcIdx = 57;
                    section.data(11).dtTransOffset = 10;

                    ;% rtDW.objisempty_n
                    section.data(12).logicalSrcIdx = 58;
                    section.data(12).dtTransOffset = 11;

                    ;% rtDW.state_not_empty_p
                    section.data(13).logicalSrcIdx = 59;
                    section.data(13).dtTransOffset = 12;

                    ;% rtDW.state_not_empty_kp
                    section.data(14).logicalSrcIdx = 60;
                    section.data(14).dtTransOffset = 13;

                    ;% rtDW.state_not_empty_h
                    section.data(15).logicalSrcIdx = 61;
                    section.data(15).dtTransOffset = 14;

                    ;% rtDW.method_not_empty_k
                    section.data(16).logicalSrcIdx = 62;
                    section.data(16).dtTransOffset = 15;

                    ;% rtDW.objisempty_j
                    section.data(17).logicalSrcIdx = 63;
                    section.data(17).dtTransOffset = 16;

            nTotData = nTotData + section.nData;
            dworkMap.sections(8) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 3096637441;
    targMap.checksum1 = 3214740571;
    targMap.checksum2 = 3317619168;
    targMap.checksum3 = 1222207877;

