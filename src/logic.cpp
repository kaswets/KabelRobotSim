#include "robot.h"
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ============================================================
//  ParseCommand - zelfde als ag_ParseData.cpp op STM32
// ============================================================
void ParseCommand(const std::string& cmd)
{
    if (cmd.empty()) return;

    std::string partCommand, partData1;
    size_t sp = cmd.find(' ');
    if (sp != std::string::npos) {
        partCommand = cmd.substr(0, sp);
        partData1   = cmd.substr(sp + 1);
    } else {
        partCommand = cmd;
    }

    int val = partData1.empty() ? 0 : atoi(partData1.c_str());

    if      (partCommand == "MainX" || partCommand == "mainx") WantedMainX = (float)val;
    else if (partCommand == "MainY" || partCommand == "mainy") WantedMainY = (float)val;
    else if (partCommand == "MainZ" || partCommand == "mainz") WantedMainZ = (float)val;
    else if (partCommand == "RotX"  || partCommand == "rotx")  WantedRotX  = (float)val;
    else if (partCommand == "RotY"  || partCommand == "roty")  WantedRotY  = (float)val;
    else if (partCommand == "RotZ"  || partCommand == "rotz")  WantedRotZ  = (float)val;
    else if (partCommand == "ToCal") {
        Mot1On=Mot2On=Mot3On=Mot4On=Mot5On=Mot6On=Mot7On=Mot8On=0;
    }
    else if (partCommand == "ToRun") {
        Mot1On=Mot2On=Mot3On=Mot4On=Mot5On=Mot6On=Mot7On=Mot8On=1;
    }
    else if (partCommand == "Center") {
        WantedMainX=WantedMainY=WantedMainZ=0;
        WantedRotX=WantedRotY=WantedRotZ=0;
    }
    else if (partCommand == "StepUnit") {
        if (val > 0) StepUnit = (float)val;
    }
}

// ============================================================
//  RobotLoop - exact zelfde logica als loop() in zz_Main.cpp
//  inclusief de bubble sort sync bug zodat je hem kan zien!
// ============================================================
void RobotLoop()
{
    // --- Kinematica: hoekpunten updaten als positie veranderd ---
    if (Inpos == 1)
    {
        if (WantedRotX != ActualRotX) {
            for (int j=1; j<9; j++)
                matrixRotX(ObjectCorner[j].X, ObjectCorner[j].Y, ObjectCorner[j].Z,
                           ActualRotX - WantedRotX, j);
            go = 1;
        }
        if (WantedRotY != ActualRotY) {
            for (int j=1; j<9; j++)
                matrixRotY(ObjectCorner[j].X, ObjectCorner[j].Y, ObjectCorner[j].Z,
                           ActualRotY - WantedRotY, j);
            go = 1;
        }
        if (WantedRotZ != ActualRotZ) {
            for (int j=1; j<9; j++)
                matrixRotZ(ObjectCorner[j].X, ObjectCorner[j].Y, ObjectCorner[j].Z,
                           ActualRotZ - WantedRotZ, j);
            go = 1;
        }
        if (WantedMainX != ActualMainX || WantedMainY != ActualMainY || WantedMainZ != ActualMainZ) {
            for (int j=1; j<9; j++)
                matrixTrans(ObjectCorner[j].X, ObjectCorner[j].Y, ObjectCorner[j].Z,
                            (ActualMainX - WantedMainX),
                            (ActualMainY - WantedMainY),
                           -(ActualMainZ - WantedMainZ), j);
            go = 1;
        }

        Mot1WantedLength = Mot1(ObjectCorner[1].X, ObjectCorner[1].Y, ObjectCorner[1].Z);
        Mot2WantedLength = Mot2(ObjectCorner[2].X, ObjectCorner[2].Y, ObjectCorner[2].Z);
        Mot3WantedLength = Mot3(ObjectCorner[3].X, ObjectCorner[3].Y, ObjectCorner[3].Z);
        Mot4WantedLength = Mot4(ObjectCorner[4].X, ObjectCorner[4].Y, ObjectCorner[4].Z);
        Mot5WantedLength = Mot5(ObjectCorner[5].X, ObjectCorner[5].Y, ObjectCorner[5].Z);
        Mot6WantedLength = Mot6(ObjectCorner[6].X, ObjectCorner[6].Y, ObjectCorner[6].Z);
        Mot7WantedLength = Mot7(ObjectCorner[7].X, ObjectCorner[7].Y, ObjectCorner[7].Z);
        Mot8WantedLength = Mot8(ObjectCorner[8].X, ObjectCorner[8].Y, ObjectCorner[8].Z);
    }

    // --- Motor sync berekening (met de bekende bubble sort bug) ---
    if (go == 1)
    {
        go    = 0;
        Inpos = 0;

        TotalMove[1] = abs(Mot1WantedLength - Mot1ActualLength);
        TotalMove[2] = abs(Mot2WantedLength - Mot2ActualLength);
        TotalMove[3] = abs(Mot3WantedLength - Mot3ActualLength);
        TotalMove[4] = abs(Mot4WantedLength - Mot4ActualLength);
        TotalMove[5] = abs(Mot5WantedLength - Mot5ActualLength);
        TotalMove[6] = abs(Mot6WantedLength - Mot6ActualLength);
        TotalMove[7] = abs(Mot7WantedLength - Mot7ActualLength);
        TotalMove[8] = abs(Mot8WantedLength - Mot8ActualLength);

        // Sla originele waarden op VOOR de sort (dit is de fix!)
        long OrigMove[9];
        for (int i=1; i<=8; i++) OrigMove[i] = TotalMove[i];

        // Bubble sort - zelfde als STM32
        for (int i=1; i<=8; i++) MotorNbr[i] = i;
        bool swapped;
        do {
            swapped = false;
            for (int i=1; i<8; i++) {
                if (TotalMove[i] < TotalMove[i+1]) {
                    long t = TotalMove[i+1]; TotalMove[i+1] = TotalMove[i]; TotalMove[i] = t;
                    int  n = MotorNbr[i+1];  MotorNbr[i+1]  = MotorNbr[i];  MotorNbr[i]  = n;
                    swapped = true;
                }
            }
        } while (swapped);

        // TotalMove[1] is nu de grootste beweging
        if (TotalMove[1] == 0) { Inpos = 1; return; }
        Procent100 = 100.0f / (float)TotalMove[1];

        // === ORIGINELE CODE (met bug) ===
        // Mot1PulseProcent = (Procent100 * TotalMove[1]) + 0.5  <- altijd 100, ongeacht welke motor
        // Mot2PulseProcent = (Procent100 * TotalMove[2]) + 0.5  <- altijd 2e grootste, niet Motor2!

        // === GECORRIGEERDE CODE (gebruik OrigMove per motor) ===
        Mot1PulseProcent = (int)((Procent100 * OrigMove[1]) + 0.5f); if(Mot1PulseProcent<1) Mot1PulseProcent=1;
        Mot2PulseProcent = (int)((Procent100 * OrigMove[2]) + 0.5f); if(Mot2PulseProcent<1) Mot2PulseProcent=1;
        Mot3PulseProcent = (int)((Procent100 * OrigMove[3]) + 0.5f); if(Mot3PulseProcent<1) Mot3PulseProcent=1;
        Mot4PulseProcent = (int)((Procent100 * OrigMove[4]) + 0.5f); if(Mot4PulseProcent<1) Mot4PulseProcent=1;
        Mot5PulseProcent = (int)((Procent100 * OrigMove[5]) + 0.5f); if(Mot5PulseProcent<1) Mot5PulseProcent=1;
        Mot6PulseProcent = (int)((Procent100 * OrigMove[6]) + 0.5f); if(Mot6PulseProcent<1) Mot6PulseProcent=1;
        Mot7PulseProcent = (int)((Procent100 * OrigMove[7]) + 0.5f); if(Mot7PulseProcent<1) Mot7PulseProcent=1;
        Mot8PulseProcent = (int)((Procent100 * OrigMove[8]) + 0.5f); if(Mot8PulseProcent<1) Mot8PulseProcent=1;
    }

    // --- InPosition check ---
    if (abs(Mot1WantedLength-Mot1ActualLength) < 25 &&
        abs(Mot2WantedLength-Mot2ActualLength) < 25 &&
        abs(Mot3WantedLength-Mot3ActualLength) < 25 &&
        abs(Mot4WantedLength-Mot4ActualLength) < 25 &&
        abs(Mot5WantedLength-Mot5ActualLength) < 25 &&
        abs(Mot6WantedLength-Mot6ActualLength) < 25 &&
        abs(Mot7WantedLength-Mot7ActualLength) < 25 &&
        abs(Mot8WantedLength-Mot8ActualLength) < 25)
    {
        ActualMainX = WantedMainX; ActualMainY = WantedMainY; ActualMainZ = WantedMainZ;
        ActualRotX  = WantedRotX;  ActualRotY  = WantedRotY;  ActualRotZ  = WantedRotZ;
        // Set actual lengths exact at target
        Mot1ActualLength = Mot1WantedLength; Mot2ActualLength = Mot2WantedLength;
        Mot3ActualLength = Mot3WantedLength; Mot4ActualLength = Mot4WantedLength;
        Mot5ActualLength = Mot5WantedLength; Mot6ActualLength = Mot6WantedLength;
        Mot7ActualLength = Mot7WantedLength; Mot8ActualLength = Mot8WantedLength;
        Inpos = 1;
    }
}
