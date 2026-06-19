#include "robot.h"
#include <algorithm>
#include <cstdio>

// ============================================================
//  Alle globale variabelen - zelfde als zz_Main.cpp
// ============================================================
float StepUnit           = 168.0f;
float StepSpeed          = 500.0f;
float LengthBlockBottom  = 118.0f;
float WidthBlockBottom   = 48.0f;
float HighBlock          = 41.0f;
float LengthBlockTop     = 68.0f;
float WidthBlockTop      = 78.0f;
float LengthFrame        = 320.0f;
float WidhtFrame         = 200.0f;
float HighFrame          = 280.0f;
float DiaKatrol          = 21.40f;

float ActualMainX = 0, ActualMainY = 0, ActualMainZ = 0;
float WantedMainX = 0, WantedMainY = 0, WantedMainZ = 0;
float ActualRotX  = 0, WantedRotX  = 0;
float ActualRotY  = 0, WantedRotY  = 0;
float ActualRotZ  = 0, WantedRotZ  = 0;

long  Mot1WantedLength=0, Mot1ActualLength=0;
long  Mot2WantedLength=0, Mot2ActualLength=0;
long  Mot3WantedLength=0, Mot3ActualLength=0;
long  Mot4WantedLength=0, Mot4ActualLength=0;
long  Mot5WantedLength=0, Mot5ActualLength=0;
long  Mot6WantedLength=0, Mot6ActualLength=0;
long  Mot7WantedLength=0, Mot7ActualLength=0;
long  Mot8WantedLength=0, Mot8ActualLength=0;

int   Mot1PulseProcent=1, Mot2PulseProcent=1;
int   Mot3PulseProcent=1, Mot4PulseProcent=1;
int   Mot5PulseProcent=1, Mot6PulseProcent=1;
int   Mot7PulseProcent=1, Mot8PulseProcent=1;

bool  Mot1On=1, Mot2On=1, Mot3On=1, Mot4On=1;
bool  Mot5On=1, Mot6On=1, Mot7On=1, Mot8On=1;

bool  Inpos = 1;
bool  go    = false;
float Procent100 = 0;
long  TotalMove[9] = {0};
int   MotorNbr[9]  = {0,1,2,3,4,5,6,7,8};

Coordinate ObjectCorner[9] = {};

// ============================================================
//  Pythagoras
// ============================================================
float Pythagoras(float a, float b, float c)
{
    return sqrtf(a*a + b*b + c*c);
}

// ============================================================
//  Kinematica - exact zelfde als de .cpp bestanden op STM32
// ============================================================
long Mot1(float X, float Y, float Z)
{
    float XCorner = -(LengthFrame / 2.0f);
    float YCorner = -(WidhtFrame  / 2.0f);
    float ZCorner =  (HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot2(float X, float Y, float Z)
{
    float XCorner = -(LengthFrame / 2.0f);
    float YCorner =  (WidhtFrame  / 2.0f);
    float ZCorner =  (HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot3(float X, float Y, float Z)
{
    float XCorner =  (LengthFrame / 2.0f);
    float YCorner = -(WidhtFrame  / 2.0f);
    float ZCorner =  (HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot4(float X, float Y, float Z)
{
    float XCorner =  (LengthFrame / 2.0f);
    float YCorner =  (WidhtFrame  / 2.0f);
    float ZCorner =  (HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot5(float X, float Y, float Z)
{
    float XCorner = -(LengthFrame / 2.0f);
    float YCorner = -(WidhtFrame  / 2.0f);
    float ZCorner = -(HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot6(float X, float Y, float Z)
{
    float XCorner = -(LengthFrame / 2.0f);
    float YCorner =  (WidhtFrame  / 2.0f);
    float ZCorner = -(HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot7(float X, float Y, float Z)
{
    float XCorner =  (LengthFrame / 2.0f);
    float YCorner = -(WidhtFrame  / 2.0f);
    float ZCorner = -(HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

long Mot8(float X, float Y, float Z)
{
    float XCorner =  (LengthFrame / 2.0f);
    float YCorner =  (WidhtFrame  / 2.0f);
    float ZCorner = -(HighFrame   / 2.0f);
    float dx = XCorner-X, dy = YCorner-Y, dz = ZCorner-Z;
    float DB = Pythagoras(dy, dx, dz);
    float BP = DB - DiaKatrol/2.0f;
    float MP = ZCorner - Z;
    float BM = Pythagoras(MP, BP, 0);
    float BK = Pythagoras(BM, DiaKatrol/2.0f, 0);
    float h  = (float)PI - asinf(std::min(1.0f,BK/BM)) - asinf(std::min(1.0f,BP/BM));
    float L  = BK + (DiaKatrol/2.0f)*h;
    return (long)(L * StepUnit + 0.5f);
}

// ============================================================
//  Matrix transformaties (vereenvoudigd voor simulator)
// ============================================================
void matrixTrans(float& X, float& Y, float& Z, float dx, float dy, float dz, int corner)
{
    ObjectCorner[corner].X += dx;
    ObjectCorner[corner].Y += dy;
    ObjectCorner[corner].Z += dz;
}

void matrixRotX(float& X, float& Y, float& Z, float angle, int corner)
{
    float rad = angle * (float)PI / 180.0f;
    float y2 = ObjectCorner[corner].Y * cosf(rad) - ObjectCorner[corner].Z * sinf(rad);
    float z2 = ObjectCorner[corner].Y * sinf(rad) + ObjectCorner[corner].Z * cosf(rad);
    ObjectCorner[corner].Y = y2;
    ObjectCorner[corner].Z = z2;
}

void matrixRotY(float& X, float& Y, float& Z, float angle, int corner)
{
    float rad = angle * (float)PI / 180.0f;
    float x2 =  ObjectCorner[corner].X * cosf(rad) + ObjectCorner[corner].Z * sinf(rad);
    float z2 = -ObjectCorner[corner].X * sinf(rad) + ObjectCorner[corner].Z * cosf(rad);
    ObjectCorner[corner].X = x2;
    ObjectCorner[corner].Z = z2;
}

void matrixRotZ(float& X, float& Y, float& Z, float angle, int corner)
{
    float rad = angle * (float)PI / 180.0f;
    float x2 = ObjectCorner[corner].X * cosf(rad) - ObjectCorner[corner].Y * sinf(rad);
    float y2 = ObjectCorner[corner].X * sinf(rad) + ObjectCorner[corner].Y * cosf(rad);
    ObjectCorner[corner].X = x2;
    ObjectCorner[corner].Y = y2;
}

// ============================================================
//  Init - zelfde als setup() in STM32
// ============================================================
void RobotInit()
{
    ActualMainX = ActualMainY = ActualMainZ = 0;
    WantedMainX = WantedMainY = WantedMainZ = 0;
    ActualRotX = ActualRotY = ActualRotZ = 0;
    WantedRotX = WantedRotY = WantedRotZ = 0;

    for (int i = 1; i <= 8; i++) MotorNbr[i] = i;

    // Hoekpunten initialiseren - zelfde als setup() in zz_Main.cpp
    // Onderste 4 hoeken: LengthBlockBottom x WidthBlockBottom
    ObjectCorner[1].X = -(LengthBlockBottom/2); ObjectCorner[1].Y = -(WidthBlockBottom/2); ObjectCorner[1].Z = -(HighBlock/2);
    ObjectCorner[2].X = -(LengthBlockBottom/2); ObjectCorner[2].Y =  (WidthBlockBottom/2); ObjectCorner[2].Z = -(HighBlock/2);
    ObjectCorner[3].X =  (LengthBlockBottom/2); ObjectCorner[3].Y = -(WidthBlockBottom/2); ObjectCorner[3].Z = -(HighBlock/2);
    ObjectCorner[4].X =  (LengthBlockBottom/2); ObjectCorner[4].Y =  (WidthBlockBottom/2); ObjectCorner[4].Z = -(HighBlock/2);
    // Bovenste 4 hoeken: LengthBlockTop x WidthBlockTop (gedraaid!)
    ObjectCorner[5].X = -(LengthBlockTop/2); ObjectCorner[5].Y = -(WidthBlockTop/2); ObjectCorner[5].Z =  (HighBlock/2);
    ObjectCorner[6].X = -(LengthBlockTop/2); ObjectCorner[6].Y =  (WidthBlockTop/2); ObjectCorner[6].Z =  (HighBlock/2);
    ObjectCorner[7].X =  (LengthBlockTop/2); ObjectCorner[7].Y = -(WidthBlockTop/2); ObjectCorner[7].Z =  (HighBlock/2);
    ObjectCorner[8].X =  (LengthBlockTop/2); ObjectCorner[8].Y =  (WidthBlockTop/2); ObjectCorner[8].Z =  (HighBlock/2);

    Mot1ActualLength = Mot1WantedLength = Mot1(ObjectCorner[1].X, ObjectCorner[1].Y, ObjectCorner[1].Z);
    Mot2ActualLength = Mot2WantedLength = Mot2(ObjectCorner[2].X, ObjectCorner[2].Y, ObjectCorner[2].Z);
    Mot3ActualLength = Mot3WantedLength = Mot3(ObjectCorner[3].X, ObjectCorner[3].Y, ObjectCorner[3].Z);
    Mot4ActualLength = Mot4WantedLength = Mot4(ObjectCorner[4].X, ObjectCorner[4].Y, ObjectCorner[4].Z);
    Mot5ActualLength = Mot5WantedLength = Mot5(ObjectCorner[5].X, ObjectCorner[5].Y, ObjectCorner[5].Z);
    Mot6ActualLength = Mot6WantedLength = Mot6(ObjectCorner[6].X, ObjectCorner[6].Y, ObjectCorner[6].Z);
    Mot7ActualLength = Mot7WantedLength = Mot7(ObjectCorner[7].X, ObjectCorner[7].Y, ObjectCorner[7].Z);
    Mot8ActualLength = Mot8WantedLength = Mot8(ObjectCorner[8].X, ObjectCorner[8].Y, ObjectCorner[8].Z);

    Inpos = 1;
}
