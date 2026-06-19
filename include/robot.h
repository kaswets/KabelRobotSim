#pragma once
#include <cmath>
#include <string>
#include <cstdlib>  // abs

// ============================================================
//  Kabel Robot Simulator - exact zelfde variabelen als STM32
//  Gebaseerd op zz_Main.cpp van Arjan Swets
// ============================================================

const double PI = 3.141592653589;

// --- Afmetingen (mm) ---
extern float StepUnit;
extern float StepSpeed;

extern float LengthBlockBottom;
extern float WidthBlockBottom;
extern float HighBlock;
extern float LengthBlockTop;
extern float WidthBlockTop;
extern float LengthFrame;
extern float WidhtFrame;
extern float HighFrame;
extern float DiaKatrol;

// --- Positie ---
extern float ActualMainX, ActualMainY, ActualMainZ;
extern float WantedMainX, WantedMainY, WantedMainZ;
extern float ActualRotX,  WantedRotX;
extern float ActualRotY,  WantedRotY;
extern float ActualRotZ,  WantedRotZ;

// --- Motor state ---
extern long  Mot1WantedLength, Mot1ActualLength;
extern long  Mot2WantedLength, Mot2ActualLength;
extern long  Mot3WantedLength, Mot3ActualLength;
extern long  Mot4WantedLength, Mot4ActualLength;
extern long  Mot5WantedLength, Mot5ActualLength;
extern long  Mot6WantedLength, Mot6ActualLength;
extern long  Mot7WantedLength, Mot7ActualLength;
extern long  Mot8WantedLength, Mot8ActualLength;

extern int   Mot1PulseProcent, Mot2PulseProcent;
extern int   Mot3PulseProcent, Mot4PulseProcent;
extern int   Mot5PulseProcent, Mot6PulseProcent;
extern int   Mot7PulseProcent, Mot8PulseProcent;

extern bool  Mot1On, Mot2On, Mot3On, Mot4On;
extern bool  Mot5On, Mot6On, Mot7On, Mot8On;

extern bool  Inpos;
extern bool  go;
extern float Procent100;
extern long  TotalMove[9];
extern int   MotorNbr[9];

// --- Hoekpunten platform ---
struct Coordinate { float X, Y, Z; };
extern Coordinate ObjectCorner[9];

// --- Functies ---
float  Pythagoras(float a, float b, float c);
long   Mot1(float X, float Y, float Z);
long   Mot2(float X, float Y, float Z);
long   Mot3(float X, float Y, float Z);
long   Mot4(float X, float Y, float Z);
long   Mot5(float X, float Y, float Z);
long   Mot6(float X, float Y, float Z);
long   Mot7(float X, float Y, float Z);
long   Mot8(float X, float Y, float Z);

void   matrixTrans(float& X, float& Y, float& Z, float dx, float dy, float dz, int corner);
void   matrixRotX (float& X, float& Y, float& Z, float angle, int corner);
void   matrixRotY (float& X, float& Y, float& Z, float angle, int corner);
void   matrixRotZ (float& X, float& Y, float& Z, float angle, int corner);

void   ParseCommand(const std::string& cmd);
void   RobotInit();
void   RobotLoop();   // één iteratie van de main loop
