#ifndef MOUSEUPPER_H
#define MOUSEUPPER_H

#include "RobotKinematics.h"

#define MOUSE_ARM_SIZE 5
#define MOUSE_ARM_ARR_SIZE 6


typedef struct MouseUpper
{
	HTmatrix RMW_0;
	DH_entry DH_RMouse[MOUSE_ARM_ARR_SIZE];
	HTmatrix RMouseAdj[MOUSE_ARM_ARR_SIZE];
	HTmatrix RMouseBase[MOUSE_ARM_ARR_SIZE];
	double ** RJ;	double ** RJ_Transpose;

	HTmatrix LMW_0;
	DH_entry DH_LMouse[MOUSE_ARM_ARR_SIZE];
	HTmatrix LMouseAdj[MOUSE_ARM_ARR_SIZE];
	HTmatrix LMouseBase[MOUSE_ARM_ARR_SIZE];
	double ** LJ;	double ** LJ_Transpose;


}MouseUpper;

void MouseInitialize(MouseUpper * Mouse, P3D BaseCoord);
void Run_Mouse(MouseUpper * Mouse, double Rxrot, double Ryrot, double Rzrot, P3D RMouseDes, double Lxrot, double Lyrot, double Lzrot, P3D LMouseDes);

#endif