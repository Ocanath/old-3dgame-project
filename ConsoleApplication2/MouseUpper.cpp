#include "MouseUpper.h"



void MouseInitialize(MouseUpper * Mouse, P3D BaseCoord)
{
	Mouse->RMW_0.H[0][0] = 1.0;		Mouse->RMW_0.H[0][1] = 0;		Mouse->RMW_0.H[0][2] = 0;		Mouse->RMW_0.H[0][3] = 0+BaseCoord.x;
	Mouse->RMW_0.H[1][0] = 0;		Mouse->RMW_0.H[1][1] = 0;		Mouse->RMW_0.H[1][2] = -1.0;	Mouse->RMW_0.H[1][3] = -25.0+BaseCoord.y;
	Mouse->RMW_0.H[2][0] = 0;		Mouse->RMW_0.H[2][1] = 1.0;		Mouse->RMW_0.H[2][2] = 0;		Mouse->RMW_0.H[2][3] = 100.0+BaseCoord.z;
	Mouse->RMW_0.H[3][0] = 0;		Mouse->RMW_0.H[3][1] = 0;		Mouse->RMW_0.H[3][2] = 0;		Mouse->RMW_0.H[3][3] = 1.0;

	Mouse->DH_RMouse[1].d = 14.564;	Mouse->DH_RMouse[1].a = 11.471;	Mouse->DH_RMouse[1].alpha = -3.14159265359 / 2;
	Mouse->DH_RMouse[2].d = 11.246;	Mouse->DH_RMouse[2].a = 10.16;	Mouse->DH_RMouse[2].alpha = 3.14159265359 / 2;
	Mouse->DH_RMouse[3].d = 98.709;	Mouse->DH_RMouse[3].a = 10.59;	Mouse->DH_RMouse[3].alpha = 3.14159265359 / 2;
	Mouse->DH_RMouse[4].d = 7.08;	Mouse->DH_RMouse[4].a = 5.99;	Mouse->DH_RMouse[4].alpha = 3.14159265359 / 2;
	Mouse->DH_RMouse[5].d = 51.135; Mouse->DH_RMouse[5].a = 0;		Mouse->DH_RMouse[5].alpha = 3.14159265359 / 2;
	Mouse->DH_RMouse[1].theta = 1.57079632679; Mouse->DH_RMouse[2].theta = -1.57079632679; Mouse->DH_RMouse[3].theta = 2.57079632679;
	Mouse->DH_RMouse[4].theta = -1.57079632679; Mouse->DH_RMouse[5].theta = 0;

	init_forward_kinematics(Mouse->DH_RMouse, Mouse->RMouseAdj, Mouse->RMouseBase, MOUSE_ARM_SIZE);
	init_jacobian(&(Mouse->RJ), &(Mouse->RJ_Transpose), MOUSE_ARM_SIZE);


	Mouse->LMW_0.H[0][0] = 1.0;		Mouse->LMW_0.H[0][1] = 0;			Mouse->LMW_0.H[0][2] = 0;		Mouse->LMW_0.H[0][3] = 0 + BaseCoord.x;
	Mouse->LMW_0.H[1][0] = 0;		Mouse->LMW_0.H[1][1] = 0;			Mouse->LMW_0.H[1][2] = 1.0;		Mouse->LMW_0.H[1][3] = 25.0 + BaseCoord.y;
	Mouse->LMW_0.H[2][0] = 0;		Mouse->LMW_0.H[2][1] = -1.0;		Mouse->LMW_0.H[2][2] = 0;		Mouse->LMW_0.H[2][3] = 100.0 + BaseCoord.z;
	Mouse->LMW_0.H[3][0] = 0;		Mouse->LMW_0.H[3][1] = 0;			Mouse->LMW_0.H[3][2] = 0;		Mouse->LMW_0.H[3][3] = 1.0;

	Mouse->DH_LMouse[1].d = 14.564;	Mouse->DH_LMouse[1].a = 11.471; Mouse->DH_LMouse[1].alpha = 3.14159265359 / 2;
	Mouse->DH_LMouse[2].d = 11.246;	Mouse->DH_LMouse[2].a = 10.16;		Mouse->DH_LMouse[2].alpha = -3.14159265359 / 2;
	Mouse->DH_LMouse[3].d = 98.709;	Mouse->DH_LMouse[3].a = 10.59;		Mouse->DH_LMouse[3].alpha = -3.14159265359 / 2;
	Mouse->DH_LMouse[4].d = 7.08;		Mouse->DH_LMouse[4].a = 5.99;	Mouse->DH_LMouse[4].alpha = -3.14159265359 / 2;
	Mouse->DH_LMouse[5].d = 51.135;	Mouse->DH_LMouse[5].a = 0;			Mouse->DH_LMouse[5].alpha = -3.14159265359 / 2;
	Mouse->DH_LMouse[1].theta = -1.57079632679; Mouse->DH_LMouse[2].theta = 1.57079632679; Mouse->DH_LMouse[3].theta = -2.57079632679;
	Mouse->DH_LMouse[4].theta = 1.57079632679; Mouse->DH_LMouse[5].theta = 0;

	init_forward_kinematics(Mouse->DH_LMouse, Mouse->LMouseAdj, Mouse->LMouseBase, MOUSE_ARM_SIZE);
	init_jacobian(&(Mouse->LJ), &(Mouse->LJ_Transpose), MOUSE_ARM_SIZE);
	
}


void Run_Mouse(MouseUpper * Mouse, double Rxrot, double Ryrot, double Rzrot, P3D RMouseDes, double Lxrot, double Lyrot, double Lzrot, P3D LMouseDes )
{
	forward_kinematics(Mouse->DH_RMouse, Mouse->RMouseAdj, Mouse->RMouseBase, MOUSE_ARM_SIZE);
	inverse_kinematics(RMouseDes, Mouse->DH_RMouse, Mouse->RJ, Mouse->RJ_Transpose, Mouse->RMouseBase, MOUSE_ARM_SIZE, .000015);
	int i;
	for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		Mouse->DH_RMouse[i].theta += Rxrot * Mouse->RJ_Transpose[i - 1][3] + Ryrot * Mouse->RJ_Transpose[i - 1][4] + Rzrot * Mouse->RJ_Transpose[i - 1][5];

	forward_kinematics(Mouse->DH_LMouse, Mouse->LMouseAdj, Mouse->LMouseBase, MOUSE_ARM_SIZE);
	inverse_kinematics(LMouseDes, Mouse->DH_LMouse, Mouse->LJ, Mouse->LJ_Transpose, Mouse->LMouseBase, MOUSE_ARM_SIZE, .000015);
	for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		Mouse->DH_LMouse[i].theta += Lxrot * Mouse->LJ_Transpose[i - 1][3] + Lyrot *Mouse->LJ_Transpose[i - 1][4] + Lzrot *Mouse->LJ_Transpose[i - 1][5];
}
