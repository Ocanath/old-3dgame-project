	//#include <SDL.h> 

#include <iostream>
#include <string>
#include <fstream>

#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2\opencv.hpp>

#include "Configurations.h"
#include "GameState.h"
#include "display_game_functions.h"

#include <stdint.h>


#include "RobotKinematics.h"
#include "MouseUpper.h"


#include "Serial.h"

//0 for the prettier, virtual mouse, and 1 for da real mouse 
#define MOUSE_MODE 1

#define TR_CONST 50000

#define MOUSE_ARM_SIZE 5
#define MOUSE_ARM_ARR_SIZE 6

#define TWITCHY_LEG_SIZE 3
#define TWITCHY_LEG_ARR_SIZE 4
#define	TWITCHY_NUM_LEGS 6
#define TWITCHY_ALL_LEGS_ARR_SIZE 8

#define AVICS_NUM_DOFS 6
#define AVICS_ARR_SIZE 7

#define PI  3.14159265359

using namespace std;
using namespace cv;





int strings_match(unsigned char * inbuf, unsigned char * refbuf)
{
	int i;
	for (i = 0; inbuf[i] != ' '; i++)
	{
		if (inbuf[i] != refbuf[i])
			return 0;
	}
	return 1;
}

void PrintHTmatrix(HTmatrix H)
{
	int r; int c;
	for (r = 0; r < 4; r++)
	{
		for (c = 0; c < 4; c++)
		{
			cout << H.H[r][c] << " ";
		}
		cout << endl;
	}
}

DH_entry DH_LMouse2[MOUSE_ARM_ARR_SIZE];
HTmatrix LMouseAdj2[MOUSE_ARM_ARR_SIZE];
HTmatrix LMouseBase2[MOUSE_ARM_ARR_SIZE];
double ** LJ2;	double ** LJ_Transpose2;


void initLMouse(KinematicChain * LM)
{
	//Left Mouse Arm Initialization
	//Mouse Base to 0
	LM->DH_Table = DH_LMouse2;	//can also be malloc
	//forward K setup, DH table
	LM->DH_Table[1].d = 14.564;			LM->DH_Table[1].a = -11.471;			LM->DH_Table[1].alpha = PI / 2;	
	LM->DH_Table[2].d = 11.246;			LM->DH_Table[2].a = -10.16;			LM->DH_Table[2].alpha = PI / 2;
	LM->DH_Table[3].d = 98.709;			LM->DH_Table[3].a = -10.59;			LM->DH_Table[3].alpha = PI / 2;	
	LM->DH_Table[4].d = 7.08;			LM->DH_Table[4].a = -5.99;			LM->DH_Table[4].alpha = PI / 2;		
	LM->DH_Table[5].d = 146.469;			LM->DH_Table[5].a = 0;				LM->DH_Table[5].alpha = -PI / 2;		
	LM->DH_Table[1].theta = -PI / 2;		LM->DH_Table[2].theta = -PI / 2;	LM->DH_Table[3].theta = PI / 2;
	LM->DH_Table[4].theta = -PI / 2;		LM->DH_Table[5].theta = 0;
	LM->HTadj = LMouseAdj2;
	LM->HTbase = LMouseBase2;
	LM->J = LJ2;
	LM->J_Transpose = LJ2;
	LM->array_size = MOUSE_ARM_ARR_SIZE;
	LM->size = MOUSE_ARM_SIZE;
	 
	//	init_forward_kinematics(LM->DH_Table, LM->HTadj, LM->HTbase, MOUSE_ARM_SIZE);
	init_forward_kinematics_KC(LM);
	init_jacobian(&LM->J, &LM->J_Transpose, MOUSE_ARM_SIZE);
}



int main(int argc, char** argv)
{

	int i;

	Player player;
	player.boundary.x = 0.0;
	player.boundary.y = 0.0;
	player.boundary.z = 0.0;
	player.boundary.R[0][0] = 1; player.boundary.R[0][1] = 0; player.boundary.R[0][2] = 0;
	player.boundary.R[1][0] = 0; player.boundary.R[1][1] = 1; player.boundary.R[1][2] = 0;
	player.boundary.R[2][0] = 0; player.boundary.R[2][1] = 0; player.boundary.R[2][2] = 1;
	player.lamda = .001;
		

	Player camAViCS;
	camAViCS.lamda = .001;
	

	Box3d box_x;
	box_x.x = 50.0;
	box_x.y = 50.0;
	box_x.z = 0.0;
	box_x.width = 10.0;
	box_x.height = 10.0;
	box_x.depth = 10.0;


	Box3d boxes[15];
	boxes[0].x = 50.0;
	boxes[0].y = 0.0;
	boxes[0].z = 0.0;
	boxes[0].width = 10.0;
	boxes[0].height = 10.0;
	boxes[0].depth = 10.0;

	boxes[1].x = -50.0;
	boxes[1].y = 0.0;
	boxes[1].z = 0.0;
	boxes[1].width = 10.0;
	boxes[1].height = 10.0;
	boxes[1].depth = 10.0;




	boxes[2].x = 0.0;
	boxes[2].y = 50.0;
	boxes[2].z = 0.0;
	boxes[2].width = 10.0;
	boxes[2].height = 10.0;
	boxes[2].depth = 10.0;

	boxes[3].x = 0.0;
	boxes[3].y = -50.0;
	boxes[3].z = 0.0;
	boxes[3].width = 10.0;
	boxes[3].height = 10.0;
	boxes[3].depth = 10.0;

	boxes[4].x = 0.0;
	boxes[4].y = 0.0;
	boxes[4].z = 50.0;
	boxes[4].width = 10.0;
	boxes[4].height = 10.0;
	boxes[4].depth = 10.0;


	int num_bloks = 5;




	double theta1 = 1.57079632679 * 2;
	double theta2 = -1.57079632679;
	double theta3 = -1.57079632679;
	//double theta2 = 0.0;
	update_R(player.boundary.R, theta1, theta2, theta3);


	Mat frame;
	frame = Mat::zeros(Size(640, 480), CV_8UC3);

	VideoWriter video;
	video.open("last_session.avi", CV_FOURCC('M', 'J', 'P', 'G'), 60, Size(frame.cols, frame.rows), true);

	Mat AViCS_ViewPort;
	AViCS_ViewPort = Mat::zeros(Size(640, 480), CV_8UC3);


	ofstream rvt;
	rvt.open("Rvtext.txt");
	rvt.clear();

	ofstream lvt;
	lvt.open("Lvtext.txt");
	lvt.clear();


	//player.boundary.z = -150.0;
	int xoff = 0;
	int yoff = 0;
	int zoff = 0;
	//bool lock_on_the_block = 0;
	player.boundary.vx = 0;
	player.boundary.vy = 0;
	player.boundary.vz = 0;

	double two_link_theta1 = 0.0;
	double two_link_theta2 = -1.57079632679;
	double a1 = 10.0;
	double a2 = 10.0;

	/////////////////////////////////////////////////////////////////////////////////////////////////
	theta2 = 2 * 1.57079632679;			//this angle is lookin down. get ridofit
	/////////////////////////////////////////////////////////////////////////////////////////////////
	double x_des = 5;
	double y_des = 5;
	double ef_v_weight = .1;




	HTmatrix HTrot_x_neg_90;
	double xrot_val = -PI / 2;
	HTrot_x_neg_90.H[0][0] = 1; HTrot_x_neg_90.H[0][1] = 0;				HTrot_x_neg_90.H[0][2] = 0;					HTrot_x_neg_90.H[0][3] = 0;
	HTrot_x_neg_90.H[1][0] = 0; HTrot_x_neg_90.H[1][1] = 0;				HTrot_x_neg_90.H[1][2] = 1;					HTrot_x_neg_90.H[1][3] = 0;
	HTrot_x_neg_90.H[2][0] = 0; HTrot_x_neg_90.H[2][1] = -1;			HTrot_x_neg_90.H[2][2] = 0;					HTrot_x_neg_90.H[2][3] = 0;
	HTrot_x_neg_90.H[3][0] = 0; HTrot_x_neg_90.H[3][1] = 0;				HTrot_x_neg_90.H[3][2] = 0;					HTrot_x_neg_90.H[3][3] = 1;
	HTmatrix HTrot_x_pos_90;
	xrot_val = PI / 2;
	HTrot_x_pos_90.H[0][0] = 1; HTrot_x_pos_90.H[0][1] = 0;				HTrot_x_pos_90.H[0][2] = 0;					HTrot_x_pos_90.H[0][3] = 0;
	HTrot_x_pos_90.H[1][0] = 0; HTrot_x_pos_90.H[1][1] = 0;				HTrot_x_pos_90.H[1][2] = -1;				HTrot_x_pos_90.H[1][3] = 0;
	HTrot_x_pos_90.H[2][0] = 0; HTrot_x_pos_90.H[2][1] = 1;				HTrot_x_pos_90.H[2][2] = 0;					HTrot_x_pos_90.H[2][3] = 0;
	HTrot_x_pos_90.H[3][0] = 0; HTrot_x_pos_90.H[3][1] = 0;				HTrot_x_pos_90.H[3][2] = 0;					HTrot_x_pos_90.H[3][3] = 1;
	HTmatrix Identity_H;
	Identity_H.H[0][0] = 1.0;	Identity_H.H[0][1] = 0;			Identity_H.H[0][2] = 0;		Identity_H.H[0][3] = 0;
	Identity_H.H[1][0] = 0;		Identity_H.H[1][1] = 1.0;		Identity_H.H[1][2] = 0;		Identity_H.H[1][3] = 0;
	Identity_H.H[2][0] = 0;		Identity_H.H[2][1] = 0;			Identity_H.H[2][2] = 1.0;	Identity_H.H[2][3] = 0;
	Identity_H.H[3][0] = 0;		Identity_H.H[3][1] = 0;			Identity_H.H[3][2] = 0;		Identity_H.H[3][3] = 1.0;


	




	
	//begin initialization of twitchy
	DH_entry DH_Twitchy[TWITCHY_ALL_LEGS_ARR_SIZE][TWITCHY_LEG_ARR_SIZE];
	HTmatrix TWAdj[TWITCHY_ALL_LEGS_ARR_SIZE][TWITCHY_LEG_ARR_SIZE];
	HTmatrix TWZero[TWITCHY_ALL_LEGS_ARR_SIZE][TWITCHY_LEG_ARR_SIZE];
	P3D TWFootCoord[TWITCHY_ALL_LEGS_ARR_SIZE];

	double ** TJ[TWITCHY_ALL_LEGS_ARR_SIZE];	double ** TJ_Transpose[TWITCHY_ALL_LEGS_ARR_SIZE];
	for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
	{
		DH_Twitchy[i][1].d = -55.095;	DH_Twitchy[i][1].a = -20.32;	DH_Twitchy[i][1].alpha = -1.57079632679;
		DH_Twitchy[i][2].d = 5.505;		DH_Twitchy[i][2].a = 95.482;	DH_Twitchy[i][2].alpha = 0;
		DH_Twitchy[i][3].d = -5.5;		DH_Twitchy[i][3].a = 154.938;	DH_Twitchy[i][3].alpha = 0;
		DH_Twitchy[i][1].theta = 3.14159265359;	DH_Twitchy[i][2].theta = 3.14159265359;
		DH_Twitchy[i][3].theta = -1.57079632679;
		init_forward_kinematics(DH_Twitchy[i], TWAdj[i], TWZero[i], TWITCHY_LEG_SIZE);
		init_jacobian(&(TJ[i]), &(TJ_Transpose[i]), TWITCHY_LEG_SIZE);
	}
	//world to central base frame
	HTmatrix dev_TW_B;
	dev_TW_B.H[0][0] = 1.0;		dev_TW_B.H[0][1] = 0;		dev_TW_B.H[0][2] = 0;		dev_TW_B.H[0][3] = -15;
	dev_TW_B.H[1][0] = 0;		dev_TW_B.H[1][1] = 1.0;		dev_TW_B.H[1][2] = 0;		dev_TW_B.H[1][3] = -150;
	dev_TW_B.H[2][0] = 0;		dev_TW_B.H[2][1] = 0;		dev_TW_B.H[2][2] = 1.0;		dev_TW_B.H[2][3] = 0;
	dev_TW_B.H[3][0] = 0;		dev_TW_B.H[3][1] = 0;		dev_TW_B.H[3][2] = 0;		dev_TW_B.H[3][3] = 1.0;
	//Establish the zero frames in the base frame
	//78.047 is the normal radius
	HTmatrix dev_TTr_0;
	dev_TTr_0.H[0][0] = 1.0;	dev_TTr_0.H[0][1] = 0;		dev_TTr_0.H[0][2] = 0;		dev_TTr_0.H[0][3] = 78.047;
	dev_TTr_0.H[1][0] = 0;		dev_TTr_0.H[1][1] = 1.0;	dev_TTr_0.H[1][2] = 0;		dev_TTr_0.H[1][3] = 0;
	dev_TTr_0.H[2][0] = 0;		dev_TTr_0.H[2][1] = 0;		dev_TTr_0.H[2][2] = 1.0;	dev_TTr_0.H[2][3] = 0;
	dev_TTr_0.H[3][0] = 0;		dev_TTr_0.H[3][1] = 0;		dev_TTr_0.H[3][2] = 0;		dev_TTr_0.H[3][3] = 1.0;
	double dev_Base_to_0_angle = 0.0;
	HTmatrix dev_TB_0[TWITCHY_ALL_LEGS_ARR_SIZE];
	HTmatrix dev_T0_B[TWITCHY_ALL_LEGS_ARR_SIZE];
	for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
	{
		dev_TB_0[i].H[0][0] = cos(dev_Base_to_0_angle);		dev_TB_0[i].H[0][1] = -sin(dev_Base_to_0_angle);	dev_TB_0[i].H[0][2] = 0;		dev_TB_0[i].H[0][3] = 0;
		dev_TB_0[i].H[1][0] = sin(dev_Base_to_0_angle);		dev_TB_0[i].H[1][1] = cos(dev_Base_to_0_angle);		dev_TB_0[i].H[1][2] = 0;		dev_TB_0[i].H[1][3] = 0;
		dev_TB_0[i].H[2][0] = 0;						dev_TB_0[i].H[2][1] = 0;						dev_TB_0[i].H[2][2] = 1.0;		dev_TB_0[i].H[2][3] = 0;
		dev_TB_0[i].H[3][0] = 0;						dev_TB_0[i].H[3][1] = 0;						dev_TB_0[i].H[3][2] = 0;		dev_TB_0[i].H[3][3] = 1.0;
		HT_Multiply(dev_TB_0[i], dev_TTr_0, &(dev_TB_0[i]));
		HT_Inverse(dev_TB_0[i], &(dev_T0_B[i]));
		dev_Base_to_0_angle += 3.14159265359 / 3;
	}
	HTmatrix dev_TW_0[TWITCHY_ALL_LEGS_ARR_SIZE];
	for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
		HT_Multiply(dev_TW_B, dev_TB_0[i], &(dev_TW_0[i]));
	//end twitchy initialization





	//Begin Mouse Initialization
	
	//Matrix for ALL Mouse
	HTmatrix HMW_B;
	copy_HT(&HMW_B, &Identity_H);

	//Left Mouse Arm Initialization
	//Mouse Base to 0
	HTmatrix LMB_0;
	copy_HT(&LMB_0, &Identity_H);
	HT_Multiply(HTrot_x_neg_90, LMB_0, &LMB_0);
	LMB_0.H[1][3] = 86.18;
	HTmatrix LMW_0;
	HT_Multiply(HMW_B, LMB_0, &LMW_0);
	//forward K setup, DH table
	DH_entry DH_LMouse[MOUSE_ARM_ARR_SIZE];
	DH_LMouse[1].d = 14.564;	DH_LMouse[1].a = -11.471;		DH_LMouse[1].alpha = PI / 2;	
	DH_LMouse[2].d = 11.246;	DH_LMouse[2].a = -10.16;		DH_LMouse[2].alpha = PI / 2;		
	DH_LMouse[3].d = 98.709;	DH_LMouse[3].a = -10.59;		DH_LMouse[3].alpha = PI / 2;	
	DH_LMouse[4].d = 7.08;		DH_LMouse[4].a = -5.99;			DH_LMouse[4].alpha = PI / 2;		
	DH_LMouse[5].d = 146.469;	DH_LMouse[5].a = 0;				DH_LMouse[5].alpha = -PI / 2;		
	DH_LMouse[1].theta = -PI / 2;	DH_LMouse[2].theta = -PI / 2;	DH_LMouse[3].theta = PI / 2;
	DH_LMouse[4].theta = -PI / 2;	DH_LMouse[5].theta = 0;
	HTmatrix LMouseAdj[MOUSE_ARM_ARR_SIZE];
	HTmatrix LMouseBase[MOUSE_ARM_ARR_SIZE];
	init_forward_kinematics(DH_LMouse, LMouseAdj, LMouseBase, MOUSE_ARM_SIZE);
	double ** LJ;	double ** LJ_Transpose;
	init_jacobian(&LJ, &LJ_Transpose, MOUSE_ARM_SIZE);
	P3D LMouseDes;
	LMouseDes.x = 50; LMouseDes.y = 0; LMouseDes.z = 50;
	P3D LM_errDeriv;	P3D LM_errIntegral;
	LM_errDeriv.x = 0;	LM_errDeriv.y = 0;	LM_errDeriv.z = 0;
	LM_errIntegral.x = 0;	LM_errIntegral.y = 0;	LM_errIntegral.z = 0;
	P3D LMprev_err;
	LMprev_err.x = 0;	LMprev_err.y = 0;	LMprev_err.z = 0;


	//Right Mouse Initialization
	HTmatrix RMB_0;
	copy_HT(&RMB_0, &Identity_H);
	HT_Multiply(HTrot_x_pos_90, RMB_0, &RMB_0);
	RMB_0.H[1][3] = -86.18;
	HTmatrix RMW_0;
	HT_Multiply(HMW_B, RMB_0, &RMW_0);
	// forward K setup, DH table
	DH_entry DH_RMouse[MOUSE_ARM_ARR_SIZE];
	DH_RMouse[1].d = 14.564;	DH_RMouse[1].a = -11.471;	DH_RMouse[1].alpha = -PI / 2;
	DH_RMouse[2].d = 11.246;	DH_RMouse[2].a = -10.16;	DH_RMouse[2].alpha = -PI / 2;
	DH_RMouse[3].d = 98.709;	DH_RMouse[3].a = -10.59;	DH_RMouse[3].alpha = -PI / 2;
	DH_RMouse[4].d = 7.08;		DH_RMouse[4].a = -5.99;		DH_RMouse[4].alpha = -PI / 2;
	DH_RMouse[5].d = 146.469;	DH_RMouse[5].a = 0;			DH_RMouse[5].alpha = PI / 2;
	DH_RMouse[1].theta = PI/2; DH_RMouse[2].theta = PI/2; DH_RMouse[3].theta = -PI/2;
	DH_RMouse[4].theta = PI/2; DH_RMouse[5].theta = 0;
	HTmatrix RMouseAdj[MOUSE_ARM_ARR_SIZE];
	HTmatrix RMouseBase[MOUSE_ARM_ARR_SIZE];
	init_forward_kinematics(DH_RMouse, RMouseAdj, RMouseBase, MOUSE_ARM_SIZE);
	double ** RJ;	double ** RJ_Transpose;
	init_jacobian(&RJ, &RJ_Transpose, MOUSE_ARM_SIZE);
	P3D RMouseDes;
	RMouseDes.x = 50; RMouseDes.y = 0; RMouseDes.z = 50;
	//End mouse initialization


	//Mouse Body and head initialization!
	 

	MouseUpper M1;
	P3D WorldOffset;
	WorldOffset.x = 0; WorldOffset.y = 100; WorldOffset.z = 0;
	MouseInitialize(&M1, WorldOffset);


	HTmatrix AViCS_HW_B;	HTmatrix AVIiCS_HB_0;
	AViCS_HW_B.H[0][0] = 1;	AViCS_HW_B.H[0][1] = 0; AViCS_HW_B.H[0][2] = 0; AViCS_HW_B.H[0][3] = 200;
	AViCS_HW_B.H[1][0] = 0; AViCS_HW_B.H[1][1] = 1; AViCS_HW_B.H[1][2] = 0; AViCS_HW_B.H[1][3] = 200;
	AViCS_HW_B.H[2][0] = 0; AViCS_HW_B.H[2][1] = 0; AViCS_HW_B.H[2][2] = 1; AViCS_HW_B.H[2][3] = -200;
	AViCS_HW_B.H[3][0] = 0;	AViCS_HW_B.H[3][1] = 0; AViCS_HW_B.H[3][2] = 0; AViCS_HW_B.H[3][3] = 1;
	
	AVIiCS_HB_0.H[0][0] = 1; AVIiCS_HB_0.H[0][1] = 0; AVIiCS_HB_0.H[0][2] = 0; AVIiCS_HB_0.H[0][3] = 0;
	AVIiCS_HB_0.H[1][0] = 0; AVIiCS_HB_0.H[1][1] = 1; AVIiCS_HB_0.H[1][2] = 0; AVIiCS_HB_0.H[1][3] = 0;
	AVIiCS_HB_0.H[2][0] = 0; AVIiCS_HB_0.H[2][1] = 0; AVIiCS_HB_0.H[2][2] = 1; AVIiCS_HB_0.H[2][3] = 0;
	AVIiCS_HB_0.H[3][0] = 0; AVIiCS_HB_0.H[3][1] = 0; AVIiCS_HB_0.H[3][2] = 0; AVIiCS_HB_0.H[3][3] = 1;

	DH_entry DH_AViCS[AVICS_ARR_SIZE];
	DH_AViCS[1].d = 21.01;	DH_AViCS[1].a = 0;			DH_AViCS[1].alpha = 3.14159265359 / 2;
	DH_AViCS[2].d = 0;		DH_AViCS[2].a = 88.269;		DH_AViCS[2].alpha = 0;
	DH_AViCS[3].d = 0;		DH_AViCS[3].a = 0.779;		DH_AViCS[3].alpha = -3.14159265359 / 2;
	DH_AViCS[4].d = 73.976;	DH_AViCS[4].a = 0;			DH_AViCS[4].alpha = 3.14159265359 / 2;
	DH_AViCS[5].d = 0;		DH_AViCS[5].a = 0;			DH_AViCS[5].alpha = -3.14159265359 / 2;
	DH_AViCS[6].d = 50.13;	DH_AViCS[6].a = 0;			DH_AViCS[6].alpha = 3.14159265359 / 2;
	double pi_v = 3.14159265359;
	DH_AViCS[1].theta = 0;	DH_AViCS[2].theta = pi_v/2;	DH_AViCS[3].theta = 0;	
	DH_AViCS[4].theta = 0;	DH_AViCS[5].theta = 0;	DH_AViCS[6].theta = -pi_v/2;
	HTmatrix AViCS_Hadj[AVICS_ARR_SIZE];	HTmatrix AViCS_Hzero[AVICS_ARR_SIZE];
	init_forward_kinematics(DH_AViCS, AViCS_Hadj, AViCS_Hzero, AVICS_NUM_DOFS);
	double ** AViCS_J; double ** AViCS_J_Transpose; 
	init_jacobian(&AViCS_J, &AViCS_J_Transpose, AVICS_NUM_DOFS);
	P3D AViCS_des;
	AViCS_des.x = AViCS_Hzero[6].H[0][3]; AViCS_des.y = AViCS_Hzero[6].H[1][3]; AViCS_des.z = AViCS_Hzero[6].H[2][3];
	AViCS_des.x = -116.280586; AViCS_des.y = 0.038099; AViCS_des.z = 10.592713;
	DH_AViCS[1].theta = -0.000403;  DH_AViCS[2].theta = 2.148064;  DH_AViCS[3].theta = 0.734308;
	DH_AViCS[4].theta = -0.000202;  DH_AViCS[5].theta = -1.047637;  DH_AViCS[6].theta = -1.570796;

	double AViCS_Xrot = 0; double AViCS_Yrot = 0;	double AViCS_Zrot = 0;
	int AViCS_LinkNum = 1;




	
	
	KinematicChain Thermo;
	//inti stuff
	Thermo.array_size = 6;
	Thermo.size = 5;
	//table
	DH_entry DH_Thermo[6];
	Thermo.DH_Table = DH_Thermo;
	Thermo.DH_Table[1].d = 10;		Thermo.DH_Table[1].a = 0;		Thermo.DH_Table[1].alpha = -PI / 2;
	Thermo.DH_Table[2].d = 0;		Thermo.DH_Table[2].a = 10;		Thermo.DH_Table[2].alpha = 0;
	Thermo.DH_Table[3].d = 0;		Thermo.DH_Table[3].a = 10;		Thermo.DH_Table[3].alpha = 0;
	Thermo.DH_Table[4].d = 0;		Thermo.DH_Table[4].a = 0;		Thermo.DH_Table[4].alpha = PI / 2;
	Thermo.DH_Table[5].d = 5;		Thermo.DH_Table[5].a = 0;		Thermo.DH_Table[5].alpha = 0;
	Thermo.DH_Table[1].theta = 0;	Thermo.DH_Table[2].theta = -PI/2;	Thermo.DH_Table[3].theta = PI/2;
	Thermo.DH_Table[4].theta = 0;	Thermo.DH_Table[5].theta = 0;
	HTmatrix ThermoAdj[6];
	HTmatrix ThermoBase[6];
	Thermo.HTbase = ThermoBase;
	Thermo.HTadj = ThermoAdj;
	//init pos and orientation
	Thermo.HB_0 = Identity_H;
	Thermo.HW_B = Identity_H;
	init_forward_kinematics_KC(&Thermo);
	Thermo.HW_B = Hscale(1.0);
	Thermo.HW_B.H[0][3] = -150;
	Thermo.HW_B.H[1][3] = 150;
	Thermo.HB_0 = Hscale(1.0);

	//init_jacobian(&(Thermo.J), &(Thermo.J_Transpose), Thermo.size);



	/*
	double I[4][4];
	I[0][0] = 1; I[0][1] = 0; I[0][2] = 0; I[0][3] = 0;
	I[1][0] = 0; I[1][1] = 1; I[1][2] = 0; I[1][3] = 0;
	I[2][0] = 0; I[2][1] = 0; I[2][2] = 1; I[2][3] = 0;
	I[3][0] = 0; I[3][1] = 0; I[3][2] = 0; I[3][3] = 1;

	double HOUT[4][4];
	Multiply_HT_Matrices(H0_1, I, HOUT);
	*/

	double T_xpos[7];	double T_ypos[7];	double T_zpos[7];
	double T_xOrient[7]; double T_yOrient[7]; double T_zOrient[7];

	for (i = 1; i <= 6; i++)
	{
		T_xpos[i] = 100.0; T_ypos[i] = 0.0; T_zpos[i] = -200.0;
		T_xOrient[i] = 0.0; T_yOrient[i] = 0.0; T_zOrient[i] = 0.0;
	}



	int T_Moving_Leg_Num = 1;

	
	theta1 = 0.3915; theta2 = 4.641;
	//player.boundary.x = 45.179; player.boundary.y = 19.34; 
	player.boundary.x = 458.388; player.boundary.y = 277.092;



	unsigned int time_step = 0;
	double t = 0;

	int Lm_linknum = 1;
	int Rm_linknum = 1;



	LMouseDes.x = LMouseBase[5].H[0][3]; LMouseDes.y = LMouseBase[5].H[1][3]; LMouseDes.z = LMouseBase[5].H[2][3];
	RMouseDes.x = RMouseBase[5].H[0][3]; RMouseDes.y = RMouseBase[5].H[1][3]; RMouseDes.z = RMouseBase[5].H[2][3];
	P3D Lc;
	HT_Point_Multiply(LMB_0, LMouseDes, &Lc);


	Pyramid pyrTest;
	//base frame vertex coordinates :)
	P3D V1; P3D V2; P3D V3; P3D V4;
	V1.x = 0; V1.y = 5; V1.z = -5;
	V2.x = -5; V2.y = -5; V2.z = -5;
	V3.x = 5; V3.y = -5; V3.z = -5;
	V4.x = 0; V4.y = 0; V4.z = 5;
	initialize_pyramid(&pyrTest, V1, V2, V3, V4);

	Pyramid MouseBodyPyramid;
	V2.x = LMB_0.H[0][3]; V2.y = LMB_0.H[1][3]; V2.z = LMB_0.H[2][3];
	V3.x = RMB_0.H[0][3]; V3.y = RMB_0.H[1][3]; V3.z = RMB_0.H[2][3];
	V4.x = -50;	V4.y = 0; V4.z = 10;
	V1.x = 0;	V1.y = 0; V1.z = -120;
	initialize_pyramid(&MouseBodyPyramid, V1, V2, V3, V4);
	copy_HT(&(MouseBodyPyramid.HW_B), &HMW_B);
	

	//start with playerC_W
	//take every W coordinate and dump into a list.
	//have a matched list that maps 
	//Take list and convert to player coordinate, and assign a distance from the player
	//have sorted list of z-distance, and a mapping of indexes to groups of 3 (triangles)
	//so for 1 triangle, it would be like:
	//zbuf[0] = 
	//trimap[0] = {0, 2, 3}

	ifstream con_file;
	con_file.open("saved_sess_1.txt");
	char endFlag = 0;
	char stop = 0;
	


	KinematicChain LM;
	initLMouse(&LM);

	DH_entry cam_wrist[4];
	cam_wrist[1].d = 100;		cam_wrist[1].a = 0;	cam_wrist[1].alpha = PI/2;
	cam_wrist[2].d = 0;			cam_wrist[2].a = 0;	cam_wrist[2].alpha = -PI / 2;
	cam_wrist[3].d = 100;		cam_wrist[3].a = 0;	cam_wrist[3].alpha = 0;
	

	cam_wrist[1].theta = 0;
	cam_wrist[2].theta = -PI/2;
	cam_wrist[3].theta = 0;
	

	HTmatrix CHadj[4];
	HTmatrix CHbase[4];
	//init fk
	init_forward_kinematics(cam_wrist, CHadj, CHbase, 3);


	while (stop == 0)
	{
		time_step++;
		t += .001;

		frame = Mat::zeros(Size(640, 480), CV_8UC3);
		AViCS_ViewPort = Mat::zeros(Size(640, 480), CV_8UC3);

		double tu1, tv1, tu2, tv2;


		int key_press = waitKey(1);

		unsigned char legcoordflag = 0;
		if (key_press == '/')
			legcoordflag = !legcoordflag;


		//if (time_step > 1000)
		//{
		//	P3D RMWc;
		//	RMWc.x = 50.0; RMWc.y = 0.0; RMWc.z = 50.0;
		//	HTmatrix RM0_W;
		//	HT_Inverse(RMW_0, &RM0_W);
		//	HT_Point_Multiply(RM0_W, RMWc, &RMouseDes);
		//	P3D LMWc;
		//	LMWc.x = 50.0; LMWc.y = 0.0; LMWc.z = 50.0;
		//	HTmatrix LM0_W;
		//	HT_Inverse(LMW_0, &LM0_W);
		//	HT_Point_Multiply(LM0_W, LMWc, &LMouseDes);
		//}

		//double scale_val = 1;
		//if (key_press == '<')
		//	scale_val = 1.01;
		//else if (key_press == '>')
		//	scale_val = .99;
		//HTmatrix Hscale;
		//Hscale.H[0][0] = scale_val;					Hscale.H[0][1] = 0;						Hscale.H[0][2] = 0;							Hscale.H[0][3] = 0;
		//Hscale.H[1][0] = 0;							Hscale.H[1][1] = scale_val;				Hscale.H[1][2] = 0;							Hscale.H[1][3] = 0;
		//Hscale.H[2][0] = 0;							Hscale.H[2][1] = 0;						Hscale.H[2][2] = scale_val;					Hscale.H[2][3] = 0;
		//Hscale.H[3][0] = 0;							Hscale.H[3][1] = 0;						Hscale.H[3][2] = 0;							Hscale.H[3][3] = scale_val;
		//HT_Multiply(Hscale, HMW_B, &HMW_B);


		HTmatrix BaseXrot;
		double angl = .01;
		BaseXrot.H[0][0] = 1; BaseXrot.H[0][1] = 0;				BaseXrot.H[0][2] = 0;					BaseXrot.H[0][3] = 0;
		BaseXrot.H[1][0] = 0; BaseXrot.H[1][1] = cos(angl);	BaseXrot.H[1][2] = -sin(angl);	BaseXrot.H[1][3] = 0;
		BaseXrot.H[2][0] = 0; BaseXrot.H[2][1] = sin(angl);	BaseXrot.H[2][2] = cos(angl);		BaseXrot.H[2][3] = 0;
		BaseXrot.H[3][0] = 0; BaseXrot.H[3][1] = 0;				BaseXrot.H[3][2] = 0;					BaseXrot.H[3][3] = 1;
		HTmatrix BaseZrot;
		angl = .01;
		BaseZrot.H[0][0] = cos(angl);	BaseZrot.H[0][1] = -sin(angl);				BaseZrot.H[0][2] = 0;					BaseZrot.H[0][3] = 0;
		BaseZrot.H[1][0] = sin(angl);	BaseZrot.H[1][1] = cos(angl);				BaseZrot.H[1][2] = 0;					BaseZrot.H[1][3] = 0;
		BaseZrot.H[2][0] = 0;			BaseZrot.H[2][1] = 0;						BaseZrot.H[2][2] = 1;					BaseZrot.H[2][3] = 0;
		BaseZrot.H[3][0] = 0;			BaseZrot.H[3][1] = 0;						BaseZrot.H[3][2] = 0;					BaseZrot.H[3][3] = 1;
		HTmatrix BaseCompRot;
		HT_Multiply(BaseXrot, BaseZrot, &BaseCompRot);
		HT_Multiply(pyrTest.HW_B, BaseCompRot, &(pyrTest.HW_B));
		pyrTest.HW_B.H[2][3] = 65 + 5 * sin(40 * t);
		update_pyramid(&pyrTest, &frame, &player, 255, 255, 255);


		if (time_step % 3 == 0)
		{
			if (endFlag == 0)
			{
				int txtDofNum = 1;
				//double read_DOF[MOUSE_ARM_ARR_SIZE];
				for (i = 0; i < 15; i++)
				{
					string fl;
					con_file >> fl;
					if (fl == "STOP")
						endFlag = 1;
					if ((i + 1) % 3 == 0 && endFlag == 0)
					{
						fl.erase(fl.size() - 1, fl.size() - 1);
						//read_DOF[txtDofNum] = stof(fl);
						DH_LMouse[txtDofNum].theta = stof(fl);
						txtDofNum++;
					}
				}
			}
		}

		if (key_press == 'I')
			Lc.z += 10;
		else if (key_press == 'K')
			Lc.z -= 10;
		else if (key_press == 'J')
			Lc.y += 10;
		else if (key_press == 'L')
			Lc.y -= 10;
		else if (key_press == 'O')
			Lc.x += 10;
		else if (key_press == 'P')
			Lc.x -= 10;
		HTmatrix LM0_B;
		HT_Inverse(LMB_0, &LM0_B);
		HT_Point_Multiply(LM0_B, Lc, &LMouseDes);
		//printf("(%f, %f, %f)\n", LMouseDes.x, LMouseDes.y, LMouseDes.z);

		//Begin mouse control block
		//Assign desired IK points for mouse		
		//LMouseDes.x = LMouseBase[5].H[0][3]; LMouseDes.y = LMouseBase[5].H[1][3]; LMouseDes.z = LMouseBase[5].H[2][3];
		//RMouseDes.x = RMouseBase[5].H[0][3]; RMouseDes.y = RMouseBase[5].H[1][3]; RMouseDes.z = RMouseBase[5].H[2][3];
		//Move Mouse in the world
		HMW_B.H[0][3] = 0; HMW_B.H[1][3] = 200; HMW_B.H[2][3] = -50;
		HT_Multiply(HMW_B, LMB_0, &LMW_0);
		HT_Multiply(HMW_B, RMB_0, &RMW_0);
		//Begin R+L forward and inverse kinematics
		//Right
		forward_kinematics(DH_RMouse, RMouseAdj, RMouseBase, MOUSE_ARM_SIZE);
		inverse_kinematics(RMouseDes, DH_RMouse, RJ, RJ_Transpose, RMouseBase, MOUSE_ARM_SIZE, .000015);
		double Rxrot = 0.00; double Ryrot = 0; double Rzrot = 0.00;
		int i;
		for (i = 1; i <= MOUSE_ARM_SIZE; i++)
			DH_RMouse[i].theta += Rxrot * RJ_Transpose[i - 1][3] + Ryrot * RJ_Transpose[i - 1][4] + Rzrot * RJ_Transpose[i - 1][5];
	
		//Left
		forward_kinematics(DH_LMouse, LMouseAdj, LMouseBase, MOUSE_ARM_SIZE);
		if (endFlag)
			inverse_kinematics(LMouseDes, DH_LMouse, RJ, RJ_Transpose, LMouseBase, MOUSE_ARM_SIZE, .000015);
		//apply rotation
		double Lxrot = 0.00; double Lyrot = 0.00; double Lzrot = 0.00;
		for (i = 1; i <= MOUSE_ARM_SIZE; i++)
			DH_LMouse[i].theta += Lxrot * LJ_Transpose[i - 1][3] + Lyrot * LJ_Transpose[i - 1][4] + Lzrot * LJ_Transpose[i - 1][5];
		//End R+L forward and inverse kinematics
		//End Mouse Control Block










		////EXPERIMENTAL!!!! Full PID implementation. I do not have good coefficients, not tuned. Needs tuning
		////cancel the inverse_kinematics function output
		//P3D LMerror;
		//LMerror.x = .000015*(LMouseDes.x - LMouseBase[MOUSE_ARM_SIZE].H[0][3]);
		//LMerror.y = .000015*(LMouseDes.y - LMouseBase[MOUSE_ARM_SIZE].H[1][3]);
		//LMerror.z = .000015*(LMouseDes.z - LMouseBase[MOUSE_ARM_SIZE].H[2][3]);
		//for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		//	DH_LMouse[i].theta -= LMerror.x * LJ_Transpose[i - 1][0] + LMerror.y * LJ_Transpose[i - 1][1] + LMerror.z * LJ_Transpose[i - 1][2];
		//double P = .000005; double D = .000001; double I = .000001;
		////calculate d term
		//LMerror.x = (LMouseDes.x - LMouseBase[MOUSE_ARM_SIZE].H[0][3]);
		//LMerror.y = (LMouseDes.y - LMouseBase[MOUSE_ARM_SIZE].H[1][3]);
		//LMerror.z = (LMouseDes.z - LMouseBase[MOUSE_ARM_SIZE].H[2][3]);
		//LM_errDeriv.x = LMerror.x - LMprev_err.x;	LM_errDeriv.y = LMerror.y - LMprev_err.y;	LM_errDeriv.z = LMerror.z - LMprev_err.z;
		//LMprev_err.x = (LMouseDes.x - LMouseBase[MOUSE_ARM_SIZE].H[0][3]);
		//LMprev_err.y = (LMouseDes.y - LMouseBase[MOUSE_ARM_SIZE].H[1][3]);
		//LMprev_err.z = (LMouseDes.z - LMouseBase[MOUSE_ARM_SIZE].H[2][3]);
		////calculate i term
		//LM_errIntegral.x += LMerror.x; LM_errIntegral.y += LMerror.y; LM_errIntegral.z += LMerror.z;
		//for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		//{
		//	P3D PID_term;
		//	PID_term.x = (P*LMerror.x + D*LM_errDeriv.x + I*LM_errIntegral.x);	
		//	PID_term.y = (P*LMerror.y + D*LM_errDeriv.y + I*LM_errIntegral.y);
		//	PID_term.z = (P*LMerror.z + D*LM_errDeriv.z + I*LM_errIntegral.z);
		//	DH_LMouse[i].theta += PID_term.x * LJ_Transpose[i - 1][0] + PID_term.y * LJ_Transpose[i - 1][1] + PID_term.z * LJ_Transpose[i - 1][2];
		//}
		////print i and d
		//printf("D: (%f, %f, %f)\n", LM_errDeriv.x, LM_errDeriv.y, LM_errDeriv.z);
		//printf("I: (%f, %f, %f)\n", LM_errIntegral.x, LM_errIntegral.y, LM_errIntegral.z);

		forward_kinematics_KC(&Thermo);
		Line3d_2 ThermoSkeleton[6];
		HTmatrix HW_idx[6];
		HT_Multiply(Thermo.HW_B, Thermo.HB_0, &(HW_idx[0]));
		for (i = 1; i <= 5; i++)
			HT_Multiply(HW_idx[0], Thermo.HTbase[i], &(HW_idx[i]));
		

		display_robot_skeleton(&frame, &player, ThermoSkeleton, 255, 255, 255, HW_idx, MOUSE_ARM_SIZE);


		
		//Begin mouse draw block
		//Right
		HTmatrix RMouseW_idx[MOUSE_ARM_ARR_SIZE];
		copy_HT(&(RMouseW_idx[0]), &RMW_0);
		for (i = 1; i <= MOUSE_ARM_SIZE; i++)
			HT_Multiply(RMW_0, RMouseBase[i], &(RMouseW_idx[i]));
		Line3d_2 RMouseSkeleton[MOUSE_ARM_ARR_SIZE];
		display_robot_skeleton(&frame, &player, RMouseSkeleton, 255, 255, 0, RMouseW_idx, MOUSE_ARM_SIZE);
		//Left
		HTmatrix LMouseW_idx[MOUSE_ARM_ARR_SIZE];
		copy_HT(&(LMouseW_idx[0]), &LMW_0);
		for (i = 1; i <= MOUSE_ARM_SIZE; i++)
			HT_Multiply(LMW_0, LMouseBase[i], &(LMouseW_idx[i]));
		Line3d_2 LMouseSkeleton[MOUSE_ARM_ARR_SIZE];
		display_robot_skeleton(&frame, &player, LMouseSkeleton, 255, 0, 255, LMouseW_idx, MOUSE_ARM_SIZE);


		//forward_kinematics(DH_LMouse, LMouseAdj, LMouseBase, MOUSE_ARM_SIZE);
		P3D l2des;
		l2des.x = 50;
		l2des.y = 50;
		l2des.z = 50;
//		forward_kinematics(LM.DH_Table, LM.HTadj, LM.HTbase, MOUSE_ARM_SIZE);
//		inverse_kinematics(l2des, LM.DH_Table, LM.J, LM.J_Transpose, LM.HTbase, MOUSE_ARM_SIZE, .000015);
		forward_kinematics_KC(&LM);
		inverse_kinematics_KC(l2des, &LM, .000015);

		Line3d_2 LMouseSkeleton2[MOUSE_ARM_ARR_SIZE];
//		display_robot_skeleton(&frame, &player, LMouseSkeleton2, 255, 255, 0, LM.HTbase, LM.size);

		CHbase[0].H[0][0] = 1; CHbase[0].H[0][1] = 0; CHbase[0].H[0][2] = 0; CHbase[0].H[0][3] = 0;
		CHbase[0].H[1][0] = 0; CHbase[0].H[1][1] = 1; CHbase[0].H[1][2] = 0; CHbase[0].H[1][3] = 0;
		CHbase[0].H[2][0] = 0; CHbase[0].H[2][1] = 0; CHbase[0].H[2][2] = 1; CHbase[0].H[2][3] = 0;
		CHbase[0].H[3][0] = 0; CHbase[0].H[3][1] = 0; CHbase[0].H[3][2] = 0; CHbase[0].H[3][3] = 1;
		forward_kinematics(cam_wrist, CHadj, CHbase, 3);
		Line3d_2 CamSK[4];
		display_robot_skeleton(&frame, &player, CamSK, 255, 255, 0, CHbase, 3);
		if (key_press == 'J')
			cam_wrist[1].theta += .1;
		else if (key_press == 'L')
			cam_wrist[1].theta -= .1;
		else if (key_press == 'I')
			cam_wrist[2].theta += .1;
		else if (key_press == 'K')
			cam_wrist[2].theta -= .1;

		else if (key_press == 'N')
			cam_wrist[3].theta += .1;
		else if (key_press == 'M')
			cam_wrist[3].theta -= .1;

		printf("(%f, %f, %f)\n", CHbase[3].H[0][3], CHbase[3].H[1][3], CHbase[3].H[2][3]);


		//Left, right X+Y+Z axes in the world, on each frame.
		Line3d_2 LAxis[MOUSE_ARM_ARR_SIZE];
		Line3d_2 RAxis[MOUSE_ARM_ARR_SIZE];
		for (i = 0; i <= MOUSE_ARM_SIZE; i++)
		{
			P3D p1t;
			P3D p2t;
			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 0; p2t.y = 0; p2t.z = 15;
			HT_Point_Multiply(LMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(LMouseW_idx[i], p2t, &p2t);
			LAxis[i].x1 = p1t.x; LAxis[i].y1 = p1t.y; LAxis[i].z1 = p1t.z;
			LAxis[i].x2 = p2t.x; LAxis[i].y2 = p2t.y; LAxis[i].z2 = p2t.z;
			project_line_geometry(player, LAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 0, 255);

			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 0; p2t.y = 0; p2t.z = 15;
			HT_Point_Multiply(RMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(RMouseW_idx[i], p2t, &p2t);
			RAxis[i].x1 = p1t.x; RAxis[i].y1 = p1t.y; RAxis[i].z1 = p1t.z;
			RAxis[i].x2 = p2t.x; RAxis[i].y2 = p2t.y; RAxis[i].z2 = p2t.z;
			project_line_geometry(player, RAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 0, 255);
		}
		for (i = 0; i <= MOUSE_ARM_SIZE; i++)
		{
			P3D p1t;
			P3D p2t;
			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 15; p2t.y = 0; p2t.z = 0;
			HT_Point_Multiply(LMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(LMouseW_idx[i], p2t, &p2t);
			LAxis[i].x1 = p1t.x; LAxis[i].y1 = p1t.y; LAxis[i].z1 = p1t.z;
			LAxis[i].x2 = p2t.x; LAxis[i].y2 = p2t.y; LAxis[i].z2 = p2t.z;
			project_line_geometry(player, LAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 255, 0, 0);

			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 15; p2t.y = 0; p2t.z = 0;
			HT_Point_Multiply(RMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(RMouseW_idx[i], p2t, &p2t);
			RAxis[i].x1 = p1t.x; RAxis[i].y1 = p1t.y; RAxis[i].z1 = p1t.z;
			RAxis[i].x2 = p2t.x; RAxis[i].y2 = p2t.y; RAxis[i].z2 = p2t.z;
			project_line_geometry(player, RAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 255, 0, 0);
		}
		for (i = 0; i <= MOUSE_ARM_SIZE; i++)
		{
			P3D p1t;
			P3D p2t;
			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 0; p2t.y = 15; p2t.z = 0;
			HT_Point_Multiply(LMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(LMouseW_idx[i], p2t, &p2t);
			LAxis[i].x1 = p1t.x; LAxis[i].y1 = p1t.y; LAxis[i].z1 = p1t.z;
			LAxis[i].x2 = p2t.x; LAxis[i].y2 = p2t.y; LAxis[i].z2 = p2t.z;
			project_line_geometry(player, LAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 255, 0);

			p1t.x = 0; p1t.y = 0; p1t.z = 0;
			p2t.x = 0; p2t.y = 15; p2t.z = 0;
			HT_Point_Multiply(RMouseW_idx[i], p1t, &p1t);
			HT_Point_Multiply(RMouseW_idx[i], p2t, &p2t);
			RAxis[i].x1 = p1t.x; RAxis[i].y1 = p1t.y; RAxis[i].z1 = p1t.z;
			RAxis[i].x2 = p2t.x; RAxis[i].y2 = p2t.y; RAxis[i].z2 = p2t.z;
			project_line_geometry(player, RAxis[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 255, 0);
		}
		copy_HT(&(MouseBodyPyramid.HW_B), &HMW_B);
		update_pyramid(&MouseBodyPyramid, &frame, &player, 255, 255, 255);
		//end mouse draw block


		if (key_press == '.')
			player.lamda *= 1.01;
		else if (key_press == ',')
			player.lamda *= .99;


		//Draw the world axes
		Line3d_2 world_axis[3];
		world_axis[0].x1 = 0;	world_axis[0].y1 = 0;	world_axis[0].z1 = 0;
		world_axis[0].x2 = 20;	world_axis[0].y2 = 0;	world_axis[0].z2 = 0;
		world_axis[1].x1 = 0;	world_axis[1].y1 = 0;	world_axis[1].z1 = 0;
		world_axis[1].x2 = 0;	world_axis[1].y2 = 20;	world_axis[1].z2 = 0;
		world_axis[2].x1 = 0;	world_axis[2].y1 = 0;	world_axis[2].z1 = 0;
		world_axis[2].x2 = 0;	world_axis[2].y2 = 0;	world_axis[2].z2 = 20;
		project_line_geometry(player, world_axis[0], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 255, 0, 0);
		project_line_geometry(player, world_axis[1], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 255, 0);
		project_line_geometry(player, world_axis[2], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 0, 255);
		//finish



		//Run second mouse. needs updating
		Run_Mouse(&M1, 0, 0, 0, RMouseDes, 0, 0, 0, LMouseDes);
		//Display_Mouse(M1, &frame, &player);
		//End

		//Begin twitchy control block
		//update world to base frames
		//W_B is user changeable			//B_0[i] is fixed, part of twitchy	//the W_0 relationship changes if twitchy moves, and we draw from 0(base frame of mr. twitchy) any other parts in the base have to be updated too.		
		dev_TW_B.H[1][3] = -300; dev_TW_B.H[2][3] = -100;
		for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
			HT_Multiply(dev_TW_B, dev_TB_0[i], &(dev_TW_0[i]));
		//assign points (all)
		P3D AllFeetB;
		AllFeetB.x = 0;	AllFeetB.y = 0;	AllFeetB.z = -200;
		double fineTime = ((double)time_step) / 1000;
		AllFeetB.x = 50 * cos(fineTime * 10); AllFeetB.y = 50 * sin(fineTime * 10);
		for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
			HT_Point_Multiply(dev_T0_B[i], AllFeetB, &(TWFootCoord[i]));
		//forward, inverse kinematics, and display nested.
		Line3d_2 Twitchy_Leg_Skeleton[TWITCHY_ALL_LEGS_ARR_SIZE][TWITCHY_LEG_ARR_SIZE];
		HTmatrix TW_W_idx[TWITCHY_ALL_LEGS_ARR_SIZE][TWITCHY_LEG_ARR_SIZE];
		for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
		{
			forward_kinematics(DH_Twitchy[i], TWAdj[i], TWZero[i], TWITCHY_LEG_SIZE);
			inverse_kinematics(TWFootCoord[i], DH_Twitchy[i], TJ[i], TJ_Transpose[i], TWZero[i], TWITCHY_LEG_SIZE, .00002);
			TW_W_idx[i][0].H[0][3] = dev_TW_0[i].H[0][3];	TW_W_idx[i][0].H[1][3] = dev_TW_0[i].H[1][3];	TW_W_idx[i][0].H[2][3] = dev_TW_0[i].H[2][3];
			int j;
			for (j = 1; j <= TWITCHY_NUM_LEGS; j++)
				HT_Multiply(dev_TW_0[i], TWZero[i][j], &(TW_W_idx[i][j]));
			display_robot_skeleton(&frame, &player, Twitchy_Leg_Skeleton[i], 255, 255, 255, TW_W_idx[i], TWITCHY_LEG_SIZE);
		}
		//end twitchy control block


		//copy_HT(&AViCS_HW_B, &(LMouseW_idx[5]));

		//Begin AVICS
		//Begin AViCS control block
		forward_kinematics(DH_AViCS, AViCS_Hadj, AViCS_Hzero, AVICS_NUM_DOFS);
		inverse_kinematics(AViCS_des, DH_AViCS, AViCS_J, AViCS_J_Transpose, AViCS_Hzero, AVICS_NUM_DOFS, .00002);
		for (i = 1; i <= AVICS_NUM_DOFS; i++)
			DH_AViCS[i].theta += AViCS_Xrot*AViCS_J_Transpose[i - 1][3] + AViCS_Yrot*AViCS_J_Transpose[i - 1][4] + AViCS_Zrot*AViCS_J_Transpose[i - 1][5];
		//end AViCS control block
		//begin avics draw
		HTmatrix AViCS_W_[AVICS_ARR_SIZE];
		HTmatrix AViCS_W_0;
		HT_Multiply(AViCS_HW_B, AVIiCS_HB_0, &AViCS_W_0);
		for (i = 1; i <= AVICS_NUM_DOFS; i++)
			HT_Multiply(AViCS_W_0, AViCS_Hzero[i], &(AViCS_W_[i]));
		AViCS_W_[0].H[0][3] = AViCS_W_[1].H[0][3];	AViCS_W_[0].H[1][3] = AViCS_W_[1].H[1][3];	AViCS_W_[0].H[2][3] = AViCS_W_[1].H[2][3];
		Line3d_2 AViCS_arm_skeleton[AVICS_ARR_SIZE];
		display_robot_skeleton(&frame, &player, AViCS_arm_skeleton, 0, 0, 255, AViCS_W_, AVICS_NUM_DOFS);
		//End AViCS block

		//Begin avics cam frame
		P3D tempPdes;
		HTmatrix AViCS_6_0;
		HT_Inverse(AViCS_Hzero[6], &AViCS_6_0);
		HT_Point_Multiply(AViCS_6_0, AViCS_des, &tempPdes);
		P3D AViCS_CAM[9];
		P3D AViCS_CAM_world[9];
		AViCS_CAM[0].x = 7.102;		AViCS_CAM[0].y = 0;		AViCS_CAM[0].z = 20.237;
		AViCS_CAM[1].x = 25.035;	AViCS_CAM[1].y = 0;		AViCS_CAM[1].z = 13.479;
		AViCS_CAM[2].x = 17.801;	AViCS_CAM[2].y = 0;		AViCS_CAM[2].z = -23.689;
		AViCS_CAM[3].x = 2.364;		AViCS_CAM[3].y = 0;		AViCS_CAM[3].z = -28.519;
		AViCS_CAM[4].x = -2.364;	AViCS_CAM[4].y = 0;		AViCS_CAM[4].z = -28.519;
		AViCS_CAM[5].x = -17.801;	AViCS_CAM[5].y = 0;		AViCS_CAM[5].z = -23.689;
		AViCS_CAM[6].x = -25.035;	AViCS_CAM[6].y = 0;		AViCS_CAM[6].z = 13.479;
		AViCS_CAM[7].x = -7.102;	AViCS_CAM[7].y = 0;		AViCS_CAM[7].z = 20.237;
		AViCS_CAM[8].x = 7.102;		AViCS_CAM[8].y = 0;		AViCS_CAM[8].z = 20.237;
		HTmatrix AViCS_CamFrame;
		HTmatrix AViCS_6Rotation;
		AViCS_6Rotation.H[0][0] = 1;	AViCS_6Rotation.H[0][1] = 0;				AViCS_6Rotation.H[0][2] = 0;				AViCS_6Rotation.H[0][3] = 0;
		AViCS_6Rotation.H[1][0] = 0;	AViCS_6Rotation.H[1][1] = cos(-pi_v / 2);	AViCS_6Rotation.H[1][2] = -sin(-pi_v / 2);	AViCS_6Rotation.H[1][3] = 0;
		AViCS_6Rotation.H[2][0] = 0;	AViCS_6Rotation.H[2][1] = sin(-pi_v / 2);	AViCS_6Rotation.H[2][2] = cos(-pi_v / 2);	AViCS_6Rotation.H[2][3] = 0;
		AViCS_6Rotation.H[3][0] = 0;	AViCS_6Rotation.H[3][1] = 0;				AViCS_6Rotation.H[3][2] = 0;				AViCS_6Rotation.H[3][3] = 1;
		Line3d_2 cam_render[9];
		for (i = 0; i < 9; i++)
			HT_Point_Multiply(AViCS_W_[6], AViCS_CAM[i], &(AViCS_CAM_world[i]));
		for (i = 0; i < 8; i++)
		{
			cam_render[i].x1 = AViCS_CAM_world[i].x;		cam_render[i].y1 = AViCS_CAM_world[i].y;		cam_render[i].z1 = AViCS_CAM_world[i].z;
			cam_render[i].x2 = AViCS_CAM_world[i + 1].x;	cam_render[i].y2 = AViCS_CAM_world[i + 1].y;		cam_render[i].z2 = AViCS_CAM_world[i + 1].z;
			project_line_geometry(player, cam_render[i], &tu1, &tv1, &tu2, &tv2);
			draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 0, 0, 255);
		}
		HT_Multiply(AViCS_W_[6], AViCS_6Rotation, &AViCS_CamFrame);
		int r; int c;
		for (r = 0; r < 3; r++)
		{
			for (c = 0; c < 3; c++)
			{
				camAViCS.boundary.R[r][c] = AViCS_CamFrame.H[r][c];
			}
		}
		camAViCS.boundary.x = AViCS_CamFrame.H[0][3]; camAViCS.boundary.y = AViCS_CamFrame.H[1][3]; camAViCS.boundary.z = AViCS_CamFrame.H[2][3];

		display_robot_skeleton(&AViCS_ViewPort, &camAViCS, RMouseSkeleton, 0, 255, 0, RMouseW_idx, MOUSE_ARM_SIZE);
		display_robot_skeleton(&AViCS_ViewPort, &camAViCS, LMouseSkeleton, 255, 0, 255, LMouseW_idx, MOUSE_ARM_SIZE);
		for (i = 1; i <= TWITCHY_NUM_LEGS; i++)
			display_robot_skeleton(&AViCS_ViewPort, &camAViCS, Twitchy_Leg_Skeleton[i], 255, 255, 255, TW_W_idx[i], TWITCHY_LEG_SIZE);
		Display_Mouse(M1, &AViCS_ViewPort, &camAViCS);
		//end avics cam frame



		if (key_press == 'p')
			cout << "player coordinate = (" << player.boundary.x << ", " << player.boundary.y << ", " << player.boundary.z << ")" << endl;
		if (key_press >= '1' && key_press <= '6')
		{
			AViCS_LinkNum = key_press - '0';
			cout << "link num: " << AViCS_LinkNum << endl;
		}
		if (key_press >= 1 && key_press <= 5)
		{
			Lm_linknum = key_press - '0';
		}

		if (key_press == 'r')
		{
			DH_LMouse[Lm_linknum].theta +=.1;
			DH_RMouse[Lm_linknum].theta -=.1;

			DH_AViCS[AViCS_LinkNum].theta += .1;
//			cout << "theta " << AViCS_LinkNum << " = " << DH_AViCS[AViCS_LinkNum].theta << endl;
//			for (i = 1; i <= MOUSE_ARM_SIZE; i++)
//				cout << "Origin " << i << " = " << "(" << LMouseBase[i].H[0][3] * 2 << ", " << LMouseBase[i].H[1][3] * 2 << ", " << LMouseBase[i].H[2][3] * 2 << ")" << endl;

		}
		else if (key_press == 'f')
		{
			DH_LMouse[Lm_linknum].theta -= .1;
			DH_RMouse[Lm_linknum].theta += .1;

			DH_AViCS[AViCS_LinkNum].theta -= .1;
//			cout << "link num: " << AViCS_LinkNum << endl;
//			cout << "theta " << AViCS_LinkNum << " = " << DH_AViCS[AViCS_LinkNum].theta << endl;
//			for (i = 1; i <= MOUSE_ARM_SIZE; i++)
//				cout << "Origin " << i << " = " << "(" << LMouseBase[i].H[0][3] * 2 << ", " << LMouseBase[i].H[1][3] * 2 << ", " << LMouseBase[i].H[2][3] * 2 << ")" << endl;

		}

		if (key_press == 'A')
			AViCS_Xrot += .01;
		else if (key_press == 'D')
			AViCS_Xrot += -.01;
		//else
		//	AViCS_Xrot = 0;

		if (key_press == 'W')
			AViCS_Yrot += .01;
		else if (key_press == 'S')
			AViCS_Yrot += -.01;
		//else
		//	AViCS_Yrot = 0;

		if (key_press == 'Z')
			AViCS_Zrot += .01;
		else if (key_press == 'X')
			AViCS_Zrot += -.01;
		//else
		//	AViCS_Zrot = 0;


		HTmatrix AViCS_H6_0;
		HT_Inverse(AViCS_Hzero[6], &AViCS_H6_0);
		P3D point6;
		HT_Point_Multiply(AViCS_H6_0, AViCS_des, &point6);
		
		
		if (key_press == 'V')
			point6.x += 5;
		else if (key_press == 'B')
			point6.x += -5;
		else if (key_press == 'I')
			point6.y += 5;
		else if (key_press == 'K')
			point6.y += -5;
		else if (key_press == 'N')
			point6.z += 5;
		else if (key_press == 'M')
			point6.z += -5;
		else if (key_press== 'J')
		{
			DH_AViCS[1].theta += .07;
			forward_kinematics(DH_AViCS, AViCS_Hadj, AViCS_Hzero, AVICS_NUM_DOFS);
			AViCS_des.x = AViCS_Hzero[6].H[0][3];   AViCS_des.y = AViCS_Hzero[6].H[1][3];   AViCS_des.z = AViCS_Hzero[6].H[2][3];
		}
		else if (key_press  == 'L')
		{
			DH_AViCS[1].theta -= .07;
			forward_kinematics(DH_AViCS, AViCS_Hadj, AViCS_Hzero, AVICS_NUM_DOFS);
			AViCS_des.x = AViCS_Hzero[6].H[0][3];   AViCS_des.y = AViCS_Hzero[6].H[1][3];   AViCS_des.z = AViCS_Hzero[6].H[2][3];
		}

		HT_Point_Multiply(AViCS_Hzero[6], point6, &AViCS_des);

		
		if (key_press > '0' && key_press < '6')
			Lm_linknum = key_press - '0';

		
		if (key_press == 'n')
		{
			cout << "Environment Paused. Press 'm' for mouse, 't' for twitchy, or 'p' for the two link planar arm." << endl;
			char newinput;
			cin >> newinput;
			if (newinput == 'm')
			{
				cout << "Enter 'l' for left arm or 'r' for right arm" << endl;
				cin >> newinput;
				if (newinput == 'r')
				{
					cout << "Enter x world coordinate, y world coordinate, z world coordinate of end effector destination" << endl;
					int xin; int yin; int zin;
					cout << "X: " << endl;
					cin >> xin;
					cout << "Y: " << endl;
					cin >> yin;
					cout << "Z: " << endl;
					cin >> zin;

					//					HT_Matrix_Vect_Multiply(RH0_W, (double)xin, (double)yin, (double)zin, &RM_xpos, &RM_ypos, &RM_zpos);
					//RM_xpos = xin;
					//RM_ypos = yin;
					//RM_zpos = zin;
				}
				else if (newinput == 'l')
				{
					cout << "Enter x world coordinate, y world coordinate, z world coordinate of end effector destination" << endl;
					int xin; int yin; int zin;
					cout << "X: " << endl;
					cin >> xin;
					cout << "Y: " << endl;
					cin >> yin;
					cout << "Z: " << endl;
					cin >> zin;

					//				HT_Matrix_Vect_Multiply(LH0_W, (double)xin, (double)yin, (double)zin, &LM_xpos, &LM_ypos, &LM_zpos);
					//			LM_xpos = xin;
					//			LM_ypos = yin;
					//			LM_zpos = zin;

				}


			}
			else if (newinput == 't')
			{
				char inpttype;
				cout << "Type 'a' for all legs, any other key for a spcific leg" << endl;
				cin >> inpttype;

				int legnum;
				double xin; double yin; double zin;

				if (inpttype != 'a')
				{
					cout << "Enter leg number and base frame coordinate" << endl;
					cout << "leg number" << endl;
					cin >> legnum;
					if (legnum > 6){ legnum = 6; }
					else if (legnum < 1){ legnum = 1; }
					T_Moving_Leg_Num = legnum;
					cout << "X: " << endl;
					cin >> xin;
					cout << "Y: " << endl;
					cin >> yin;
					cout << "Z: " << endl;
					cin >> zin;

					cout << "'z' for zero frame, 'b' for base frame" << endl;
					char bzin;
					cin >> bzin;
					//if (bzin == 'b')
					//	HT_Matrix_Vect_Multiply(T0_B[legnum], xin, yin, zin, &(T_xpos[legnum]), &(T_ypos[legnum]), &(T_zpos[legnum]));
					//else if (bzin == 'z')
					//{
					//	T_xpos[legnum] = xin; T_ypos[legnum] = yin; T_zpos[legnum] = zin;
					//}
				}

				else
				{
					cout << "X: " << endl;
					cin >> xin;
					cout << "Y: " << endl;
					cin >> yin;
					cout << "Z: " << endl;
					cin >> zin;
					cout << "'z' for zero frame, 'b' for base frame" << endl;
					char bzin;
					cin >> bzin;
					for (legnum = 1; legnum <= 6; legnum++)
					{
						//if (bzin == 'b')
						//	HT_Matrix_Vect_Multiply(T0_B[legnum], xin, yin, zin, &(T_xpos[legnum]), &(T_ypos[legnum]), &(T_zpos[legnum]));
						//else if (bzin == 'z')
						//{
						//	T_xpos[legnum] = xin; T_ypos[legnum] = yin; T_zpos[legnum] = zin;
						//}
					}
				}

				//T_xpos[legnum] = xin;

			}
			else if (newinput == 'p')
			{

				cout << "Enter x world coordinate and y world coordinate end effector destination" << endl;
				int xin; int yin;
				cout << "X: " << endl;
				cin >> xin;
				cout << "Y: " << endl;
				cin >> yin;
				x_des = xin;
				y_des = yin;

			}
		}
		else if (key_press == 'b')
		{
			cout << "Environment Paused. Enter new speed" << endl;
			cin >> ef_v_weight;
			cout << "Speed is: " << ef_v_weight << endl;

		}
		////////////////////////////////////////////////////////////////////////////////////EOC		






		update_R(player.boundary.R, theta1, theta2, theta3);
		if (key_press == 'w')
			theta2 += .05;
		else if (key_press == 's')
			theta2 -= .05;
		else if (key_press == 'a')
			theta1 += .05;
		else if (key_press == 'd')
			theta1 -= .05;
		else if (key_press == 'z')
			theta3 += 3.14159265359 / 2;
		else if (key_press == 'x')
			theta3 -= 3.14159265359 / 2;
		if (key_press == 'i')
			move_forward_backward(theta1, &player.boundary.x, &player.boundary.y, 1.0);
		else if (key_press == 'k')
			move_forward_backward(theta1, &player.boundary.x, &player.boundary.y, -1.0);
		else if (key_press == 'j')
			strafe_left_right(theta1, &player.boundary.x, &player.boundary.y, 1.0);
		else if (key_press == 'l')
			strafe_left_right(theta1, &player.boundary.x, &player.boundary.y, -1.0);
		if (key_press == '[')
			player.boundary.z--;
		else if (key_press == ']')
			player.boundary.z++;



		HTmatrix playerW_C;
		for (r = 0; r < 3; r++)
		{
			for (c = 0; c < 3; c++)
			{
				playerW_C.H[r][c] = player.boundary.R[r][c];
			}
		}
		playerW_C.H[0][3] = player.boundary.x;	playerW_C.H[1][3] = player.boundary.y;	playerW_C.H[2][3] = player.boundary.z;
		playerW_C.H[3][0] = 0; playerW_C.H[3][1] = 0; playerW_C.H[3][2] = 0; playerW_C.H[3][3] = 1;
	
		
		Line3d_2 sphereRender;
		P3D cir1; P3D cir2;
		double us; double vs;
		for (us = -2*pi_v; us < 2*pi_v; us += .5)
		{
			for (vs = -2*pi_v; vs < 2*pi_v; vs += .5)
			{
				double term1 = sqrt(6.25 - us*us);
				cir1.x = term1*cos(vs);
				cir1.y = term1*sin(vs);
				cir1.z = us;
				cir2.x = cir1.x*.9;
				cir2.y = cir1.y*.9;
				cir2.z = cir1.z*.9;
				HT_Point_Multiply(playerW_C, cir1, &(cir1));
				HT_Point_Multiply(playerW_C, cir2, &(cir2));

				sphereRender.x1 = cir1.x; sphereRender.y1 = cir1.y; sphereRender.z1 = cir1.z;
				sphereRender.x2 = cir2.x; sphereRender.y2 = cir2.y; sphereRender.z2 = cir2.z;
				project_line_geometry(camAViCS, sphereRender, &tu1, &tv1, &tu2, &tv2);
				draw_projected_line2_on_frame(AViCS_ViewPort, tu1, tv1, tu2, tv2, 128, 0, 0);
			}
		}
		P3D eyeSlit[4];
		eyeSlit[0].x = 1;	eyeSlit[0].y = 0;	eyeSlit[0].z = 2.5;
		eyeSlit[1].x = -1;	eyeSlit[1].y = 0;	eyeSlit[1].z = 2.5;
		eyeSlit[2].x = 0;	eyeSlit[2].y = 1;	eyeSlit[2].z = 2.5;
		eyeSlit[3].x = 0;	eyeSlit[3].y = -1;	eyeSlit[3].z = 2.5;
		for (i = 0; i < 4; i++)
			HT_Point_Multiply(playerW_C, eyeSlit[i], &(eyeSlit[i]) );
		Line3d_2 slit1, slit2;
		slit1.x1 = eyeSlit[0].x; slit1.y1 = eyeSlit[0].y; slit1.z1 = eyeSlit[0].z;
		slit1.x2 = eyeSlit[1].x; slit1.y2 = eyeSlit[1].y; slit1.z2 = eyeSlit[1].z;
		slit2.x1 = eyeSlit[2].x; slit2.y1 = eyeSlit[2].y; slit2.z1 = eyeSlit[2].z;
		slit2.x2 = eyeSlit[3].x; slit2.y2 = eyeSlit[3].y; slit2.z2 = eyeSlit[3].z;
		project_line_geometry(camAViCS, slit1, &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(AViCS_ViewPort, tu1, tv1, tu2, tv2, 128, 0, 128);
		project_line_geometry(camAViCS, slit2, &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(AViCS_ViewPort, tu1, tv1, tu2, tv2, 128, 0, 128);

		
		draw_box_on_frame(frame, player, boxes[0]);
		draw_box_on_frame(frame, player, boxes[1]);
		draw_box_on_frame(frame, player, boxes[2]);
		draw_box_on_frame(frame, player, boxes[3]);
		draw_box_on_frame(frame, player, boxes[4]);
		draw_box_on_frame(AViCS_ViewPort, camAViCS, boxes[0]);
		draw_box_on_frame(AViCS_ViewPort, camAViCS, boxes[1]);
		draw_box_on_frame(AViCS_ViewPort, camAViCS, boxes[2]);
		draw_box_on_frame(AViCS_ViewPort, camAViCS, boxes[3]);
		draw_box_on_frame(AViCS_ViewPort, camAViCS, boxes[4]);


		

		if (time_step < 10000000)
			video.write(frame);
		imshow("Env", frame);
		imshow("AViCS Viewport", AViCS_ViewPort);

		if (key_press == 'Q' || key_press == 'q')
			stop = 1;

	}

	video.release();
}














































/*

string x_base_name = "logic [15:0] obj_x_pos_";
string y_base_name = "logic [15:0] obj_y_pos_";
string width_base_name = "logic [15:0] obj_width_";
string height_base_name = "logic [15:0] obj_height_";
string vlog_dec_format_base_name = " = 16'h";

ofstream myfile;
myfile.open("init_sysvlg.txt");
myfile.clear();
for (int i = 0; i < num_obstacles; i++)
{
unsigned int num_to_op;
string tmp_op;

tmp_op = x_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].x - obj_list[i].width / 2;
myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; ";

tmp_op = y_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].y - obj_list[i].height / 2;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; " << endl;

tmp_op = width_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].width;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; ";

tmp_op = height_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].height;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; " << endl;
}
myfile.close();

*/








//code for slidey camera
/*
//THESE GO OUTSIDE FOR LOOP
//use for sliding window??
int camera_right_border = 640 / 2 + 640 / 4;
int camera_left_border = 640 / 4;
int camera_up_border = 200 - player.height - 10;
int camera_down_border = (480 - player.height - 10);
int player_right_push = 640/2;
int player_left_push = 640 / 2;

int player_down_push = (480 - player.height - 10);
int player_up_push = (480 - player.height - 10);

//END OF OUSTIDE FOR LOOP



//update the cam offset in the for loop
if (player.x > player_right_push)
{
player_right_push = player.x;
player_left_push = player.x - 640 / 2;
}
else if (player.x < player_left_push)
{
player_left_push = player.x;
player_right_push = player.x + 640 / 2;
}

if (player.y > player_down_push)
{
player_down_push = player.y;
player_up_push = player.y - 280;
}
else if (player.y < player_up_push)
{
player_up_push = player.y;
player_down_push = player.y + 280;
}

if (player_right_push > camera_right_border)
{
camera_x_offset = camera_right_border - player_right_push;
}
else if (player_left_push < camera_left_border)
{
camera_x_offset = camera_left_border - player_left_push;
}

if (player_down_push > camera_down_border)
{
camera_y_offset = camera_down_border - player_down_push;
}
else if (player_up_push < camera_up_border)
{
camera_y_offset = camera_up_border - player_up_push;
}











string x_base_name = "logic [15:0] obj_x_pos_";
string y_base_name = "logic [15:0] obj_y_pos_";
string width_base_name = "logic [15:0] obj_width_";
string height_base_name = "logic [15:0] obj_height_";
string vlog_dec_format_base_name = " = 16'h";

ofstream myfile;
myfile.open("init_sysvlg.txt");
myfile.clear();
for (int i = 0; i < num_obstacles; i++)
{
unsigned int num_to_op;
string tmp_op;

tmp_op = x_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].x - obj_list[i].width / 2;
myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; ";

tmp_op = y_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].y - obj_list[i].height / 2;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; " << endl;

tmp_op = width_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].width;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; ";

tmp_op = height_base_name;
tmp_op.append(to_string(i));
tmp_op.append(vlog_dec_format_base_name);
num_to_op = obj_list[i].height;

myfile << tmp_op;
//printf("%x", num_to_op);
myfile << hex << num_to_op;
myfile << "; " << endl;
}
myfile.close();


*/





















/*







string H0_1[4][4];
H0_1[0][0] = "(c1)"; H0_1[0][1] = "0"; H0_1[0][2] = "(-s1)"; H0_1[0][3] = "(a1*c1)";
H0_1[1][0] = "(s1)"; H0_1[1][1] = "0"; H0_1[1][2] = "(c1)"; H0_1[1][3] = "(a1*s1)";
H0_1[2][0] = "0"; H0_1[2][1] = "(-1)"; H0_1[2][2] = "0"; H0_1[2][3] = "(d1)";
H0_1[3][0] = "0"; H0_1[3][1] = "0"; H0_1[3][2] = "0"; H0_1[3][3] = "1";

string H1_2[4][4];
H1_2[0][0] = "(c2)"; H1_2[0][1] = "0"; H1_2[0][2] = "(-s2)"; H1_2[0][3] = "(a2*c2)";
H1_2[1][0] = "(s2)"; H1_2[1][1] = "0"; H1_2[1][2] = "(c2)"; H1_2[1][3] = "(a2*s2)";
H1_2[2][0] = "0"; H1_2[2][1] = "(-1)"; H1_2[2][2] = "0"; H1_2[2][3] = "(d2)";
H1_2[3][0] = "0"; H1_2[3][1] = "0"; H1_2[3][2] = "0"; H1_2[3][3] = "1";

string H2_3[4][4];
H2_3[0][0] = "(c3)"; H2_3[0][1] = "0"; H2_3[0][2] = "(-s3)"; H2_3[0][3] = "(a3*c3)";
H2_3[1][0] = "(s3)"; H2_3[1][1] = "0"; H2_3[1][2] = "(c3)"; H2_3[1][3] = "(a3*s3)";
H2_3[2][0] = "0"; H2_3[2][1] = "(-1)"; H2_3[2][2] = "0"; H2_3[2][3] = "(d3)";
H2_3[3][0] = "0"; H2_3[3][1] = "0"; H2_3[3][2] = "0"; H2_3[3][3] = "1";


string H3_4[4][4];
H3_4[0][0] = "(c4)"; H3_4[0][1] = "0"; H3_4[0][2] = "(-s4)"; H3_4[0][3] = "(a4*c4)";
H3_4[1][0] = "(s4)"; H3_4[1][1] = "0"; H3_4[1][2] = "(c4)"; H3_4[1][3] = "(a4*s4)";
H3_4[2][0] = "0"; H3_4[2][1] = "(-1)"; H3_4[2][2] = "0"; H3_4[2][3] = "(d4)";
H3_4[3][0] = "0"; H3_4[3][1] = "0"; H3_4[3][2] = "0"; H3_4[3][3] = "1";

string H4_5[4][4];
H4_5[0][0] = "(c5)"; H4_5[0][1] = "(-s5)"; H4_5[0][2] = "0"; H4_5[0][3] = "0";
H4_5[1][0] = "(s5)"; H4_5[1][1] = "(c5)"; H4_5[1][2] = "0"; H4_5[1][3] = "0";
H4_5[2][0] = "0"; H4_5[2][1] = "0"; H4_5[2][2] = "(1)"; H4_5[2][3] = "(d5)";
H4_5[3][0] = "0"; H4_5[3][1] = "0"; H4_5[3][2] = "0"; H4_5[3][3] = "1";



Write_HT_to_file("H0_1.txt", H0_1);

string H0_2[4][4];
Multiply_STR_HTM(H0_1, H1_2, H0_2);
Write_HT_to_file("H0_2.txt", H0_2);


string H0_3[4][4];
Multiply_STR_HTM(H0_2, H2_3, H0_3);
Write_HT_to_file("H0_3.txt", H0_3);


string H0_4[4][4];
Multiply_STR_HTM(H0_3, H3_4, H0_4);
Write_HT_to_file("H0_4.txt", H0_4);

string H0_5[4][4];
Multiply_STR_HTM(H0_4, H4_5, H0_5);
Write_HT_to_file("H0_5.txt", H0_5);







*/




























//////////////////////////////////////////////////////////////////////////////////////////left arm
//double Lm_prev[6];
//Lm_prev[1] = -1.57079632679; Lm_prev[2] = 1.57079632679;  Lm_prev[3] = 1.57079632679;  Lm_prev[4] = 1.57079632679;  Lm_prev[5] = 0.0;

//double Lm_theta[6];
//Lm_theta[1] = -1.57079632679; Lm_theta[2] = 1.57079632679;  Lm_theta[3] = 1.57079632679;  Lm_theta[4] = 1.57079632679;  Lm_theta[5] = 0.0;

//double Lm_a1; double Lm_a2; double Lm_a3; double Lm_a4;
//double Lm_d1; double Lm_d2; double Lm_d3; double Lm_d4; double Lm_d5;

//if (MOUSE_MODE == 1)
//{
//	Lm_a1 = 11.471;  Lm_a2 = 10.16;  Lm_a3 = 10.59;  Lm_a4 = 5.99;
//	Lm_d1 = 14.564;  Lm_d2 = 11.246;  Lm_d3 = 98.709;  Lm_d4 = 7.08;  Lm_d5 = 51.135;
//}
//else
//{
//	 Lm_a1 = 0.0;  Lm_a2 = 0.0;  Lm_a3 = 0.0;  Lm_a4 = 0.0;
//	 Lm_d1 = 0.0;  Lm_d2 = 0.0;  Lm_d3 = 9.80;  Lm_d4 = 0.0;  Lm_d5 = 5.0;
//}

//double LHW_0[4][4];
//if (MOUSE_MODE == 1)
//{
//	LHW_0[0][0] = 1.0;		LHW_0[0][1] = 0;			LHW_0[0][2] = 0;		LHW_0[0][3] = 0;
//	LHW_0[1][0] = 0;		LHW_0[1][1] = 0;			LHW_0[1][2] = 1.0;		LHW_0[1][3] = 25.0;
//	LHW_0[2][0] = 0;		LHW_0[2][1] = -1.0;			LHW_0[2][2] = 0;		LHW_0[2][3] = 100.0;
//	LHW_0[3][0] = 0;		LHW_0[3][1] = 0;			LHW_0[3][2] = 0;		LHW_0[3][3] = 1.0;
//}
//else
//{
//	LHW_0[0][0] = 1.0;		LHW_0[0][1] = 0;			LHW_0[0][2] = 0;		LHW_0[0][3] = 0;
//	LHW_0[1][0] = 0;		LHW_0[1][1] = 0;			LHW_0[1][2] = 1.0;		LHW_0[1][3] = 2.50;
//	LHW_0[2][0] = 0;		LHW_0[2][1] = -1.0;			LHW_0[2][2] = 0;		LHW_0[2][3] = 10.0;
//	LHW_0[3][0] = 0;		LHW_0[3][1] = 0;			LHW_0[3][2] = 0;		LHW_0[3][3] = 1.0;
//}

//double LH0_W[4][4];
//HTInverse(LHW_0, LH0_W);



//double LH0_1[4][4];
//LH0_1[0][0] = cos(Lm_theta[1]);		LH0_1[0][1] = 0;			LH0_1[0][2] = -sin(Lm_theta[1]);		LH0_1[0][3] = Lm_a1*cos(Lm_theta[1]);
//LH0_1[1][0] = sin(Lm_theta[1]);		LH0_1[1][1] = 0;			LH0_1[1][2] = cos(Lm_theta[1]);		LH0_1[1][3] = Lm_a1*sin(Lm_theta[1]);
//LH0_1[2][0] = 0;						LH0_1[2][1] = -1.0;		LH0_1[2][2] = 0;						LH0_1[2][3] = Lm_d1;
//LH0_1[3][0] = 0;						LH0_1[3][1] = 0;			LH0_1[3][2] = 0;						LH0_1[3][3] = 1.0;
//double LH1_2[4][4];
//LH1_2[0][0] = cos(Lm_theta[2]);		LH1_2[0][1] = 0;			LH1_2[0][2] = -sin(Lm_theta[2]);		LH1_2[0][3] = Lm_a2*cos(Lm_theta[2]);
//LH1_2[1][0] = sin(Lm_theta[2]);		LH1_2[1][1] = 0;			LH1_2[1][2] = cos(Lm_theta[2]);		LH1_2[1][3] = Lm_a2*sin(Lm_theta[2]);
//LH1_2[2][0] = 0;						LH1_2[2][1] = -1.0;		LH1_2[2][2] = 0;						LH1_2[2][3] = Lm_d2;
//LH1_2[3][0] = 0;						LH1_2[3][1] = 0;			LH1_2[3][2] = 0;						LH1_2[3][3] = 1.0;
//double LH2_3[4][4];
//LH2_3[0][0] = cos(Lm_theta[3]);		LH2_3[0][1] = 0;			LH2_3[0][2] = -sin(Lm_theta[3]);		LH2_3[0][3] = Lm_a3*cos(Lm_theta[3]);
//LH2_3[1][0] = sin(Lm_theta[3]);		LH2_3[1][1] = 0;			LH2_3[1][2] = cos(Lm_theta[3]);		LH2_3[1][3] = Lm_a3*sin(Lm_theta[3]);
//LH2_3[2][0] = 0;						LH2_3[2][1] = -1.0;		LH2_3[2][2] = 0;						LH2_3[2][3] = Lm_d3;
//LH2_3[3][0] = 0;						LH2_3[3][1] = 0;			LH2_3[3][2] = 0;						LH2_3[3][3] = 1.0;
//double LH3_4[4][4];
//LH3_4[0][0] = cos(Lm_theta[4]);		LH3_4[0][1] = 0;			LH3_4[0][2] = -sin(Lm_theta[4]);		LH3_4[0][3] = Lm_a4*cos(Lm_theta[4]);
//LH3_4[1][0] = sin(Lm_theta[4]);		LH3_4[1][1] = 0;			LH3_4[1][2] = cos(Lm_theta[4]);		LH3_4[1][3] = Lm_a4*sin(Lm_theta[4]);
//LH3_4[2][0] = 0;						LH3_4[2][1] = -1.0;		LH3_4[2][2] = 0;						LH3_4[2][3] = Lm_d4;
//LH3_4[3][0] = 0;						LH3_4[3][1] = 0;			LH3_4[3][2] = 0;						LH3_4[3][3] = 1.0;
//double LH4_5[4][4];
//LH4_5[0][0] = cos(Lm_theta[5]);		LH4_5[0][1] = -sin(Lm_theta[5]);		LH4_5[0][2] = 0;				LH4_5[0][3] = 0;
//LH4_5[1][0] = sin(Lm_theta[5]);		LH4_5[1][1] = cos(Lm_theta[5]);		LH4_5[1][2] = 0;				LH4_5[1][3] = 0;
//LH4_5[2][0] = 0;						LH4_5[2][1] = 0;						LH4_5[2][2] = 1.0;			LH4_5[2][3] = Lm_d5;
//LH4_5[3][0] = 0;						LH4_5[3][1] = 0;						LH4_5[3][2] = 0;				LH4_5[3][3] = 1.0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////left arm end




//////////////////////////////////////////////////////////////////////////////////////////right arm
//double Rm_prev[6];
//Rm_prev[1] = 1.57079632679; Rm_prev[2] = -1.57079632679;  Rm_prev[3] = -1.57079632679;  Rm_prev[4] = -1.57079632679;  Rm_prev[5] = 0.0;

//double Rm_theta[6];
//Rm_theta[1] = 1.57079632679; Rm_theta[2] = -1.57079632679;  Rm_theta[3] = -1.57079632679;  Rm_theta[4] = -1.57079632679;  Rm_theta[5] = 0.0;

//double Rm_a1; double Rm_a2; double Rm_a3; double Rm_a4;
//double Rm_d1; double Rm_d2; double Rm_d3; double Rm_d4; double Rm_d5;

//if (MOUSE_MODE == 1)
//{
//	 Rm_a1 = 11.471;  Rm_a2 = 10.16;  Rm_a3 = 10.59;  Rm_a4 = 5.99;
//	 Rm_d1 = 14.564;  Rm_d2 = 11.246;  Rm_d3 = 98.709;  Rm_d4 = 7.08;  Rm_d5 = 51.135;
//}
//else
//{
//	 Rm_a1 = 0.0;  Rm_a2 = 0.0;  Rm_a3 = 0.0;  Rm_a4 = 0.0;
//	 Rm_d1 = 0.0;  Rm_d2 = 0.0;  Rm_d3 = 9.80;  Rm_d4 = 0.0;  Rm_d5 = 5.0;
//}


//double RHW_0[4][4];
//if (MOUSE_MODE == 1)
//{
//	RHW_0[0][0] = 1.0;		RHW_0[0][1] = 0;			RHW_0[0][2] = 0;		RHW_0[0][3] = 0;
//	RHW_0[1][0] = 0;		RHW_0[1][1] = 0;			RHW_0[1][2] = -1.0;		RHW_0[1][3] = -25.0;
//	RHW_0[2][0] = 0;		RHW_0[2][1] = 1.0;			RHW_0[2][2] = 0;		RHW_0[2][3] = 100.0;
//	RHW_0[3][0] = 0;		RHW_0[3][1] = 0;			RHW_0[3][2] = 0;		RHW_0[3][3] = 1.0;
//}
//else
//{
//	RHW_0[0][0] = 1.0;		RHW_0[0][1] = 0;			RHW_0[0][2] = 0;		RHW_0[0][3] = 0;
//	RHW_0[1][0] = 0;		RHW_0[1][1] = 0;			RHW_0[1][2] = -1.0;		RHW_0[1][3] = -2.50;
//	RHW_0[2][0] = 0;		RHW_0[2][1] = 1.0;			RHW_0[2][2] = 0;		RHW_0[2][3] = 10.0;
//	RHW_0[3][0] = 0;		RHW_0[3][1] = 0;			RHW_0[3][2] = 0;		RHW_0[3][3] = 1.0;
//}
//double RH0_W[4][4];
//HTInverse(RHW_0, RH0_W);



//double RH0_1[4][4];
//RH0_1[0][0] = cos(Rm_theta[1]);		RH0_1[0][1] = 0;			RH0_1[0][2] = sin(Rm_theta[1]);		RH0_1[0][3] = Rm_a1*cos(Rm_theta[1]);
//RH0_1[1][0] = sin(Rm_theta[1]);		RH0_1[1][1] = 0;			RH0_1[1][2] = -cos(Rm_theta[1]);		RH0_1[1][3] = Rm_a1*sin(Rm_theta[1]);
//RH0_1[2][0] = 0;						RH0_1[2][1] = 1.0;		RH0_1[2][2] = 0;						RH0_1[2][3] = Rm_d1;
//RH0_1[3][0] = 0;						RH0_1[3][1] = 0;			RH0_1[3][2] = 0;						RH0_1[3][3] = 1.0;
//double RH1_2[4][4];
//RH1_2[0][0] = cos(Rm_theta[2]);		RH1_2[0][1] = 0;			RH1_2[0][2] = sin(Rm_theta[2]);		RH1_2[0][3] = Rm_a2*cos(Rm_theta[2]);
//RH1_2[1][0] = sin(Rm_theta[2]);		RH1_2[1][1] = 0;			RH1_2[1][2] = -cos(Rm_theta[2]);		RH1_2[1][3] = Rm_a2*sin(Rm_theta[2]);
//RH1_2[2][0] = 0;						RH1_2[2][1] = 1.0;		RH1_2[2][2] = 0;						RH1_2[2][3] = Rm_d2;
//RH1_2[3][0] = 0;						RH1_2[3][1] = 0;			RH1_2[3][2] = 0;						RH1_2[3][3] = 1.0;
//double RH2_3[4][4];
//RH2_3[0][0] = cos(Rm_theta[3]);		RH2_3[0][1] = 0;			RH2_3[0][2] = sin(Rm_theta[3]);		RH2_3[0][3] = Rm_a3*cos(Rm_theta[3]);
//RH2_3[1][0] = sin(Rm_theta[3]);		RH2_3[1][1] = 0;			RH2_3[1][2] = -cos(Rm_theta[3]);		RH2_3[1][3] = Rm_a3*sin(Rm_theta[3]);
//RH2_3[2][0] = 0;						RH2_3[2][1] = 1.0;		RH2_3[2][2] = 0;						RH2_3[2][3] = Rm_d3;
//RH2_3[3][0] = 0;						RH2_3[3][1] = 0;			RH2_3[3][2] = 0;						RH2_3[3][3] = 1.0;
//double RH3_4[4][4];
//RH3_4[0][0] = cos(Rm_theta[4]);		RH3_4[0][1] = 0;			RH3_4[0][2] = sin(Rm_theta[4]);		RH3_4[0][3] = Rm_a4*cos(Rm_theta[4]);
//RH3_4[1][0] = sin(Rm_theta[4]);		RH3_4[1][1] = 0;			RH3_4[1][2] = -cos(Rm_theta[4]);		RH3_4[1][3] = Rm_a4*sin(Rm_theta[4]);
//RH3_4[2][0] = 0;						RH3_4[2][1] = 1.0;		RH3_4[2][2] = 0;						RH3_4[2][3] = Rm_d4;
//RH3_4[3][0] = 0;						RH3_4[3][1] = 0;			RH3_4[3][2] = 0;						RH3_4[3][3] = 1.0;
//double RH4_5[4][4];
//RH4_5[0][0] = cos(Rm_theta[5]);		RH4_5[0][1] = -sin(Rm_theta[5]);			RH4_5[0][2] = 0;				RH4_5[0][3] = 0;
//RH4_5[1][0] = sin(Rm_theta[5]);		RH4_5[1][1] = cos(Rm_theta[5]);				RH4_5[1][2] = 0;				RH4_5[1][3] = 0;
//RH4_5[2][0] = 0;						RH4_5[2][1] = 0;						RH4_5[2][2] = 1.0;				RH4_5[2][3] = Rm_d5;
//RH4_5[3][0] = 0;						RH4_5[3][1] = 0;						RH4_5[3][2] = 0;				RH4_5[3][3] = 1;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////right arm end


//double RM_xpos = 5.0; double RM_ypos = -9.8; double RM_zpos = 0;
//double RM_xOrient = 0.0; double RM_yOrient = 0.0; double RM_zOrient = 0.0;
//double LM_xpos = 5.0; double LM_ypos = 9.8; double LM_zpos = 0;
//double LM_xOrient = 0.0; double LM_yOrient = 0.0; double LM_zOrient = 0.0;
//RM_xOrient = 0.0;  RM_yOrient = 0.0;  RM_zOrient = 0.0;
//LM_xOrient = 0.0;  LM_yOrient = 0.0;  LM_zOrient = 0.0;
//if (MOUSE_MODE == 1)
//{
//	RM_xpos = 50.0;  RM_ypos = -50.0;  RM_zpos = 50.0;
//	LM_xpos = 5.0;  LM_ypos = 50.0;  LM_zpos = 50.0;
//
//}
//else
//{
//	 RM_xpos = 5.0;  RM_ypos = -9.8;  RM_zpos = 0;
//	 LM_xpos = 5.0;  LM_ypos = 9.8;  LM_zpos = 0;
//}







//the 7 is because of the c32 compiler in mplab x. it won't actually allocate the seventh block of memory on the stack
//i indexes the leg number. All remaining numbers refer to the coordinate frame in question. Each leg has the same DH convention
//double TLG0_1[7][4][4];
//for (i = 1; i <= 6; i++)			//0_1, DH entry 1
//{
//	TLG0_1[i][0][0] = cos(TW_theta[i][1]);		TLG0_1[i][0][1] = -sin(TW_theta[i][1])*cos(TW_alpha1);		TLG0_1[i][0][2] = sin(TW_theta[i][1])*sin(TW_alpha1); 	TLG0_1[i][0][3] = TW_a1*cos(TW_theta[i][1]);
//	TLG0_1[i][1][0] = sin(TW_theta[i][1]);		TLG0_1[i][1][1] = cos(TW_theta[i][1])*cos(TW_alpha1);		TLG0_1[i][1][2] = -cos(TW_theta[i][1])*sin(TW_alpha1);	TLG0_1[i][1][3] = TW_a1*sin(TW_theta[i][1]);
//	TLG0_1[i][2][0] = 0;						TLG0_1[i][2][1] = sin(TW_alpha1);						TLG0_1[i][2][2] = cos(TW_alpha1);						TLG0_1[i][2][3] = TW_d1;
//	TLG0_1[i][3][0] = 0;						TLG0_1[i][3][1] = 0;										TLG0_1[i][3][2] = 0;									TLG0_1[i][3][3] = 1.0;
//}
//double TLG1_2[7][4][4];
//for (i = 1; i <= 6; i++)	//DH entry 2
//{
//	TLG1_2[i][0][0] = cos(TW_theta[i][2]);		TLG1_2[i][0][1] = -sin(TW_theta[i][2])*cos(TW_alpha2);		TLG1_2[i][0][2] = sin(TW_theta[i][2])*sin(TW_alpha2); 	TLG1_2[i][0][3] = TW_a2*cos(TW_theta[i][2]);
//	TLG1_2[i][1][0] = sin(TW_theta[i][2]);		TLG1_2[i][1][1] = cos(TW_theta[i][2])*cos(TW_alpha2);		TLG1_2[i][1][2] = -cos(TW_theta[i][2])*sin(TW_alpha2);	TLG1_2[i][1][3] = TW_a2*sin(TW_theta[i][2]);
//	TLG1_2[i][2][0] = 0;						TLG1_2[i][2][1] = sin(TW_alpha2);						TLG1_2[i][2][2] = cos(TW_alpha2);						TLG1_2[i][2][3] = TW_d2;
//	TLG1_2[i][3][0] = 0;						TLG1_2[i][3][1] = 0;										TLG1_2[i][3][2] = 0;									TLG1_2[i][3][3] = 1.0;
//}
//double TLG2_3[7][4][4];
//for (i = 1; i <= 6; i++)	//DH entry 3
//{
//	TLG2_3[i][0][0] = cos(TW_theta[i][3]);		TLG2_3[i][0][1] = -sin(TW_theta[i][3])*cos(TW_alpha3);		TLG2_3[i][0][2] = sin(TW_theta[i][3])*sin(TW_alpha3); 	TLG2_3[i][0][3] = TW_a3*cos(TW_theta[i][3]);
//	TLG2_3[i][1][0] = sin(TW_theta[i][3]);		TLG2_3[i][1][1] = cos(TW_theta[i][3])*cos(TW_alpha3);		TLG2_3[i][1][2] = -cos(TW_theta[i][3])*sin(TW_alpha3);	TLG2_3[i][1][3] = TW_a3*sin(TW_theta[i][3]);
//	TLG2_3[i][2][0] = 0;						TLG2_3[i][2][1] = sin(TW_alpha3);						TLG2_3[i][2][2] = cos(TW_alpha3);						TLG2_3[i][2][3] = TW_d3;
//	TLG2_3[i][3][0] = 0;						TLG2_3[i][3][1] = 0;										TLG2_3[i][3][2] = 0;									TLG2_3[i][3][3] = 1.0;
//}

////Establish the base frame in the world frame
//double TW_B[4][4];
//TW_B[0][0] = 1.0;	TW_B[0][1] = 0;		TW_B[0][2] = 0;		TW_B[0][3] = -15;
//TW_B[1][0] = 0;		TW_B[1][1] = 1.0;	TW_B[1][2] = 0;		TW_B[1][3] = -150;
//TW_B[2][0] = 0;		TW_B[2][1] = 0;		TW_B[2][2] = 1.0;	TW_B[2][3] = 0;
//TW_B[3][0] = 0;		TW_B[3][1] = 0;		TW_B[3][2] = 0;		TW_B[3][3] = 1.0;


////Establish the zero frames in the base frame
////78.047 is the normal radius
//double TTr_0[4][4];
//TTr_0[0][0] = 1.0;	TTr_0[0][1] = 0;		TTr_0[0][2] = 0;		TTr_0[0][3] = 78.047;
//TTr_0[1][0] = 0;	TTr_0[1][1] = 1.0;		TTr_0[1][2] = 0;		TTr_0[1][3] = 0;
//TTr_0[2][0] = 0;	TTr_0[2][1] = 0;		TTr_0[2][2] = 1.0;		TTr_0[2][3] = 0;
//TTr_0[3][0] = 0;	TTr_0[3][1] = 0;		TTr_0[3][2] = 0;		TTr_0[3][3] = 1.0;
//double Base_to_0_angle = 0.0;
//double TB_0[7][4][4];
//double T0_B[7][4][4];
//for (i = 1; i <= 6; i++)
//{
//	TB_0[i][0][0] = cos(Base_to_0_angle);		TB_0[i][0][1] = -sin(Base_to_0_angle);		TB_0[i][0][2] = 0;		TB_0[i][0][3] = 0;
//	TB_0[i][1][0] = sin(Base_to_0_angle);		TB_0[i][1][1] = cos(Base_to_0_angle);		TB_0[i][1][2] = 0;		TB_0[i][1][3] = 0;
//	TB_0[i][2][0] = 0;							TB_0[i][2][1] = 0;							TB_0[i][2][2] = 1.0;	TB_0[i][2][3] = 0;
//	TB_0[i][3][0] = 0;							TB_0[i][3][1] = 0;							TB_0[i][3][2] = 0;		TB_0[i][3][3] = 1.0;
//	Multiply_HT_Matrices(TB_0[i], TTr_0, TB_0[i]);
//	HTInverse(TB_0[i], T0_B[i]);
//	Base_to_0_angle += 3.14159265359 / 3;
//}
//double TW_0[7][4][4];
//for (i = 1; i <= 6; i++)
//	Multiply_HT_Matrices(TW_B, TB_0[i], TW_0[i]);

//init_walk(T0_B);
////////////////////////////////////////////end of twitchy-style leg



