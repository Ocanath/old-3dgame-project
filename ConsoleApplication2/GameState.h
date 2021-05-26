#ifndef GAMESTATE_H
#define GAMESTATE_H

#include <math.h>
#include "RobotKinematics.h"
//for now fixed camera 
#include <vector>
using namespace std;

typedef struct 
{
	double x;	//box center x value
	double y;	//box center y value
	double z;	//box center z value

	double width;
	double height; 
	double depth;

	//double x_rot;		//angle of rotation about x
	//double y_rot;		//about y
	//double z_rot;		//about z
		

	double R[3][3];		//ROTATION MATRIX describing the transformation R with world on bottom, object frame on top
	
	float vx;
	float vy;
	float vz;
	

	unsigned int anim_step;		//not useful

}Box3d;



typedef struct
{
	double kx;
	double ky;
	double kz;

	double dx;
	double dy; 
	double dz;

	double length;
	
}Line3d_1;

typedef struct
{
	double x2;
	double y2;
	double z2;

	double x1;
	double y1;
	double z1;

	double length;

}Line3d_2;


typedef struct
{
	HTmatrix HW_B;
	HTmatrix HB_v[4];
	HTmatrix HW_v[4];
	Line3d_2 edges[6];
	int num_edges;
}Pyramid;



typedef struct
{

	Box3d boundary;		//contains x,y,z	
	double depth_offset;
	double lamda;

	vector<P3D> renderQueue;
	vector<int> map;
	//use unit focal length for now
	//width = 640, height = 480, so the viewing angle is 
	//nearly 180 degrees. a larger 
	//focal length may be desired

}Player;


typedef struct
{
	P3D v[3];
}Triangle;

typedef struct
{
	double x;
	double y;
	double z;	
}Point_3d;

typedef struct
{
	double d;
	double a;
	double alpha;
	double theta;

}DH_TABLE_ROW;


typedef struct
{
	double L[6];
}RCon;

void initialize_pyramid(Pyramid * P, P3D V1, P3D V2, P3D V3, P3D V4);

void INV_and_Multiply_HT_matrix_by_vector(double R[3][3], double xc, double yc, double zc, double xin, double yin, double zin, double * x_out, double * y_out, double * z_out);
void update_R(double R[3][3], double theta1, double theta2, double theta3);
void move_forward_backward(double theta1, double *xc, double *yc, double fw_v);
void strafe_left_right(double theta1, double *xc, double *yc, double st_v);

void Multiply_HT_Matrices(double H1[4][4], double H2[4][4], double Hout[4][4]);
void Cross_Product(double * u, double * v, double * res);
void MouseJacobianTranspose(double HTin[6][5], double HTout[5][6]);
void HTInverse(double HTin[4][4], double HTout[4][4]);
void HT_Matrix_Vect_Multiply(double H[4][4], double x, double y, double z, double * xout, double * yout, double * zout);
void Twitchy_Leg_Limit_Joints(double * Theta);

void Limit_Left_Arm_Joints(double * Lm_theta);
void Limit_Right_Arm_Joints(double * Rm_theta);

void Mouse_Left_Arm_Forward_Kinematics(double * M_theta, double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], double H3_4[4][4], double H4_5[4][4], double M_a1, double M_a2, double M_a3, double M_a4, double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4]);
void Mouse_Right_Arm_Forward_Kinematics(double * M_theta, double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], double H3_4[4][4], double H4_5[4][4], double M_a1, double M_a2, double M_a3, double M_a4, double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4]);

void Mouse_Arm_Inverse_Kinematics(double X_desired, double Y_desired, double Z_desired, double XROT_desired, double YROT_desired, double ZROT_desired,
	double M_theta[6], double M_theta_previous[6],
	double H0_1[4][4], double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4],
	double M_J[6][5], double TM_J[5][6],
	double * XVS, double * YVS, double * ZVS, double * XROTVS, double * YROTVS, double * ZROTVS, double weight);
//void Rotate_camera_about_x(double R[3][3], double x_angle);
//void Rotate_camera_about_y(double R[3][3], double y_angle);





void Twitchy_Leg_Forward_Kinematics(double * Theta,
				double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], 
				double alpha1, double alpha2, double alpha3, double a1, double a2, double a3, 
				double H0_2[4][4], double H0_3[4][4]);

void Twitchy_Leg_Inverse_Kinematics(double X_desired, double Y_desired, double Z_desired, double XROT_desired, double YROT_desired, double ZROT_desired,
	double Theta[4], double Theta_previous[4],
	double H0_1[4][4], double H0_2[4][4], double H0_3[4][4],
	double J[6][3], double J_T[3][6],
	double * XVS, double * YVS, double * ZVS, double * XROTVS, double * YROTVS, double * ZROTVS, double weight);

void TwitchyJacobianTranspose(double HTin[6][3], double HTout[3][6]);


#endif //GAMESTATE_H