#include "display_game_functions.h"
#include "GameState.h"
#include <math.h>




void Multiply_HT_Matrices(double H1[4][4], double H2[4][4], double Hout[4][4])
{

	for (int out_r = 0; out_r < 4; out_r++)
	{
		for (int out_c = 0; out_c < 4; out_c++)
		{
			double tmp = 0;

			for (int i = 0; i < 4; i++)
			{
				//tmp.append(" + ");
				//tmp.append(H1[out_r][i]);
				//tmp.append("*");
				//tmp.append(H2[i][out_c]);
				tmp = tmp + H1[out_r][i] * H2[i][out_c];

			}

			Hout[out_r][out_c] = tmp;
		}
	}
	//Hout[3][0] = 0.0; Hout[3][1] = 0.0; Hout[3][2] = 0.0; Hout[3][3] = 1.0;

}

void HT_Matrix_Vect_Multiply(double H[4][4], double x, double y, double z, double * xout, double * yout, double * zout)
{

	*xout = H[0][0] * x + H[0][1] * y + H[0][2] * z + H[0][3];		//last number of the input 4 vector is always 1
	*yout = H[1][0] * x + H[1][1] * y + H[1][2] * z + H[1][3];
	*zout = H[2][0] * x + H[2][1] * y + H[2][2] * z + H[2][3];

}


void update_R(double R[3][3], double theta1, double theta2, double theta3)
{		
	R[0][0] = cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1) * sin(theta3);							R[0][1] = -cos(theta1)*cos(theta2)*sin(theta3) - sin(theta1)*cos(theta3);					R[0][2] = cos(theta1)*sin(theta2);
	R[1][0] = sin(theta1)*cos(theta2)*cos(theta3) + cos(theta1)*sin(theta3);							R[1][1] = -sin(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3);					R[1][2] = sin(theta1)*sin(theta2);
	R[2][0] = -sin(theta2)*cos(theta3);																	R[2][1] = sin(theta2)*sin(theta3);															R[2][2] = cos(theta2);
}

void P3Ds_to_Line3d_3(P3D p1, P3D p2, Line3d_2 * l)
{
	l->x1 = p1.x;	l->y1 = p1.y;	l->z1 = p1.z;	
	l->x2 = p2.x;	l->y2 = p2.y;	l->z2 = p2.z;
}

void initialize_pyramid(Pyramid * P, P3D V1, P3D V2, P3D V3, P3D V4)
{
	P->num_edges = 6;
	HTmatrix I;
	I.H[0][0] = 1; I.H[0][1] = 0; I.H[0][2] = 0; I.H[0][3] = 0;
	I.H[1][0] = 0; I.H[1][1] = 1; I.H[1][2] = 0; I.H[1][3] = 0;
	I.H[2][0] = 0; I.H[2][1] = 0; I.H[2][2] = 1; I.H[2][3] = 0;
	I.H[3][0] = 0; I.H[3][1] = 0; I.H[3][2] = 0; I.H[3][3] = 1;
	copy_HT(&(P->HW_B), &I);
	int i;
	for (i = 0; i < 4; i++)
		copy_HT(&(P->HB_v[i]), &I);
	HT_load_point(&(P->HB_v[0]), V1);
	HT_load_point(&(P->HB_v[1]), V2);
	HT_load_point(&(P->HB_v[2]), V3);
	HT_load_point(&(P->HB_v[3]), V4);
	//			v1
	//
	//			v4	
	//
	//	 v2				v3
	//
	for (i = 0; i < 4; i++)
		HT_Multiply(P->HW_B, P->HB_v[i], &(P->HW_v[i]));
}



/*
xc, yc, zc are the coordinates of the camera in the worldframe
R is the rotation w world on bottom and camera on top
outputs by reference to the _out vector
*/
void INV_and_Multiply_HT_matrix_by_vector(double R[3][3], double xc, double yc, double zc, double xin, double yin, double zin, double * x_out, double * y_out, double * z_out)
{

	//shift world coordinates to camera base/0 frame first, and then multiply by T with 0 on the bototm and C on top (C for camera) which is the inverse of the matrix T C on bott and 0 on top that
	//you get as a composition of DH rotations and translations.


	
	xin = xin - xc;
	yin = yin - yc;
	zin = zin - zc;

	*x_out = R[0][0] * xin + R[1][0] * yin + R[2][0] * zin;		
	*y_out = R[0][1] * xin + R[1][1] * yin + R[2][1] * zin;
	*z_out = R[0][2] * xin + R[1][2] * yin + R[2][2] * zin;



	//if you want to strafe around a target, you can add offsets once you're in the CAMERA FRAME
}






void strafe_left_right(double theta1, double *xc, double *yc, double st_v)
{

//	R[0][0] = cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1) * sin(theta3);			
//	R[1][0] = sin(theta1)*cos(theta2)*cos(theta3) + cos(theta1)*sin(theta3);				
//	R[2][0] = -sin(theta2)*cos(theta3);														


	double x_0 = sin(theta1) * st_v;
	double y_0 = -cos(theta1) * st_v;
	*xc = *xc + x_0;	//in the world frame
	*yc = *yc + y_0;	//in the world frame
		
}




//lock on


void move_forward_backward(double theta1, double *xc, double *yc, double fw_v)
{

	//R[0][2] = cos(theta1)*sin(theta2);
	//R[1][2] = sin(theta1)*sin(theta2);
	//R[2][2] = cos(theta2);
//	double x_0 = R[0][2] * fw_v;		//start with a vector in the camera frame, i.e. forward. This is a fake camera frame, where the camera is always level. This prevents the player from flying.
//	double y_0 = R[1][2] * fw_v;
//	double z_0 = R[2][2] * fw_v;


	double x_0 = -cos(theta1) * fw_v;		//start with a vector in the camera frame, i.e. forward. This is a fake camera frame, where the camera is always level. This prevents the player from flying.
	double y_0 = -sin(theta1) * fw_v;
		
	*xc = *xc + x_0;	//in the world frame
	*yc = *yc + y_0;	//in the world frame
	
}




//This code came from an intial misunderstanding of how to express the camera rotations.


/*
void Rotate_camera_about_y(double R[3][3], double y_angle)			//this is pre-multiplied, since it's a rotation about the world
{
double R_c[3][3];
for (int i = 0; i < 3; i++)
{
for (int j = 0; j < 3; j++)
{
R_c[i][j] = R[i][j];
}
}
R[0][0] = cos(y_angle) * R_c[0][0] + sin(y_angle) * R_c[2][0];
R[0][1] = cos(y_angle) * R_c[0][1] + sin(y_angle) * R_c[2][1];
R[0][2] = cos(y_angle) * R_c[0][2] + sin(y_angle) * R_c[2][2];
//second row unchanged
R[2][0] = -1.0 * sin(y_angle) * R_c[0][0] + cos(y_angle) * R_c[2][0];
R[2][1] = -1.0 * sin(y_angle) * R_c[0][1] + cos(y_angle) * R_c[2][1];
R[2][2] = -1.0 * sin(y_angle) * R_c[0][2] + cos(y_angle) * R_c[2][2];
}


void Rotate_camera_about_x(double R[3][3], double x_angle)			//post multiplied, since it's a rotation about the current
{
double R_c[3][3];
for (int i = 0; i < 3; i++)
{
for (int j = 0; j < 3; j++)
{
R_c[i][j] = R[i][j];
}
}
R[0][1] = R_c[0][1] * cos(x_angle) + R_c[0][2] * sin(x_angle);
R[1][1] = R_c[1][1] * cos(x_angle) + R_c[1][2] * sin(x_angle);
R[2][1] = R_c[2][1] * cos(x_angle) + R_c[2][2] * sin(x_angle);
//second column unchanged
R[0][2] = -1.0 * R_c[0][1] * sin(x_angle) + R_c[0][2] * cos(x_angle);
R[1][2] = -1.0 * R_c[1][1] * sin(x_angle) + R_c[1][2] * cos(x_angle);
R[2][2] = -1.0 * R_c[2][1] * sin(x_angle) + R_c[2][2] * cos(x_angle);
}
*/




//void Twitchy_Leg_Limit_Joints(double * Theta)
//{
//	
//
//	if (Theta[1] > 3.14159265359 + 1.57079632679 - .01)
//		Theta[1] = 3.14159265359 + 1.57079632679 - .01;
//	else if (Theta[1] < 3.14159265359 - 1.57079632679 + .01)
//		Theta[1] = 3.14159265359 - 1.57079632679 + .01;
//	
//	if (Theta[2] > 3.14159265359 + 1.57079632679 - .01)
//		Theta[2] = 3.14159265359 + 1.57079632679 - .01;
//	else if (Theta[2] < 3.14159265359 - 1.57079632679 + .01)
//		Theta[2] = 3.14159265359 - 1.57079632679 + .01;
//
//	if (Theta[3] > -1.57079632679 + 1.57079632679 - .01)
//		Theta[3] = -1.57079632679 + 1.57079632679 - .01;
//	else if (Theta[3] < -1.57079632679 - 1.57079632679 + .01)
//		Theta[3] = -1.57079632679 - 1.57079632679 + .01;
//
//}
//
//void Twitchy_Leg_Forward_Kinematics(double * Theta, double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], double alpha1, double alpha2, double alpha3, double a1, double a2, double a3, double H0_2[4][4], double H0_3[4][4])
//{
//	H0_1[0][0] = cos(Theta[1]);		H0_1[0][1] = -sin(Theta[1])*cos(alpha1);		H0_1[0][2] = sin(Theta[1])*sin(alpha1); 	H0_1[0][3] = a1*cos(Theta[1]);
//	H0_1[1][0] = sin(Theta[1]);		H0_1[1][1] = cos(Theta[1])*cos(alpha1);		H0_1[1][2] = -cos(Theta[1])*sin(alpha1);	H0_1[1][3] = a1*sin(Theta[1]);
//										
//	
//	H1_2[0][0] = cos(Theta[2]);		H1_2[0][1] = -sin(Theta[2])*cos(alpha2);		H1_2[0][2] = sin(Theta[2])*sin(alpha2); 	H1_2[0][3] = a2*cos(Theta[2]);
//	H1_2[1][0] = sin(Theta[2]);		H1_2[1][1] = cos(Theta[2])*cos(alpha2);		H1_2[1][2] = -cos(Theta[2])*sin(alpha2);	H1_2[1][3] = a2*sin(Theta[2]);
//	
//	
//	H2_3[0][0] = cos(Theta[3]);		H2_3[0][1] = -sin(Theta[3])*cos(alpha3);		H2_3[0][2] = sin(Theta[3])*sin(alpha3); 	H2_3[0][3] = a3*cos(Theta[3]);
//	H2_3[1][0] = sin(Theta[3]);		H2_3[1][1] = cos(Theta[3])*cos(alpha3);		H2_3[1][2] = -cos(Theta[3])*sin(alpha3);	H2_3[1][3] = a3*sin(Theta[3]);
//			
//
//	Multiply_HT_Matrices(H0_1, H1_2, H0_2);
//	Multiply_HT_Matrices(H0_2, H2_3, H0_3);
//
//}
//
//void Twitchy_Leg_Inverse_Kinematics(double X_desired, double Y_desired, double Z_desired, double XROT_desired, double YROT_desired, double ZROT_desired,
//	double Theta[4], double Theta_previous[4],
//	double H0_1[4][4], double H0_2[4][4], double H0_3[4][4],
//	double J[6][3], double J_T[3][6],
//	double * XVS, double * YVS, double * ZVS, double * XROTVS, double * YROTVS, double * ZROTVS, double weight)
//{
//
//
//	double d1[3]; double d2[3]; double d3[3]; 
//	double z0[3]; double z1[3]; double z2[3]; 
//
//	z0[0] = 0; z0[1] = 0; z0[2] = 1;
//
//
//	z1[0] = H0_1[0][2]; z1[1] = H0_1[1][2]; z1[2] = H0_1[2][2];
//	z2[0] = H0_2[0][2]; z2[1] = H0_2[1][2]; z2[2] = H0_2[2][2];
//	
//	int i;
//	for (i = 0; i < 3; i++)
//	{
//		d1[i] = H0_3[i][3];
//		d2[i] = H0_3[i][3] - H0_1[i][3];
//		d3[i] = H0_3[i][3] - H0_2[i][3];
//	}
//
//	double res[3];
//	Cross_Product(z0, d1, res);
//	for (i = 0; i < 3; i++)
//		J[i][0] = res[i];
//	for (i = 3; i < 6; i++)
//		J[i][0] = z0[i];
//
//	Cross_Product(z1, d2, res);
//	for (i = 0; i < 3; i++)
//		J[i][1] = res[i];
//	for (i = 3; i < 6; i++)
//		J[i][1] = z1[i];
//
//	Cross_Product(z2, d3, res);
//	for (i = 0; i < 3; i++)
//		J[i][2] = res[i];
//	for (i = 3; i < 6; i++)
//		J[i][2] = z2[i];
//
//	double JointVel[6];
//	for (i = 1; i < 6; i++)
//		JointVel[i] = Theta[i] - Theta_previous[i];
//
//
//		
//	TwitchyJacobianTranspose(J, J_T);
//
//
//
//	*XVS = weight*(X_desired - H0_3[0][3]); *YVS = weight*(Y_desired - H0_3[1][3]); *ZVS = weight*(Z_desired - H0_3[2][3]);
//	*XROTVS = 0; *YROTVS = 0; *ZROTVS = 0;
//
//
//
//}
//
//void TwitchyJacobianTranspose(double HTin[6][3], double HTout[3][6])
//{
//	for (int r = 0; r < 6; r++)
//	{
//		for (int c = 0; c < 3; c++)
//		{
//			HTout[r][c] = HTin[c][r];
//		}
//	}
//}
//
//void Mouse_Left_Arm_Forward_Kinematics(double * M_theta, double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], double H3_4[4][4], double H4_5[4][4], double M_a1, double M_a2, double M_a3, double M_a4, double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4])
//{
//	H0_1[0][0] = cos(M_theta[1]);		H0_1[0][2] = -sin(M_theta[1]);		H0_1[0][3] = M_a1*cos(M_theta[1]);
//	H0_1[1][0] = sin(M_theta[1]);		H0_1[1][2] = cos(M_theta[1]);		H0_1[1][3] = M_a1*sin(M_theta[1]);
//
//	H1_2[0][0] = cos(M_theta[2]);		H1_2[0][2] = -sin(M_theta[2]);		H1_2[0][3] = M_a2*cos(M_theta[2]);
//	H1_2[1][0] = sin(M_theta[2]);		H1_2[1][2] = cos(M_theta[2]);		H1_2[1][3] = M_a2*sin(M_theta[2]);
//	H2_3[0][0] = cos(M_theta[3]);		H2_3[0][2] = -sin(M_theta[3]);		H2_3[0][3] = M_a3*cos(M_theta[3]);
//	H2_3[1][0] = sin(M_theta[3]);		H2_3[1][2] = cos(M_theta[3]);		H2_3[1][3] = M_a3*sin(M_theta[3]);
//	H3_4[0][0] = cos(M_theta[4]);		H3_4[0][2] = -sin(M_theta[4]);		H3_4[0][3] = M_a4*cos(M_theta[4]);
//	H3_4[1][0] = sin(M_theta[4]);		H3_4[1][2] = cos(M_theta[4]);		H3_4[1][3] = M_a4*sin(M_theta[4]);
//	H4_5[0][0] = cos(M_theta[5]);		H4_5[0][1] = -sin(M_theta[5]);
//	H4_5[1][0] = sin(M_theta[5]);		H4_5[1][1] = cos(M_theta[5]);
//
//
//	//	double HW_1[4][4];
//	//	Multiply_HT_Matrices(HW_0, H0_1, HW_1);
//
//	Multiply_HT_Matrices(H0_1, H1_2, H0_2);
//	Multiply_HT_Matrices(H0_2, H2_3, H0_3);
//	Multiply_HT_Matrices(H0_3, H3_4, H0_4);
//	Multiply_HT_Matrices(H0_4, H4_5, H0_5);
//
//
//}
//
//void Limit_Left_Arm_Joints(double * Lm_theta)
//{
//	if (Lm_theta[1] > -1.57079632679){ Lm_theta[1] = -1.57079632679; }
//	else if (Lm_theta[1] < -3 * 1.57079632679){ Lm_theta[1] = -3 * 1.57079632679; }
//	if (Lm_theta[2] < 1.17079632679){ Lm_theta[2] = 1.17079632679; }
//	else if (Lm_theta[2] > 1.17079632679 + 2 * 1.57079632679){ Lm_theta[2] = 1.17079632679 + 2 * 1.57079632679; }
//	if (Lm_theta[3] < 0){ Lm_theta[3] = 0; }
//	else if (Lm_theta[3] > 2 * 1.57079632679){ Lm_theta[3] = 2 * 1.57079632679; }
//	if (Lm_theta[3] < 0){ Lm_theta[3] = 0; }
//	else if (Lm_theta[3] > 2 * 1.57079632679){ Lm_theta[3] = 2 * 1.57079632679; }
//	if (Lm_theta[4] < 0){ Lm_theta[4] = 0; }
//	else if (Lm_theta[4] > 2 * 1.57079632679){ Lm_theta[4] = 2 * 1.57079632679; }
//	if (Lm_theta[5] < -1.57079632679){ Lm_theta[5] = -1.57079632679; }
//	else if (Lm_theta[5] > 1.57079632679){ Lm_theta[5] = 1.57079632679; }
//}
//
//void Limit_Right_Arm_Joints(double * Rm_theta)
//{
//	if (Rm_theta[1] < 1.57079632679){ Rm_theta[1] = 1.57079632679; }
//	else if (Rm_theta[1] > 3 * 1.57079632679){ Rm_theta[1] = 3 * 1.57079632679; }
//	if (Rm_theta[2] > -1.17079632679){ Rm_theta[2] = -1.17079632679; }
//	else if (Rm_theta[2] < -(1.17079632679 + 2 * 1.57079632679)){ Rm_theta[2] = -(1.17079632679 + 2 * 1.57079632679); }
//	if (Rm_theta[3] > 0){ Rm_theta[3] = 0; }
//	else if (Rm_theta[3] < -2 * 1.57079632679){ Rm_theta[3] = -2 * 1.57079632679; }
//	if (Rm_theta[4] > 0){ Rm_theta[4] = 0; }
//	else if (Rm_theta[4] < -2 * 1.57079632679){ Rm_theta[4] = -2 * 1.57079632679; }
//	if (Rm_theta[5] > 1.57079632679){ Rm_theta[5] = 1.57079632679; }
//	else if (Rm_theta[5] < -1.57079632679){ Rm_theta[5] = -1.57079632679; }
//}
//
//void Mouse_Right_Arm_Forward_Kinematics(double * M_theta, double H0_1[4][4], double H1_2[4][4], double H2_3[4][4], double H3_4[4][4], double H4_5[4][4], double M_a1, double M_a2, double M_a3, double M_a4, double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4])
//{
//	H0_1[0][0] = cos(M_theta[1]);		H0_1[0][2] = sin(M_theta[1]);		H0_1[0][3] = M_a1*cos(M_theta[1]);
//	H0_1[1][0] = sin(M_theta[1]);		H0_1[1][2] = -cos(M_theta[1]);		H0_1[1][3] = M_a1*sin(M_theta[1]);
//
//	H1_2[0][0] = cos(M_theta[2]);		H1_2[0][2] = sin(M_theta[2]);		H1_2[0][3] = M_a2*cos(M_theta[2]);
//	H1_2[1][0] = sin(M_theta[2]);		H1_2[1][2] = -cos(M_theta[2]);		H1_2[1][3] = M_a2*sin(M_theta[2]);
//
//	H2_3[0][0] = cos(M_theta[3]);		H2_3[0][2] = sin(M_theta[3]);		H2_3[0][3] = M_a3*cos(M_theta[3]);
//	H2_3[1][0] = sin(M_theta[3]);		H2_3[1][2] = -cos(M_theta[3]);		H2_3[1][3] = M_a3*sin(M_theta[3]);
//
//	H3_4[0][0] = cos(M_theta[4]);		H3_4[0][2] = sin(M_theta[4]);		H3_4[0][3] = M_a4*cos(M_theta[4]);
//	H3_4[1][0] = sin(M_theta[4]);		H3_4[1][2] = -cos(M_theta[4]);		H3_4[1][3] = M_a4*sin(M_theta[4]);
//
//	H4_5[0][0] = cos(M_theta[5]);		H4_5[0][1] = sin(M_theta[5]);
//	H4_5[1][0] = sin(M_theta[5]);		H4_5[1][1] = -cos(M_theta[5]);
//
//
////	double HW_1[4][4];
////	Multiply_HT_Matrices(HW_0, H0_1, HW_1);
//	
//	Multiply_HT_Matrices(H0_1, H1_2, H0_2);
//	Multiply_HT_Matrices(H0_2, H2_3, H0_3);
//	Multiply_HT_Matrices(H0_3, H3_4, H0_4);
//	Multiply_HT_Matrices(H0_4, H4_5, H0_5);
//	
//
//}
//void Mouse_Arm_Inverse_Kinematics(double X_desired, double Y_desired, double Z_desired, double XROT_desired, double YROT_desired, double ZROT_desired, 
//									double M_theta[6], double M_theta_previous[6],
//									double H0_1[4][4], double H0_2[4][4], double H0_3[4][4], double H0_4[4][4], double H0_5[4][4], 
//									double M_J[6][5], double TM_J[5][6],
//									double * XVS, double * YVS, double * ZVS, double * XROTVS, double * YROTVS, double * ZROTVS, double weight)
//{
//	
//	double d1[3]; double d2[3]; double d3[3]; double d4[3]; double d5[3];
//	double z0[3]; double z1[3]; double z2[3]; double z3[3]; double z4[3];
//
//	z0[0] = 0; z0[1] = 0; z0[2] = 1;
//
//
//	z1[0] = H0_1[0][2]; z1[1] = H0_1[1][2]; z1[2] = H0_1[2][2];
//	z2[0] = H0_2[0][2]; z2[1] = H0_2[1][2]; z2[2] = H0_2[2][2];
//	z3[0] = H0_3[0][2]; z3[1] = H0_3[1][2]; z3[2] = H0_3[2][2];
//	z4[0] = H0_4[0][2]; z4[1] = H0_4[1][2]; z4[2] = H0_4[2][2];
//		
//	int i;
//	for (i = 0; i < 3; i++)
//	{
//		d1[i] = H0_5[i][3];
//		d2[i] = H0_5[i][3] - H0_1[i][3];
//		d3[i] = H0_5[i][3] - H0_2[i][3];
//		d4[i] = H0_5[i][3] - H0_3[i][3];
//		d5[i] = H0_5[i][3] - H0_4[i][3];
//	}
//
//	double res[3];
//	Cross_Product(z0, d1, res);
//	for (i = 0; i < 3; i++)
//		M_J[i][0] = res[i];
//	for (i = 3; i < 6; i++)
//		M_J[i][0] = z0[i];
//
//	Cross_Product(z1, d2, res);
//	for (i = 0; i < 3; i++)
//		M_J[i][1] = res[i];
//	for (i = 3; i < 6; i++)
//		M_J[i][1] = z1[i];
//
//	Cross_Product(z2, d3, res);
//	for (i = 0; i < 3; i++)
//		M_J[i][2] = res[i];
//	for (i = 3; i < 6; i++)
//		M_J[i][2] = z2[i];
//
//	Cross_Product(z3, d4, res);
//	for (i = 0; i < 3; i++)
//		M_J[i][3] = res[i];
//	for (i = 3; i < 6; i++)
//		M_J[i][3] = z3[i];
//
//	Cross_Product(z4, d5, res);
//	for (i = 0; i < 3; i++)
//		M_J[i][4] = res[i];
//	for (i = 3; i < 6; i++)
//		M_J[i][4] = z4[i];
//
//
//	double JointVel[6];
//	for (i = 1; i < 6; i++)
//		JointVel[i] = M_theta[i] - M_theta_previous[i];
//
//
//
//	///////////////////////////////jacobian transpose/joint angle updates
//
//
//
//	/////////////////Linear Velocity Calkoo-layshun
//	//double Rxv = M_J[0][0] * JointVel[1] + M_J[0][1] * JointVel[2] + M_J[0][2] * JointVel[3] + M_J[0][3] * JointVel[4] + M_J[0][4] * JointVel[5];
//	//double Ryv = M_J[1][0] * JointVel[1] + M_J[1][1] * JointVel[2] + M_J[1][2] * JointVel[3] + M_J[1][3] * JointVel[4] + M_J[1][4] * JointVel[5];
//	//double Rzv = M_J[2][0] * JointVel[1] + M_J[2][1] * JointVel[2] + M_J[2][2] * JointVel[3] + M_J[2][3] * JointVel[4] + M_J[2][4] * JointVel[5];
//
//
//	
//	//double Transp_J[5][6];
//	MouseJacobianTranspose(M_J, TM_J);
//	
//	*XVS = weight*(X_desired - H0_5[0][3]); *YVS = weight*(Y_desired - H0_5[1][3]); *ZVS = weight*(Z_desired - H0_5[2][3]);
//	*XROTVS= 0; *YROTVS = 0; *ZROTVS = 0;
//	
//	
//	
//
//}
//
//void HTInverse(double HTin[4][4], double HTout[4][4])
//{
//	for (int r = 0; r < 3; r++)
//	{
//		for (int c = 0; c < 3; c++)
//		{
//			HTout[r][c] = HTin[c][r];
//		}
//	}
//	HTout[0][3] = -(HTout[0][0] * HTin[0][3] + HTout[0][1] * HTin[1][3] + HTout[0][2] * HTin[2][3]);
//	HTout[1][3] = -(HTout[1][0] * HTin[0][3] + HTout[1][1] * HTin[1][3] + HTout[1][2] * HTin[2][3]);
//	HTout[2][3] = -(HTout[2][0] * HTin[0][3] + HTout[2][1] * HTin[1][3] + HTout[2][2] * HTin[2][3]);
//
//	HTout[3][0] = 0; HTout[3][1] = 0; HTout[3][2] = 0; HTout[3][3] = 1.0;		
//}
//
//void MouseJacobianTranspose(double HTin[6][5], double HTout[5][6])
//{
//	for (int r = 0; r < 5; r++)
//	{
//		for (int c = 0; c < 6; c++)
//		{
//			HTout[r][c] = HTin[c][r];
//		}
//	}
//}
//
//void Cross_Product(double * u, double * v, double * res)
//{
//	res[0] = u[1] * v[2] - u[2] * v[1];
//	res[1] = u[2]*v[0] - u[0]*v[2];
//	res[2] = u[0] * v[1] - u[1] * v[0];
//}
