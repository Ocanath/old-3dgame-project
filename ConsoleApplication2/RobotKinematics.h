#ifndef ROBOTKINEMATICS_H
#define ROBOTKINEMATICS_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


typedef struct DH_entry
{
	double d;
	double a;
	double alpha;
	double theta;
}DH_entry;

typedef struct HTmatrix
{
	double H[4][4];
}HTmatrix;

typedef struct KinematicChain
{
	HTmatrix HW_B;
	HTmatrix HB_0;
	DH_entry * DH_Table;
	HTmatrix * HTadj;
	HTmatrix * HTbase;
	double ** J;	
	double ** J_Transpose;
	int size;
	int array_size;
}KinematicChain;



typedef struct P3D
{
	double x;
	double y;
	double z;
}P3D;

typedef struct Vect3
{
	double u;
	double v;
	double w;
}Vect3;





void copy_HT(HTmatrix * Hdes, HTmatrix * Hsource);
void HT_Multiply(HTmatrix H1, HTmatrix H2, HTmatrix * Hout);
void HT_Inverse(HTmatrix HTin, HTmatrix * HTout);
void HT_Point_Multiply(HTmatrix Hout_in, P3D Pin, P3D * Pout);
void Vect3_CrossProduct(Vect3 in1, Vect3 in2, Vect3 * res);
void HT_load_point(HTmatrix * H, P3D p);

void init_jacobian(double *** J, double *** J_Transpose, int size);
void init_forward_kinematics(DH_entry * DH_Table, HTmatrix * HTadj, HTmatrix * HTbase, int size);

void inverse_kinematics(P3D Pdes, DH_entry * DH_Table, double ** J, double ** J_Transpose, HTmatrix * HTbase, int size, double weight);
void forward_kinematics(DH_entry * DH_Table, HTmatrix * HTadj, HTmatrix * HTbase, int size);

void inverse_kinematics_KC(P3D Pdes, KinematicChain * R, double weight);
void forward_kinematics_KC(KinematicChain * R);
void init_forward_kinematics_KC(KinematicChain * R);
HTmatrix Hx(float angle);
HTmatrix Hy(float angle);
HTmatrix Hz(float angle);
HTmatrix Hscale(float scale);

#endif