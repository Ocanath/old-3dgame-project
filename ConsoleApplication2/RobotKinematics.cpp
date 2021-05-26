#include "RobotKinematics.h"



/*
Wrapper function for the kinematic chain type. See init_forward_kinematics for details
*/
void init_forward_kinematics_KC(KinematicChain * R)
{
	init_forward_kinematics(R->DH_Table, R->HTadj, R->HTbase, R->size);
}
/*
initializes the list of homogeneous transformation matrices corresponding to a single kinematic chain.
IMPORTANT NOTE!!!:  HTbase starts at index 1, and relates the INDEX frame to the BASE frame: i.e. HTlist[2] will be H0_2.
Similarly, HTadj relates the INDEX to the PREVIOUS frame; i.e. HTadj[3] is H2_3.
This rule will apply for all further functions.
NOTE 2:				Should be obvious, but this should only be called once outside the main loop
INPUTS: DH table, a blank HT list to initialize, and the number of joints in the chain
OUTPUTS: To HTadj and HTbase: matrices for a given kinematic chain
*/
void init_forward_kinematics(DH_entry * DH_Table, HTmatrix * HTadj, HTmatrix * HTbase, int size)
{
	int i;
	for (i = 1; i <= size; i++)
	{
		//[row][column]
		HTadj[i].H[0][0] = cos(DH_Table[i].theta);		HTadj[i].H[0][1] = -1.0*sin(DH_Table[i].theta)*cos(DH_Table[i].alpha);		HTadj[i].H[0][2] = sin(DH_Table[i].theta)*sin(DH_Table[i].alpha); 	HTadj[i].H[0][3] = DH_Table[i].a*cos(DH_Table[i].theta);
		HTadj[i].H[1][0] = sin(DH_Table[i].theta);		HTadj[i].H[1][1] = cos(DH_Table[i].theta)*cos(DH_Table[i].alpha);		HTadj[i].H[1][2] = -1.0*cos(DH_Table[i].theta)*sin(DH_Table[i].alpha);	HTadj[i].H[1][3] = DH_Table[i].a*sin(DH_Table[i].theta);
		HTadj[i].H[2][0] = 0;							HTadj[i].H[2][1] = sin(DH_Table[i].alpha);								HTadj[i].H[2][2] = cos(DH_Table[i].alpha);								HTadj[i].H[2][3] = DH_Table[i].d;
		HTadj[i].H[3][0] = 0;							HTadj[i].H[3][1] = 0;													HTadj[i].H[3][2] = 0;													HTadj[i].H[3][3] = 1.0;
	}
	//base case
	copy_HT(&(HTbase[1]), &(HTadj[1]));
	for (i = 2; i <= size; i++)
		HT_Multiply(HTbase[i - 1], HTadj[i], &(HTbase[i]));
}


/*
Wrapper function for the kinematic chain type. See forward_kinematics for details
*/
void forward_kinematics_KC(KinematicChain * R)
{
	forward_kinematics(R->DH_Table, R->HTadj, R->HTbase, R->size);
}
/*
calculates forward kinematics at each time step by using theta to update HTadj, then recalculatin HTbase
INPUTS: DH table, HTadj, chain size
OUTPUTS: updates adj and base.
*/
void forward_kinematics(DH_entry * DH_Table, HTmatrix * HTadj, HTmatrix * HTbase, int size)
{
	int i;
	for (i = 1; i <= size; i++)
	{
		//update all variable entries (only rotational joints for this framework, although uncommenting the d line below will allow for FK for prismatic joints. 
		//IK for prismatic will not be supported
		HTadj[i].H[0][0] = cos(DH_Table[i].theta);		HTadj[i].H[0][1] = -1.0*sin(DH_Table[i].theta)*cos(DH_Table[i].alpha);		HTadj[i].H[0][2] = sin(DH_Table[i].theta)*sin(DH_Table[i].alpha); 	HTadj[i].H[0][3] = DH_Table[i].a*cos(DH_Table[i].theta);
		HTadj[i].H[1][0] = sin(DH_Table[i].theta);		HTadj[i].H[1][1] = cos(DH_Table[i].theta)*cos(DH_Table[i].alpha);		HTadj[i].H[1][2] = -1.0*cos(DH_Table[i].theta)*sin(DH_Table[i].alpha);	HTadj[i].H[1][3] = DH_Table[i].a*sin(DH_Table[i].theta);
		//HTadj[i].H[2][3] = DH_Table[i].d;
	}

	copy_HT(&(HTbase[1]), &(HTadj[1]));
	for (i = 2; i <= size; i++)
		HT_Multiply(HTbase[i - 1], HTadj[i], &(HTbase[i]));
}



/*
dynamic memory allocation for jacobian. Note the triple pointer is for pass by reference
NOTE: This function should be called only ONCE, OUTSIDE of the main loop.
example input format:
double ** J; double ** J_Transpose;
init_jacobian(&J, &J_Transpose, size);
triple pointer is for pass-by-reference allocation (in case you get confused in the future :)

INPUTS: c-style pass by reference jacobian and transpose arrays, and the chain size (NOT ARRAY).
OUTPUTS: dynamically allocates space for jacobians
*/
void init_jacobian(double *** J, double *** J_Transpose, int size)
{
	//6DOF for a coordinate frame in R3 (position and orientation)
	int dim = 6;
	//Jacobian has 6 rows and the number of joints columns
	*J = (double **)malloc(dim * sizeof(double *));
	int i;
	for (i = 0; i < dim; i++)
		(*J)[i] = (double *)malloc(size * sizeof(double));
	//Transpose of Jacobian has number of joints rows and 6 columns
	*J_Transpose = (double **)malloc(size * sizeof(double *));
	for (i = 0; i < size; i++)
		(*J_Transpose)[i] = (double *)malloc(dim*sizeof(double));
}

/*
matout = Transpose(matin). Helper function for IK only
INPUTS: J, chain size (NOT ARRAY SIZE)
OUTPUTS: J_Transpose
*/
void JacobianTranspose(double ** J, double ** J_Transpose, int size)
{
	int r, c;
	for (r = 0; r < size; r++)
	{
		for (c = 0; c < 6; c++)
		{
			J_Transpose[r][c] = J[c][r];
		}
	}
}

/*
Wrapper function for the kinematic chain type. See inverse_kinematics for details
*/
void inverse_kinematics_KC(P3D Pdes, KinematicChain * R, double weight)
{
	inverse_kinematics(Pdes, R->DH_Table, R->J, R->J_Transpose, R->HTbase, R->size, weight);
}
/*
calculates inverse kinematics. calculates the jacobian transpose, and uses a weighted velocity vector to multiply to the jacobian transpose to update theta with some calculated velocity. Jacobian is exposed.
INPUTS: Pdes, the desired output point, HTbase, which tells you where the end effector is, size of robot, and the weight to apply to the control system to keep the IK calculation stable
OUTPUTS: updates theta in the DH table, the jacobian and it's transpose
*/
void inverse_kinematics(P3D Pdes, DH_entry * DH_Table, double ** J, double ** J_Transpose, HTmatrix * HTbase, int size, double weight)
{

	Vect3 * d; Vect3 * z;
	d = (Vect3 *)malloc((size + 1) * sizeof(Vect3));
	z = (Vect3 *)malloc(size * sizeof(Vect3));


	int i;
	z[0].u = 0; z[0].v = 0; z[0].w = 1;
	for (i = 1; i < size; i++)
	{
		z[i].u = HTbase[i].H[0][2]; z[i].v = HTbase[i].H[1][2];	z[i].w = HTbase[i].H[2][2];
	}
	d[1].u = HTbase[size].H[0][3];	d[1].v = HTbase[size].H[1][3];	d[1].w = HTbase[size].H[2][3];
	for (i = 2; i <= size; i++)
	{
		d[i].u = HTbase[size].H[0][3] - HTbase[i - 1].H[0][3];
		d[i].v = HTbase[size].H[1][3] - HTbase[i - 1].H[1][3];
		d[i].w = HTbase[size].H[2][3] - HTbase[i - 1].H[2][3];
	}

	for (i = 0; i < size; i++)
	{
		Vect3 res;
		Vect3_CrossProduct(z[i], d[i + 1], &res);
		J[0][i] = res.u;	J[1][i] = res.v;	J[2][i] = res.w;
		J[3][i] = z[i].u;	J[4][i] = z[i].v;	J[5][i] = z[i].w;	
	}

	JacobianTranspose(J, J_Transpose, size);
	double XVS = weight*(Pdes.x - HTbase[size].H[0][3]);
	double YVS = weight*(Pdes.y - HTbase[size].H[1][3]);
	double ZVS = weight*(Pdes.z - HTbase[size].H[2][3]);

	for (i = 1; i <= size; i++)
		DH_Table[i].theta += J_Transpose[i - 1][0] * XVS + J_Transpose[i - 1][1] * YVS + J_Transpose[i - 1][2] * ZVS;

	free(d);
	free(z);

}




/*
res = in1 x in2
INPUTS:	in1, in2
OUTPUTS: res
*/
void Vect3_CrossProduct(Vect3 in1, Vect3 in2, Vect3 * res)
{
	res->u = in1.v * in2.w - in1.w * in2.v;
	res->v = in1.w * in2.u - in1.u * in2.w;
	res->w = in1.u * in2.v - in1.v * in2.u;
}

/*
H1*H2 = Hout
INPUTS: H1, H2
OUTPUS: Hout
*/
void HT_Multiply(HTmatrix H1, HTmatrix H2, HTmatrix * Hout)
{
	int out_r; int out_c; int i;
	for (out_r = 0; out_r < 4; out_r++)
	{
		for (out_c = 0; out_c < 4; out_c++)
		{
			double tmp = 0;
			for (i = 0; i < 4; i++)
			{
				tmp = tmp + H1.H[out_r][i] * H2.H[i][out_c];
			}
			Hout->H[out_r][out_c] = tmp;
		}
	}
}

/*
HTout = HTin^-1
INPUTS: HTin
OUTPUTS: HTout
*/
void HT_Inverse(HTmatrix HTin, HTmatrix * HTout)
{
	int r; int c;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			HTout->H[r][c] = HTin.H[c][r];
		}
	}
	HTout->H[0][3] = -(HTout->H[0][0] * HTin.H[0][3] + HTout->H[0][1] * HTin.H[1][3] + HTout->H[0][2] * HTin.H[2][3]);
	HTout->H[1][3] = -(HTout->H[1][0] * HTin.H[0][3] + HTout->H[1][1] * HTin.H[1][3] + HTout->H[1][2] * HTin.H[2][3]);
	HTout->H[2][3] = -(HTout->H[2][0] * HTin.H[0][3] + HTout->H[2][1] * HTin.H[1][3] + HTout->H[2][2] * HTin.H[2][3]);

	HTout->H[3][0] = 0; HTout->H[3][1] = 0; HTout->H[3][2] = 0; HTout->H[3][3] = 1.0;
}

/*
Point coordinate transformation. for use in IK
INPUTS: Pin, Hout_in
OUPTUTS: Pout
*/
void HT_Point_Multiply(HTmatrix Hout_in, P3D Pin, P3D * Pout)
{
	Pout->x = Hout_in.H[0][0] * Pin.x + Hout_in.H[0][1] * Pin.y + Hout_in.H[0][2] * Pin.z + Hout_in.H[0][3];
	Pout->y = Hout_in.H[1][0] * Pin.x + Hout_in.H[1][1] * Pin.y + Hout_in.H[1][2] * Pin.z + Hout_in.H[1][3];
	Pout->z = Hout_in.H[2][0] * Pin.x + Hout_in.H[2][1] * Pin.y + Hout_in.H[2][2] * Pin.z + Hout_in.H[2][3];
}


/*
Hsource = Hdes
INPUTS: Hsource
OUTPUTS: Hdes
*/
void copy_HT(HTmatrix * Hdes, HTmatrix * Hsource)
{
	int r; int c;
	for (r = 0; r < 4; r++)
	{
		for (c = 0; c < 4; c++)
		{
			Hdes->H[r][c] = Hsource->H[r][c];
		}
	}
}

/*Returns rotation about coordinate. 0 = identity*/
HTmatrix Hx(float angle)
{
	HTmatrix ret;
	ret.H[0][0] = 1;	ret.H[0][1] = 0;			ret.H[0][2] = 0;			ret.H[0][3] = 0;
	ret.H[1][0] = 0;	ret.H[1][1] = cos(angle);	ret.H[1][2] = -sin(angle);	ret.H[1][3] = 0;
	ret.H[2][0] = 0;	ret.H[2][1] = sin(angle);	ret.H[2][2] = cos(angle);	ret.H[2][3] = 0;	
	ret.H[3][0] = 0;	ret.H[3][1] = 0;			ret.H[3][2] = 0;			ret.H[3][3] = 1;
	return ret;
}
/*Returns rotation about coordinate. 0 = identity*/
HTmatrix Hy(float angle)
{
	HTmatrix ret;
	ret.H[0][0] = cos(angle);	ret.H[0][1] = 0;	ret.H[0][2] = sin(angle);	ret.H[0][3] = 0;
	ret.H[1][0] = 0;			ret.H[1][1] = 1;	ret.H[1][2] = 0;			ret.H[1][3] = 0;
	ret.H[2][0] = -sin(angle);	ret.H[2][1] = 0;	ret.H[2][2] = cos(angle);	ret.H[2][3] = 0;
	ret.H[3][0] = 0;			ret.H[3][1] = 0;	ret.H[3][2] = 0;			ret.H[3][3] = 1;
	return ret;
}
/*Returns rotation about coordinate. 0 = identity*/
HTmatrix Hz(float angle)
{
	HTmatrix ret;
	ret.H[0][0] = cos(angle);	ret.H[0][1] = -sin(angle);		ret.H[0][2] = 0;	ret.H[0][3] = 0;
	ret.H[1][0] = sin(angle);	ret.H[1][1] = cos(angle);		ret.H[1][2] = 0;	ret.H[1][3] = 0;
	ret.H[2][0] = 0;			ret.H[2][1] = 0;				ret.H[2][2] = 1;	ret.H[2][3] = 0;
	ret.H[3][0] = 0;			ret.H[3][1] = 0;				ret.H[3][2] = 0;	ret.H[3][3] = 1;
	return ret;
}
/*Grow a vector*/
HTmatrix Hscale(float scale)
{
	HTmatrix ret;
	ret.H[0][0] = scale;	ret.H[0][1] = 0;		ret.H[0][2] = 0;		ret.H[0][3] = 0;
	ret.H[1][0] = 0;		ret.H[1][1] = scale;	ret.H[1][2] = 0;		ret.H[1][3] = 0;
	ret.H[2][0] = 0;		ret.H[2][1] = 0;		ret.H[2][2] = scale;	ret.H[2][3] = 0;
	ret.H[3][0] = 0;		ret.H[3][1] = 0;		ret.H[3][2] = 0;		ret.H[3][3] = 1;
	return ret;
}



void HT_load_point(HTmatrix * H, P3D p)
{
	H->H[0][3] = p.x; H->H[1][3] = p.y; H->H[2][3] = p.z;
}