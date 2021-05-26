#include "display_game_functions.h"

#include <iostream>
#include <fstream>


#include <bitset>




void Project_And_Draw_Line_On_Frame(Mat & frame, Line3d_2 Line, HTmatrix HW_C, double lamda, double r, double g, double b)
{
	
	HTmatrix HC_W;
	HT_Inverse(HW_C, &HC_W);
	P3D wp1; P3D wp2; P3D cam_p1; P3D cam_p2;
	wp1.x = Line.x1; wp1.y = Line.y1; wp1.z = Line.z1;
	wp2.x = Line.x2; wp2.y = Line.y2; wp2.z = Line.z2;
	HT_Point_Multiply(HC_W, wp1, &cam_p1);
	HT_Point_Multiply(HC_W, wp2, &cam_p2);
	
	double z_step = 0.1 * lamda;

	double u1, v1, u2, v2;

	if (cam_p1.z > lamda + z_step)	//works since z is in the camera frame. This is the check for valid coordinates
	{
		u1 = (cam_p1.x * lamda) / cam_p1.z;
		v1 = (cam_p1.y * lamda) / cam_p1.z;
	}
	else
	{
		u1 = (cam_p1.x * lamda) / lamda;
		v1 = (cam_p1.y * lamda) / lamda;
	}
	if (cam_p2.z > lamda + z_step)
	{
		u2 = (cam_p2.x * lamda) / cam_p2.z;
		v2 = (cam_p2.y * lamda) / cam_p2.z;
	}
	else
	{
		u2 = (cam_p2.x * lamda) / lamda;
		v2 = (cam_p2.y * lamda) / lamda;
	}

	double scale = 0.000003;
	int x1 = (int)(-1.0*(u1 / scale) + frame.cols / 2);
	int y1 = (int)(-1.0*(v1 / scale) + frame.rows / 2);
	int x2 = (int)(-1.0*(u2 / scale) + frame.cols / 2);
	int y2 = (int)(-1.0*(v2 / scale) + frame.rows / 2);
	
	if (!((x1 < 0 - frame.cols / 2 || x1 > frame.cols + frame.cols / 2 || y1 < 0 - frame.rows / 2 || y1 > frame.rows + frame.rows / 2)))
		line(frame, Point(x1, y1), Point(x2, y2), Scalar(b, g, r), 1, 8, 0);

}




void draw_projected_line2_on_frame(Mat & frame, double u1, double v1, double u2, double v2, double r, double g, double b)
{
	
	double scale = 0.000003;
	int x1 = -1.0*(u1 / scale) + frame.cols / 2;
	int y1 = -1.0*(v1 / scale) + frame.rows / 2;
	int x2 = -1.0*(u2 / scale) + frame.cols / 2;
	int y2 = -1.0*(v2 / scale) + frame.rows / 2;

	

	if (x1 < 0 - frame.cols / 2 || x1 > frame.cols + frame.cols / 2 || y1 < 0 - frame.rows / 2 || y1 > frame.rows + frame.rows / 2)
	{}
	else
		line(frame, Point(x1, y1), Point(x2, y2), Scalar(b,g,r), 1, 8, 0);

}

/*
NOTES FOR TOMORROWS JESSE:

OK so on your list of things you need to do so far:

implement a function to draw a box, in the way you did in the main

your scheme:
all objects (including the player) are expressed in the coordinates of the world frame (relative to some arbitrary 0, 0 position).
Therefore, you need a coordinate transoMation T, with the player on top and the world on bottom. You can express this as the players
origin in the world, and the rotation matrix is the zyz/xyz whatever composite rotation matrix.

In order to move the player around, rotate the player, and apply positions and rotations to game objects,
you need a scheme for multiplying matrices.

You also need to implement the check where if an objects z coordinate in the CAMERA frame is less than lamda you don't display

*/


void project_line_geometry(Player player, Line3d_2 line_to_project, double * u1, double * v1, double * u2, double * v2)
{

	double x1_c; double y1_c; double z1_c; double x2_c; double y2_c; double z2_c;


	INV_and_Multiply_HT_matrix_by_vector(player.boundary.R, player.boundary.x, player.boundary.y, player.boundary.z, line_to_project.x1, line_to_project.y1, line_to_project.z1, &x1_c, &y1_c, &z1_c);
	INV_and_Multiply_HT_matrix_by_vector(player.boundary.R, player.boundary.x, player.boundary.y, player.boundary.z, line_to_project.x2, line_to_project.y2, line_to_project.z2, &x2_c, &y2_c, &z2_c);


	double z_step = 0.1 * player.lamda;

	if (z1_c > player.lamda + z_step)	//works since z is in the camera frame. This is the check for valid coordinates
	{
		*u1 = (x1_c * player.lamda) / z1_c;
		*v1 = (y1_c * player.lamda) / z1_c;
	}
	else
	{
		*u1 = (x1_c * player.lamda) / player.lamda;
		*v1 = (y1_c * player.lamda) / player.lamda;
	}

	if (z2_c > player.lamda + z_step)
	{
		*u2 = (x2_c * player.lamda) / z2_c;
		*v2 = (y2_c * player.lamda) / z2_c;
	}
	else
	{
		*u2 = (x2_c * player.lamda) / player.lamda;
		*v2 = (y2_c * player.lamda) / player.lamda;
	}

}


/*
Function to display the robot by projecting onto the player camera frame.
INPUTS: Player (by reference), preallocated line list, world to index frame list (W_idx), chain size
OUTPUTS: calculates the mapping of world to 0 frame
*/
void display_robot_skeleton(Mat * frame, Player * player, Line3d_2 * RobotRender, double r, double g, double b,
								HTmatrix * W_idx, int size)
{
	double tu1, tv1, tu2, tv2;
	int i;
	for (i = 1; i <= size; i++)
	{
		//'render' into the list
		RobotRender[i].x1 = W_idx[i - 1].H[0][3];	RobotRender[i].y1 = W_idx[i - 1].H[1][3];	RobotRender[i].z1 = W_idx[i - 1].H[2][3];
		RobotRender[i].x2 = W_idx[i].H[0][3];		RobotRender[i].y2 = W_idx[i].H[1][3];		RobotRender[i].z2 = W_idx[i].H[2][3];
		
		project_line_geometry(*player, RobotRender[i], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(*frame, tu1, tv1, tu2, tv2, r, g, b);
	}
}

void update_pyramid(Pyramid * P, Mat * frame, Player * player, double r, double g, double b)
{
	//recalculate the w_vertex transformation. User can change HW_B and rotate/translate the pyramid
	
	double tu1, tv1, tu2, tv2;

	int i;
	for (i = 0; i < 4; i++)
		HT_Multiply(P->HW_B, P->HB_v[i], &(P->HW_v[i]));
	//			v0
	//
	//			v3	
	//
	//	 v1				v2
	P->edges[0].x1 = P->HW_v[0].H[0][3];	P->edges[0].y1 = P->HW_v[0].H[1][3];	P->edges[0].z1 = P->HW_v[0].H[2][3];
	P->edges[0].x2 = P->HW_v[1].H[0][3];	P->edges[0].y2 = P->HW_v[1].H[1][3];	P->edges[0].z2 = P->HW_v[1].H[2][3];
	
	P->edges[1].x1 = P->HW_v[1].H[0][3];	P->edges[1].y1 = P->HW_v[1].H[1][3];	P->edges[1].z1 = P->HW_v[1].H[2][3];
	P->edges[1].x2 = P->HW_v[2].H[0][3];	P->edges[1].y2 = P->HW_v[2].H[1][3];	P->edges[1].z2 = P->HW_v[2].H[2][3];

	P->edges[2].x1 = P->HW_v[2].H[0][3];	P->edges[2].y1 = P->HW_v[2].H[1][3];	P->edges[2].z1 = P->HW_v[2].H[2][3];
	P->edges[2].x2 = P->HW_v[3].H[0][3];	P->edges[2].y2 = P->HW_v[3].H[1][3];	P->edges[2].z2 = P->HW_v[3].H[2][3];

	P->edges[3].x1 = P->HW_v[3].H[0][3];	P->edges[3].y1 = P->HW_v[3].H[1][3];	P->edges[3].z1 = P->HW_v[3].H[2][3];
	P->edges[3].x2 = P->HW_v[0].H[0][3];	P->edges[3].y2 = P->HW_v[0].H[1][3];	P->edges[3].z2 = P->HW_v[0].H[2][3];

	P->edges[4].x1 = P->HW_v[2].H[0][3];	P->edges[4].y1 = P->HW_v[2].H[1][3];	P->edges[4].z1 = P->HW_v[2].H[2][3];
	P->edges[4].x2 = P->HW_v[0].H[0][3];	P->edges[4].y2 = P->HW_v[0].H[1][3];	P->edges[4].z2 = P->HW_v[0].H[2][3];

	P->edges[5].x1 = P->HW_v[1].H[0][3];	P->edges[5].y1 = P->HW_v[1].H[1][3];	P->edges[5].z1 = P->HW_v[1].H[2][3];
	P->edges[5].x2 = P->HW_v[3].H[0][3];	P->edges[5].y2 = P->HW_v[3].H[1][3];	P->edges[5].z2 = P->HW_v[3].H[2][3];
	for (i = 0; i < 6; i++)
	{
		project_line_geometry(*player, P->edges[i], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(*frame, tu1, tv1, tu2, tv2, 255, 255, 255);
	}
}




void Display_Mouse(MouseUpper Mouse, Mat * frame, Player * player)
{
	HTmatrix RMouseW_idx[MOUSE_ARM_ARR_SIZE];
	RMouseW_idx[0].H[0][3] = Mouse.RMW_0.H[0][3]; RMouseW_idx[0].H[1][3] = Mouse.RMW_0.H[1][3]; RMouseW_idx[0].H[2][3] = Mouse.RMW_0.H[2][3];
	int i;
	for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		HT_Multiply(Mouse.RMW_0, Mouse.RMouseBase[i], &(RMouseW_idx[i]));
	Line3d_2 RMouseSkeleton[MOUSE_ARM_ARR_SIZE];
	display_robot_skeleton(frame, player, RMouseSkeleton, 0, 255, 0, RMouseW_idx, MOUSE_ARM_SIZE);

	HTmatrix LMouseW_idx[MOUSE_ARM_ARR_SIZE];
	LMouseW_idx[0].H[0][3] = Mouse.LMW_0.H[0][3]; LMouseW_idx[0].H[1][3] = Mouse.LMW_0.H[1][3]; LMouseW_idx[0].H[2][3] = Mouse.LMW_0.H[2][3];
	for (i = 1; i <= MOUSE_ARM_SIZE; i++)
		HT_Multiply(Mouse.LMW_0, Mouse.LMouseBase[i], &(LMouseW_idx[i]));
	Line3d_2 LMouseSkeleton[MOUSE_ARM_ARR_SIZE];
	display_robot_skeleton(frame, player, LMouseSkeleton, 255, 0, 255, LMouseW_idx, MOUSE_ARM_SIZE);
}





void draw_box_on_frame(Mat & frame, Player player, Box3d box)
{
	
	Line3d_2 box_edges[12];
	//width -> x
	//height -> y
	//depth -> z
	
	box_edges[0].x1 = box.x - box.width / 2;
	box_edges[0].x2 = box.x - box.width / 2;
	box_edges[0].y1 = box.y - box.height / 2;
	box_edges[0].y2 = box.y - box.height / 2;
	box_edges[0].z1 = box.z - box.depth / 2;
	box_edges[0].z2 = box.z + box.depth / 2;

	box_edges[1].x1 = box.x + box.width / 2;
	box_edges[1].x2 = box.x + box.width / 2;
	box_edges[1].y1 = box.y - box.height / 2;
	box_edges[1].y2 = box.y - box.height / 2;
	box_edges[1].z1 = box.z - box.depth / 2;
	box_edges[1].z2 = box.z + box.depth / 2;
	
	box_edges[2].x1 = box.x - box.width / 2;
	box_edges[2].x2 = box.x - box.width / 2;
	box_edges[2].y1 = box.y + box.height / 2;
	box_edges[2].y2 = box.y + box.height / 2;
	box_edges[2].z1 = box.z - box.depth / 2;
	box_edges[2].z2 = box.z + box.depth / 2;

	box_edges[3].x1 = box.x + box.width / 2;
	box_edges[3].x2 = box.x + box.width / 2;
	box_edges[3].y1 = box.y + box.height / 2;
	box_edges[3].y2 = box.y + box.height / 2;
	box_edges[3].z1 = box.z - box.depth / 2;
	box_edges[3].z2 = box.z + box.depth / 2;

	box_edges[4].x1 = box.x - box.width / 2;
	box_edges[4].x2 = box.x - box.width / 2;
	box_edges[4].y1 = box.y - box.height / 2;
	box_edges[4].y2 = box.y + box.height / 2;
	box_edges[4].z1 = box.z - box.depth / 2;
	box_edges[4].z2 = box.z - box.depth / 2;

	box_edges[5].x1 = box.x + box.width / 2;
	box_edges[5].x2 = box.x + box.width / 2;
	box_edges[5].y1 = box.y - box.height / 2;
	box_edges[5].y2 = box.y + box.height / 2;
	box_edges[5].z1 = box.z - box.depth / 2;
	box_edges[5].z2 = box.z - box.depth / 2;

	box_edges[6].x1 = box.x - box.width / 2;
	box_edges[6].x2 = box.x - box.width / 2;
	box_edges[6].y1 = box.y - box.height / 2;
	box_edges[6].y2 = box.y + box.height / 2;
	box_edges[6].z1 = box.z + box.depth / 2;
	box_edges[6].z2 = box.z + box.depth / 2;

	box_edges[7].x1 = box.x + box.width / 2;
	box_edges[7].x2 = box.x + box.width / 2;
	box_edges[7].y1 = box.y - box.height / 2;
	box_edges[7].y2 = box.y + box.height / 2;
	box_edges[7].z1 = box.z + box.depth / 2;
	box_edges[7].z2 = box.z + box.depth / 2;

	box_edges[8].x1 = box.x - box.width / 2;
	box_edges[8].x2 = box.x + box.width / 2;
	box_edges[8].y1 = box.y - box.height / 2;
	box_edges[8].y2 = box.y - box.height / 2;
	box_edges[8].z1 = box.z - box.depth / 2;
	box_edges[8].z2 = box.z - box.depth / 2;

	box_edges[9].x1 = box.x - box.width / 2;
	box_edges[9].x2 = box.x + box.width / 2;
	box_edges[9].y1 = box.y + box.height / 2;
	box_edges[9].y2 = box.y + box.height / 2;
	box_edges[9].z1 = box.z - box.depth / 2;
	box_edges[9].z2 = box.z - box.depth / 2;

	box_edges[10].x1 = box.x - box.width / 2;
	box_edges[10].x2 = box.x + box.width / 2;
	box_edges[10].y1 = box.y - box.height / 2;
	box_edges[10].y2 = box.y - box.height / 2;
	box_edges[10].z1 = box.z + box.depth / 2;
	box_edges[10].z2 = box.z + box.depth / 2;

	box_edges[11].x1 = box.x - box.width / 2;
	box_edges[11].x2 = box.x + box.width / 2;
	box_edges[11].y1 = box.y + box.height / 2;
	box_edges[11].y2 = box.y + box.height / 2;
	box_edges[11].z1 = box.z + box.depth / 2;
	box_edges[11].z2 = box.z + box.depth / 2;

	

	for (int i = 0; i < 12; i++)
	{
		double tu1, tv1, tu2, tv2;
		project_line_geometry(player, box_edges[i], &tu1, &tv1, &tu2, &tv2);
		draw_projected_line2_on_frame(frame, tu1, tv1, tu2, tv2, 255, 255, 255);
	}


}





void invert_fm(Mat & frame)
{
	for (int y = 0; y < frame.rows; y++)
	{
		for (int x = 0; x < frame.cols; x++)
		{
			int b = frame.data[frame.step[0] * (y)+frame.step[1] * (x)+0];
			int g = frame.data[frame.step[0] * (y)+frame.step[1] * (x)+1];
			int r = frame.data[frame.step[0] * (y)+frame.step[1] * (x)+2];
			frame.data[frame.step[0] * (y)+frame.step[1] * (x)+0] = 255 - b;
			frame.data[frame.step[0] * (y)+frame.step[1] * (x)+1] = 255 - g;
			frame.data[frame.step[0] * (y)+frame.step[1] * (x)+2] = 255 - r;
		}
	}
}










void Multiply_STR_HTM(string H1[4][4], string H2[4][4], string Hout[4][4])
{
	for (int out_r = 0; out_r < 3; out_r++)
	{
		for (int out_c = 0; out_c < 4; out_c++)
		{
			string tmp = "";
		
			for (int i = 0; i < 4; i++)
			{
				if (H1[out_r][i] != "0" && H2[i][out_c] != "0")
				{
					if (tmp != "")
						tmp.append(" + ");
					tmp.append(H1[out_r][i]);
					tmp.append("*");
					tmp.append(H2[i][out_c]);

				}
			}

			if (tmp == "")
				tmp = "0";
			Hout[out_r][out_c] = tmp;
		}
	}
	Hout[3][0] = "0"; Hout[3][1] = "0"; Hout[3][2] = "0"; Hout[3][3] = "1";

}


void Write_HT_to_file(string filename, string H[4][4])
{
	ofstream myfile;
	myfile.open(filename);
	myfile.clear();

	for (int r = 0; r < 4; r++)
	{
		for (int c = 0; c < 4; c++)
		{
			myfile << H[r][c] << "					";
		}
		myfile << endl;
	}

	myfile.close();

}