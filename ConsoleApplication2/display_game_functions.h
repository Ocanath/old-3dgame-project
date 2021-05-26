#ifndef DISPLAY_GAME_FUNCTIONS_H 
#define DISPLAY_GAME_FUNCTIONS_H


#include <iostream>
#include <string>

#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2\opencv.hpp>

#include "GameState.h"
#include "RobotKinematics.h"
#include "MouseUpper.h"

using namespace std;
using namespace cv;


void update_pyramid(Pyramid * P, Mat * frame, Player * player, double r, double g, double b);
void Project_And_Draw_Line_On_Frame(Mat & frame, Line3d_2 Line, HTmatrix HW_C, double lamda, double r, double g, double b);
void project_line_geometry(Player player, Line3d_2 line_to_project, double * u1, double * v1, double * u2, double * v2);
void draw_projected_line2_on_frame(Mat & frame, double u1, double v1, double u2, double v2, double r, double g, double b);
void draw_box_on_frame(Mat & frame, Player player, Box3d box);
void invert_fm(Mat & frame);
void Multiply_STR_HTM(string H1[4][4], string H2[4][4], string Hout[4][4]);
void Write_HT_to_file(string filename, string H[4][4]);
void display_robot_skeleton(Mat * frame, Player * player, Line3d_2 * RobotRender, double r, double g, double b,	HTmatrix * W_idx, int size);
void Display_Mouse(MouseUpper Mouse, Mat * frame, Player * player);

#endif