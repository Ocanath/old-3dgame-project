#include "Configurations.h"
#include "GameState.h"

#define STEPSIZE 20
#define OUTSIZE 40
#define RAIZE_LEVEL -170
#define NUM_CONFIGS 4

Point_3d WALK_kp[4][7];

//establish keypoints. label them with configuration numbers
void init_walk(double T0_B[7][4][4])
{

	int cf_id = 0;
	WALK_kp[cf_id][1].x = 178;				WALK_kp[cf_id][1].y = 0;					WALK_kp[cf_id][1].z = RAIZE_LEVEL;
	WALK_kp[cf_id][2].x = 89 + OUTSIZE;		WALK_kp[cf_id][2].y = 154 + STEPSIZE;		WALK_kp[cf_id][2].z = -200;
	WALK_kp[cf_id][3].x = -89;				WALK_kp[cf_id][3].y = 154;					WALK_kp[cf_id][3].z = RAIZE_LEVEL;
	WALK_kp[cf_id][4].x = -178 - OUTSIZE;	WALK_kp[cf_id][4].y = 0 + STEPSIZE;			WALK_kp[cf_id][4].z = -200;
	WALK_kp[cf_id][5].x = -89;				WALK_kp[cf_id][5].y = -154;					WALK_kp[cf_id][5].z = RAIZE_LEVEL;
	WALK_kp[cf_id][6].x = 89 + OUTSIZE;		WALK_kp[cf_id][6].y = -154 + STEPSIZE;		WALK_kp[cf_id][6].z = -200;

	cf_id = 1;
	WALK_kp[cf_id][1].x = 178 + OUTSIZE;	WALK_kp[cf_id][1].y = 0 + STEPSIZE;			WALK_kp[cf_id][1].z = RAIZE_LEVEL;
	WALK_kp[cf_id][2].x = 89 + OUTSIZE;		WALK_kp[cf_id][2].y = 154 - STEPSIZE;		WALK_kp[cf_id][2].z = -200;
	WALK_kp[cf_id][3].x = -89 - OUTSIZE;	WALK_kp[cf_id][3].y = 154 + STEPSIZE;		WALK_kp[cf_id][3].z = RAIZE_LEVEL;
	WALK_kp[cf_id][4].x = -178 - OUTSIZE;	WALK_kp[cf_id][4].y = 0 - STEPSIZE;			WALK_kp[cf_id][4].z = -200;
	WALK_kp[cf_id][5].x = -89 - OUTSIZE;	WALK_kp[cf_id][5].y = -154 + STEPSIZE;		WALK_kp[cf_id][5].z = RAIZE_LEVEL;
	WALK_kp[cf_id][6].x = 89 + OUTSIZE;		WALK_kp[cf_id][6].y = -154 - STEPSIZE;		WALK_kp[cf_id][6].z = -200;

	cf_id = 2;
	WALK_kp[cf_id][1].x = 178 + OUTSIZE;	WALK_kp[cf_id][1].y = 0 + STEPSIZE;			WALK_kp[cf_id][1].z = -200;
	WALK_kp[cf_id][2].x = 89;				WALK_kp[cf_id][2].y = 154;					WALK_kp[cf_id][2].z = RAIZE_LEVEL;
	WALK_kp[cf_id][3].x = -89 - OUTSIZE;	WALK_kp[cf_id][3].y = 154 + STEPSIZE;		WALK_kp[cf_id][3].z = -200;
	WALK_kp[cf_id][4].x = -178;				WALK_kp[cf_id][4].y = 0;					WALK_kp[cf_id][4].z = RAIZE_LEVEL;
	WALK_kp[cf_id][5].x = -89 - OUTSIZE;	WALK_kp[cf_id][5].y = -154 + STEPSIZE;		WALK_kp[cf_id][5].z = -200;
	WALK_kp[cf_id][6].x = 89;				WALK_kp[cf_id][6].y = -154;					WALK_kp[cf_id][6].z = RAIZE_LEVEL;

	cf_id = 3;
	WALK_kp[cf_id][1].x = 178 + OUTSIZE;	WALK_kp[cf_id][1].y = 0 - STEPSIZE;			WALK_kp[cf_id][1].z = -200;
	WALK_kp[cf_id][2].x = 89 + OUTSIZE;		WALK_kp[cf_id][2].y = 154 + STEPSIZE;		WALK_kp[cf_id][2].z = RAIZE_LEVEL;
	WALK_kp[cf_id][3].x = -89 - OUTSIZE;	WALK_kp[cf_id][3].y = 154 - STEPSIZE;		WALK_kp[cf_id][3].z = -200;
	WALK_kp[cf_id][4].x = -178 - OUTSIZE;	WALK_kp[cf_id][4].y = 0 + STEPSIZE;			WALK_kp[cf_id][4].z = RAIZE_LEVEL;
	WALK_kp[cf_id][5].x = -89 - OUTSIZE;	WALK_kp[cf_id][5].y = -154 - STEPSIZE;		WALK_kp[cf_id][5].z = -200;
	WALK_kp[cf_id][6].x = 89 + OUTSIZE;		WALK_kp[cf_id][6].y = -154 + STEPSIZE;		WALK_kp[cf_id][6].z = RAIZE_LEVEL;

	int i, legnum;
	for (i = 0; i < NUM_CONFIGS; i++)
	{
		for (legnum = 1; legnum <= 6; legnum++)
			HT_Matrix_Vect_Multiply(T0_B[legnum], WALK_kp[i][legnum].x, WALK_kp[i][legnum].y, WALK_kp[i][legnum].z, &WALK_kp[i][legnum].x, &WALK_kp[i][legnum].y, &WALK_kp[i][legnum].z);
	}

}

//for now, just move to the keypoits.
//next step: use the keypoints to establish lines, and establish a function of 
// the time that iterates through the line. this will be k*(KP1-KP2)+KP2, where k is
//a function of time, and kp1 and kp2 are two keypoints forming a line for a given leg
void Walk(double TW_B[4][4], unsigned int time_step, double * xDes, double * yDes, double * zDes)
{

	if (time_step > 100)
	{
		int i;
		int config_num;
		config_num = (time_step / 15) % NUM_CONFIGS;
		
		for (i = 1; i <= 6; i++)
		{
			xDes[i] = WALK_kp[config_num][i].x;		
			yDes[i] = WALK_kp[config_num][i].y;	
			zDes[i] = WALK_kp[config_num][i].z;
		}


//		if (((time_step - 1) / 100 % NUM_CONFIGS) != ((time_step) / 100 % NUM_CONFIGS))
//			TW_B[1][3] += 30
	}
}


