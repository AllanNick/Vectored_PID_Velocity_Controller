#include "stdint.h"

#define GblIttLength 20 //Global Interger Length
#define PI 3.1415926535
#define SysDt 20;

#define Default_KP 1;
#define Default_KI 0.2;
#define Default_KD 0.01;

typedef struct PID_Vector{
    float parameters[3];
    //[0]->P, [1]->I, [2]->D
    int16_t history_errs[GblIttLength]; 
    uint16_t pointer;
    uint16_t this_container_len;
    //For interger process
    uint16_t Access_tag_ID;
}PIDVexHandleTypedef;

typedef struct Velocies_Target_Vector{
    float velocies_of_whls[4];
    float velocies_of_targs[4];
    float velocies_of_deltas[4];
}VelcVexTypedef;

typedef struct Differential_Control_Interface{
    int16_t Differential_Layer[4];
    int16_t Velocity_Restriction[4];
    int16_t Targ_Velocity_Layer[4];
    int16_t Velocity_Targ;
    int16_t Rotation_Targ;
}DCIHandleTypedef;

//global variables:
PIDVexHandleTypedef PID0;
PIDVexHandleTypedef PID1;
PIDVexHandleTypedef PID2;
PIDVexHandleTypedef PID3;

VelcVexTypedef Wheels_Velocities;

DCIHandleTypedef DCIController;

void PID_CON_Init(PIDVexHandleTypedef *PIDController, uint16_t Access_Tag_ID);
void VLC_CON_Init(void);
void DCI_CON_Init(DCIHandleTypedef *DCIC);
void DCI_Targ_Modify_Overwrite(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Targ_Modify_Offset(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Update_Calc(DCIHandleTypedef *DCIC);
void error_update(PIDVexHandleTypedef *TargPIDController);
float error_calc(PIDVexHandleTypedef *PIDController);
