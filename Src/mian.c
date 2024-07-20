#include "stdint.h"

#define GblIttLength 20 //Global Interger Length
#define PI 3.1415926535
#define SysDt 20;

#define Default_KP 1;
#define Default_KI 0.2;
#define Default_KD 0.01;

typedef struct PID_Vector{
    int16_t parameters[3];
    //[0]->P, [1]->I, [2]->D
    int16_t history_errs[GblIttLength]; 
    uint16_t pointer;
    uint16_t this_container_len;
    //For interger process
    uint16_t Access_tag_ID;
}PIDVexHandleTypedef;

typedef struct Velocies_Target_Vector{
    int16_t velocies_of_whls[4];
    int16_t velocies_of_targs[4];
    int16_t velocies_of_deltas[4];
}VelcVexTypedef;

//global variables:
PIDVexHandleTypedef PID0;
PIDVexHandleTypedef PID1;
PIDVexHandleTypedef PID2;
PIDVexHandleTypedef PID3;

VelcVexTypedef Wheels_Velocities;

void PID_CON_Init(PIDVexHandleTypedef *PIDController, uint16_t Access_Tag_ID);

void PID_CON_Init(PIDVexHandleTypedef *PIDController, uint16_t Access_Tag_ID){
    PIDController->parameters[0] = Default_KP;
    PIDController->parameters[1] = Default_KI;
    PIDController->parameters[2] = Default_KD;
    for(int i =0; i < GblIttLength; i++){
        PIDController->history_errs[i] = 0;
    }
    PIDController->this_container_len = GblIttLength;
    PIDController->Access_tag_ID = Access_Tag_ID;
}

void VLC_CON_Init(void);

void VLC_CON_Init(void){
    for(int i =0; i <4; i++){
        Wheels_Velocities.velocies_of_deltas[i] = 0;
        Wheels_Velocities.velocies_of_targs[i] = 0;
        Wheels_Velocities.velocies_of_whls[i] = 0;
    }
}
/*
void Get_velocties(void);
void Delta_velocties(void);
*/
void error_update(PIDVexHandleTypedef *TargPIDController);

void error_update(PIDVexHandleTypedef *TargPIDController){
    uint16_t Data_Index = TargPIDController->Access_tag_ID;
    if(TargPIDController->pointer == GblIttLength){
        TargPIDController->pointer = 0;
    }else{
        TargPIDController->pointer ++;
        ;
    }
    //calculate velocities then transmit to PID Controller history
    Wheels_Velocities.velocies_of_deltas[Data_Index] = 
        Wheels_Velocities.velocies_of_targs - Wheels_Velocities.velocies_of_whls;
    //to access PID's velocity: Wheels_Velocities.velocies_of_whls[Data_Index];
    TargPIDController->history_errs[TargPIDController->pointer];
}

float error_calc(PIDVexHandleTypedef *PIDController);

float error_calc(PIDVexHandleTypedef *PIDController){
    float result = 0;
    int16_t itteration_val = 0;
    for(int i =0; i < PIDController->this_container_len; i++){
        itteration_val += PIDController->history_errs[i];
    }
    result = PIDController->parameters[0] * PIDController->history_errs[PIDController->pointer]
            +PIDController->parameters[1] * itteration_val;
            +PIDController->parameters[2] * SysDt;

    return result;
}
//global functions declearations:

int main(){

    return 0;
}
 