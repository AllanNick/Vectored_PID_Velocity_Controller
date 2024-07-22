#include "stdint.h"
#include "stdlib.h"

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

void DCI_CON_Init(DCIHandleTypedef *DCIC);

void DCI_CON_Init(DCIHandleTypedef *DCIC){
    for(int i = 0; i<4 ;i++){
        DCIC->Differential_Layer[i] = 0;
        DCIC->Targ_Velocity_Layer[i] =0;
        DCIC->Velocity_Restriction[i] = 100;
    }
    DCIC->Rotation_Targ = 0;
    DCIC->Velocity_Targ = 0;
}

void DCIC_To_VLC_Update(DCIHandleTypedef *DCIC, VelcVexTypedef *VLCC){
    for(int i =0; i<4; i++){
        VLCC->velocies_of_targs[i] = DCIC->Targ_Velocity_Layer[i];
    }
}

void DCI_Targ_Modify_Overwrite(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Targ_Modify_Offset(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Update_Calc(DCIHandleTypedef *DCIC);

void DCI_Targ_Modify_Overwrite(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = Targ_V;
    DCIC->Rotation_Targ = Targ_R;
}

void DCI_Targ_Modify_Offset(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = DCIC->Velocity_Targ + Targ_V;
    DCIC->Rotation_Targ = DCIC->Rotation_Targ + Targ_R;
}

void DCI_Update_Calc(DCIHandleTypedef *DCIC){
    //DIC LOCIC PART
    //PREVILIAGE OF 'R' HIGHER THAN 'V'
    if(abs(DCIC->Velocity_Targ)>100){
        if(DCIC->Velocity_Targ>100){
            DCIC->Velocity_Targ = 100;
        }else{
            DCIC->Velocity_Targ = -100;
        }
    }
    for(int i =0;i <4; i++){
        DCIC->Targ_Velocity_Layer[i] = DCIC->Velocity_Targ;
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
    }
    //calculate velocities then transmit to PID Controller history
    float delta = Wheels_Velocities.velocies_of_targs[Data_Index] - Wheels_Velocities.velocies_of_whls[Data_Index]; 
    Wheels_Velocities.velocies_of_deltas[Data_Index] = delta;
        
    //to access PID's velocity: Wheels_Velocities.velocies_of_whls[Data_Index];
    TargPIDController->history_errs[TargPIDController->pointer] = delta;
}

float error_calc(PIDVexHandleTypedef *PIDController);

float error_calc(PIDVexHandleTypedef *PIDController){
    float result = 0;
    float itteration_val = 0;
    for(int i =0; i < PIDController->this_container_len; i++){
        itteration_val += PIDController->history_errs[i];
    }
    float this_error = PIDController->history_errs[PIDController->pointer];
    result = PIDController->parameters[0] * this_error
            +PIDController->parameters[1] * itteration_val
            +PIDController->parameters[2] * this_error/SysDt;

    return result;
}
//global functions declearations:

//PIDVexHandleTypedef *PIDS = {&PID0, &PID1, &PID2, &PID3};
PIDVexHandleTypedef *PID_Devices_Handles_Container[4] = {&PID0, &PID1, &PID2, &PID3};

int main(){
    for(int i =0; i <4; i++){
        PID_CON_Init(PID_Devices_Handles_Container[i],i);
    }
    VLC_CON_Init();
    DCI_CON_Init(&DCIController);

    DCI_Targ_Modify_Offset(&DCIController, 20, 0);
    DCI_Update_Calc(&DCIController);
    DCIC_To_VLC_Update(&DCIController, &Wheels_Velocities);

    for(int j =0; j< 10; j++){
        for(int i =0; i <4; i++){
            error_calc(PID_Devices_Handles_Container[i]);
            error_update(PID_Devices_Handles_Container[i]);
        }
    }
    return 0;
}
 