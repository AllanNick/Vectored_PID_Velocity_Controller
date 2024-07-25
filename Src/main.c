#include "stdint.h"
#include "stdlib.h"

#define ItteriateorLength 20

#define SysDt 20;

/*default parameters of PID controller*/
#define Default_KP 1;
#define Default_KI 0.2;
#define Default_KD 0.01;
/*Standard data structure of a PID controller*/
typedef struct PID_Vector
{
    /*[0]->P, [1]->I, [2]->D*/
    float PID_Paremeters[3];
    /*This list contains historical errors
    with its own Data Pointer in this structure*/
    uint16_t History_Err[ItteriateorLength];
    /*active(latest) data can be access with:
    [PIDVexHandleTypedef.Data_Pointer]*/
    uint16_t Data_Pointer;
    uint16_t This_Content_Len;
    /*Access ID can help external data linking*/
    uint16_t Access_ID;

}PIDVexHandleTypedef;

typedef struct Velocity_Vector
{
    float VWhls[4];
    float VTargs[4];
    float VDeltas[4];
    float ErrorsIncrements[4];
    uint16_t Latest_Flag;
}VelocityVexDataTypedef;

typedef struct Differential_Control_Interface{
    int16_t Differential_Layer[4];
    int16_t Velocity_Restrictionz_Layer[4];
    int16_t Targ_Velocity_Layer[4];
    int16_t Velocity_Targ;
    int16_t Rotation_Targ;
}DCInterfaceHandleTypedef;

//global variables:
PIDVexHandleTypedef PIDC0;
PIDVexHandleTypedef PIDC1;
PIDVexHandleTypedef PIDC2;
PIDVexHandleTypedef PIDC3;

VelocityVexDataTypedef WHLInfo;

DCInterfaceHandleTypedef DCIController;

PIDVexHandleTypedef *PIDDeviceHandles[4] = {&PIDC0, &PIDC1, &PIDC2, &PIDC3};

/*Single PID Vector update method*/
void PID_CON_Init(PIDVexHandleTypedef *thisPIDController, uint16_t thisID);
void PID_CON_Init(PIDVexHandleTypedef *thisPIDController, uint16_t thisID){
    thisPIDController->PID_Paremeters[0] = Default_KP;
    thisPIDController->PID_Paremeters[1] = Default_KI;
    thisPIDController->PID_Paremeters[2] = Default_KD;

    for(int i =0; i < ItteriateorLength; i++){
        thisPIDController->History_Err[i] = 0;
    }
    thisPIDController->This_Content_Len = ItteriateorLength;
    thisPIDController->Access_ID = thisID;
}

/*Extend method for global PIDController update*/
void PID_ALL_Init(void);
void PID_ALL_Init(void){
    for(int i =0; i< 4; i++){
        PID_CON_Init(PIDDeviceHandles[i],i);
    }
}

/*PID Data update [interface with Velocity Vector]*/
void PID_DAT_Update(PIDVexHandleTypedef *thisPIDController);
void PID_DAT_Update(PIDVexHandleTypedef *thisPIDController){
    /*shift and correct pointer*/
    if(thisPIDController->Data_Pointer == 19){
        thisPIDController->Data_Pointer == 0;
    }else{
        thisPIDController->Data_Pointer ++;
    }
    if(WHLInfo.Latest_Flag == 1){
        thisPIDController->History_Err[thisPIDController->Data_Pointer] 
        = WHLInfo.VDeltas[thisPIDController->Access_ID];
        WHLInfo.Latest_Flag == 0;
    }
}

/*Extened PID Data update [interface with Velocity Vector]*/
void PID_ALL_Update(void);
void PID_ALL_Update(void){
    for(int i =0; i <4; i++){
        PID_DAT_Update(PIDDeviceHandles[i]);
    }
}
/*PID calculate optput*/
float PID_DAT_Calc(PIDVexHandleTypedef *thisPIDController);
float PID_DAT_Calc(PIDVexHandleTypedef *thisPIDController){
    float result = 0;
    float itteration_val = 0;
    float this_error;
    for(int i =0; i < thisPIDController->This_Content_Len; i++){
        itteration_val += thisPIDController->History_Err[i];
    }
    this_error = thisPIDController->History_Err[thisPIDController->Data_Pointer];
    result = thisPIDController->PID_Paremeters[0] * this_error
            +thisPIDController->PID_Paremeters[1] * itteration_val
            +thisPIDController->PID_Paremeters[2] * this_error/ SysDt;
    
    return result;
}
/*Extened for GLOBAL PID calculate optput and transmit*/
void PID_ALL_Calc(void);
void PID_ALL_Calc(void){
    for(int i =0; i <4; i++){
        WHLInfo.ErrorsIncrements[i] = PID_DAT_Calc(PIDDeviceHandles[i]);
    }
}

void DCI_CON_Init(DCInterfaceHandleTypedef *DCIC);
void DCI_CON_Init(DCInterfaceHandleTypedef *DCIC){
    for(int i =0; i< 4;i ++){
        DCIC->Differential_Layer[i] = 0;
        DCIC->Targ_Velocity_Layer[i] = 0;
        DCIC->Velocity_Restrictionz_Layer[i] = 100;
    }
    DCIC->Rotation_Targ = 0;
    DCIC->Velocity_Targ = 0;
}

/*do an uptate for velocity layer 
after giving command to Interface*/
void DCI_TO_VLC_Update(DCInterfaceHandleTypedef *DCIC, VelocityVexDataTypedef *VVDC);
void DCI_TO_VLC_Update(DCInterfaceHandleTypedef *DCIC, VelocityVexDataTypedef *VVDC){
    for(int i =0; i<4; i++){
        VVDC->VTargs[i] = DCIC->Targ_Velocity_Layer[i];
    }
}

/*DCIController Commands /Interface commands*/

void DCI_Targ_Modify_Overwrite(DCInterfaceHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Targ_Modify_Offset(DCInterfaceHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Update_Calc(DCInterfaceHandleTypedef *DCIC);

void DCI_Targ_Modify_Overwrite(DCInterfaceHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = Targ_V;
    DCIC->Rotation_Targ = Targ_R;
}

void DCI_Targ_Modify_Offset(DCInterfaceHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = DCIC->Velocity_Targ + Targ_V;
    DCIC->Rotation_Targ = DCIC->Rotation_Targ + Targ_R;
}

/*Compile the scalar commands into machine vector*/
void DCI_Update_Calc(DCInterfaceHandleTypedef *DCIC){
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

void VLC_CON_Init(VelocityVexDataTypedef *VVDC){
    for(int i =0; i < 4; i++){
        VVDC->ErrorsIncrements[i] = 0;
        VVDC->VWhls[i] = 0;
        VVDC->VTargs[i] = 0;
        VVDC->VDeltas[i] = 0;
    }
    VVDC->Latest_Flag = 1;
}

void VLC_CON_FetchData(VelocityVexDataTypedef *VVDC){
    /*Get datas from sensor part*/

    /*Set update flag*/
    VVDC->Latest_Flag = 1;
}

void VLC_CON_DeltaUpdate(VelocityVexDataTypedef *VVDC){
    for(int i =0; i <4; i++){
        VVDC->VDeltas[i] = VVDC->VTargs[i] - VVDC->VWhls[i]; 
    }
}

int main(){
    /*init all Data structures*/
    PID_ALL_Init();
    VLC_CON_Init(&WHLInfo);
    DCI_CON_Init(&DCIController);

    DCI_Targ_Modify_Offset(&DCIController, 20, 0);
    DCI_Update_Calc(&DCIController);
    DCI_TO_VLC_Update(&DCIController, &WHLInfo);
    for(int i =0; i <30; i++){
        VLC_CON_FetchData(&WHLInfo);
        VLC_CON_DeltaUpdate(&WHLInfo);
        PID_ALL_Update();
        PID_ALL_Calc();
    }
    return 0;
}