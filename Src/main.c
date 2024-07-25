#include "stdint.h"

#define ITTLEN 20

#define DefaultKP 1
#define DefaultKI 0.2
#define DefaultKD 0.1

#define DefatltRefDt 2

typedef struct PIDCON{
    /*Parameters*/
    float Parameters[3];
    /*Data*/
    float HistoryErrs[ITTLEN];
    /*History Data Pointer*/
    uint16_t HDPTR;
    
    float *Error;
    int16_t VelocityWheel;
    int16_t VelocityTarg;
    float Increment;

    float RefDt;

    int16_t PIDC_INIT_STAT;
    /*
    4 status of DIRIN:
    0x00, 0x01, 0x02, 0x03
    00  , 01  , 10  , 11
    free fwrd  back  break
    */
    uint8_t DIRIN;

}PIDVexHandleTypedef;


void PID_CON_INIT(PIDVexHandleTypedef *PIDC);
void PID_CON_INIT(PIDVexHandleTypedef *PIDC){
    PIDC->Parameters[0] = DefaultKP;
    PIDC->Parameters[1] = DefaultKI;
    PIDC->Parameters[2] = DefaultKD;
    
    for(int16_t i =0; i<20; i++){
        PIDC->HistoryErrs[i] = 0;
    }
    PIDC->HDPTR =0;
    PIDC->Error = &PIDC->HistoryErrs[PIDC->HDPTR];
    PIDC->VelocityTarg = 0;
    PIDC->VelocityWheel = 0;
    PIDC->Increment =0;
    PIDC->PIDC_INIT_STAT = 1;

    PIDC->RefDt = DefatltRefDt;
    PIDC->DIRIN = 0x00;
}

void PID_CON_ModifyTarg(PIDVexHandleTypedef *PIDC, uint16_t TargVelocity);
void PID_CON_ModifyTarg(PIDVexHandleTypedef *PIDC, uint16_t TargVelocity){
    PIDC->VelocityTarg = TargVelocity;
}


void PID_CON_Update(PIDVexHandleTypedef *PIDC, uint16_t Velocity);
void PID_CON_Update(PIDVexHandleTypedef *PIDC, uint16_t Velocity){
    if(PIDC->HDPTR >= ITTLEN && PIDC->PIDC_INIT_STAT == 1){
        PIDC->HDPTR = 0;
    }else{
        PIDC->HDPTR ++;
    }
    PIDC->Error = &PIDC->HistoryErrs[PIDC->HDPTR];
    *(PIDC->Error) = PIDC->VelocityTarg - Velocity;

    float Integral = 0;
    for(uint16_t i =0; i< ITTLEN; i++){
        Integral = Integral + PIDC->HistoryErrs[i]; 
    }

    PIDC->Increment = PIDC->Parameters[0] * *(PIDC->Error)
                    + PIDC->Parameters[1] * Integral
                    + PIDC->Parameters[2] * (*(PIDC->Error)/PIDC->RefDt);

    /*Reserved Function*/
    if(PIDC->VelocityTarg >> 16 | 1 == 1){
        PIDC->DIRIN = 0x01;
    }else{
        PIDC->DIRIN = 0X02;
    }
}

PIDVexHandleTypedef PIDC0;
PIDVexHandleTypedef PIDC1;

PIDVexHandleTypedef *PIDControllerHandlers[2] = {&PIDC0, &PIDC1};

/*Differential Control Interface*/
typedef struct DCICON{
    float TargetVelocoty;
    float TargetAngle;

    float SolvedAbsolutVector[4];
    float SolvedDifferentialVector[4];
}DCIVexHandleTypedef;

int main(){
    PID_CON_INIT(PIDControllerHandlers[0]);
    PID_CON_INIT(PIDControllerHandlers[1]);

    PID_CON_ModifyTarg(PIDControllerHandlers[0], 20);
    PID_CON_ModifyTarg(PIDControllerHandlers[1], 20);

    PID_CON_Update(PIDControllerHandlers[0], 0);
    PID_CON_Update(PIDControllerHandlers[1], 40);
    return 0;
}