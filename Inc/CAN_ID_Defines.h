#ifndef CAN_ID_DEFINES_H
#define	CAN_ID_DEFINES_H


/************************** EFI ***************************/
#define EFI_HALL_ID                     ((uint16_t)(772))
#define EFI_WATER_TEMPERATURE_ID        ((uint16_t)(780))
#define EFI_OIL_T_ENGINE_BAT_ID         ((uint16_t)(781))
#define EFI_GEAR_RPM_TPS_APPS_ID        ((uint16_t)(773))
#define EFI_TRACTION_CONTROL_ID         ((uint16_t)(774))
#define EFI_FUEL_FAN_H2O_LAUNCH_ID      ((uint16_t)(782))
#define EFI_PRESSURES_LAMBDA_SMOT_ID    ((uint16_t)(775))
#define EFI_DIAG_IGN_EXHAUST_ID         ((uint16_t)(783))

/************************** GCU ***************************/
#define GCU_TRACTION_CONTROL_EFI_ID     ((uint16_t)(1280))  // MANDATO SOLO DA GCU AD EFI
#define GCU_LAUNCH_CONTROL_EFI_ID       ((uint16_t)(1281))  // MANDATO SOLO DA GCU AD EFI
#define GCU_CLUTCH_FB_SW_ID             ((uint16_t)(784))
#define GCU_GEAR_TIMING_TELEMETRY_ID    ((uint16_t)(1624))

/********************* STEERING WHEEL *********************/
#define SW_FIRE_GCU_ID                  ((uint16_t)(516))
#define SW_GEARSHIFT_ID                 ((uint16_t)(512))
#define SW_CLUTCH_TARGET_GCU_ID         ((uint16_t)(513))
#define SW_LAUNCH_CONTROL_GCU_ID        ((uint16_t)(514))
#define SW_TRACTION_CONTROL_GCU_ID      ((uint16_t)(515))
#define SW_BRAKE_BIAS_EBB_ID            ((uint16_t)(1024))
#define SW_DRS_GCU_ID                   ((uint16_t)(517))

/************************** DCU ***************************/
#define DCU_GEAR_TIMING_GCU_ID          ((uint16_t)(518))
#define DCU_AUTO_GEARSHIFT_GCU_ID       ((uint16_t)(519))

/************************** DAU ***************************/
#define DAU_FR_ID                       ((uint16_t)(1616))
#define DAU_FL_ID                       ((uint16_t)(1617))
#define DAU_REAR_ID                     ((uint16_t)(1618))
#define DAU_FR_APPS_ID                  ((uint16_t)(1619))
#define IR_FL_ID                        ((uint16_t)(1620))
#define IR_FR_ID                        ((uint16_t)(1621))
#define IR_RL_ID                        ((uint16_t)(1622))
#define IR_RR_ID                        ((uint16_t)(1623))

/************************** IMU ***************************/
#define IMU_DATA_1_ID                   ((uint16_t)(1802))
#define IMU_DATA_2_ID                   ((uint16_t)(1803))
#define IMU_DATA_3_ID                   ((uint16_t)(1804))

/************************** EBB ***************************/
#define EBB_BIAS_ID                     ((uint16_t)(1805))

/************************* DEBUG **************************/
#define DAU_FR_DEBUG_ID                 ((uint16_t)(785))
#define DAU_FL_DEBUG_ID                 ((uint16_t)(786))
#define DAU_REAR_DEBUG_ID               ((uint16_t)(787))
#define SW_DEBUG_ID                     ((uint16_t)(788))
#define EBB_DEBUG_ID                    ((uint16_t)(789))
#define GCU_DEBUG_1_ID                  ((uint16_t)(790))
#define GCU_DEBUG_2_ID                  ((uint16_t)(791))
#define DCU_DEBUG_ID                    ((uint16_t)(792))

/************************** AUX ***************************/
#define SW_AUX_ID                       ((uint16_t)(2032))
#define GCU_AUX_ID                      ((uint16_t)(2033))
#define EBB_AUX_ID                      ((uint16_t)(2034))
#define DAU_FR_AUX_ID                   ((uint16_t)(2035))
#define DAU_FL_AUX_ID                   ((uint16_t)(2036))
#define DAU_REAR_AUX_ID                 ((uint16_t)(2037))
#define IMU_AUX_ID                      ((uint16_t)(2038))
#define DCU_AUX_ID                      ((uint16_t)(2039))


#endif
