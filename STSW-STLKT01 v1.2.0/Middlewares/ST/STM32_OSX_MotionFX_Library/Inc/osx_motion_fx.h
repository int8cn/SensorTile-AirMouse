/**
  ******************************************************************************
  * @file    osx_motion_fx.h
  * @author  VMA Application Team
  * @version V1.0.7
  * @date    17-September-2015
  * @brief   Header for OSX MotionFX module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _OSX_MOTION_FX_H_
#define _OSX_MOTION_FX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup OSX_MOTION_FX OSX_MOTION_FX
  * @{
  */

/** @defgroup OSX_MOTION_FX_Exported_Defines OSX_MOTION_FX_Exported_Defines
 * @{
 */
/* Exported types ------------------------------------------------------------*/
#define NUM_AXES    3
#define QNUM_AXES   4
/**
  * @}
  */

/** @defgroup OSX_MOTION_FX_Exported_Types OSX_MOTION_FX_Exported_Types
 * @{
 */
typedef enum
{
  OSXMFX_ENGINE_DISABLE = 0,
  OSXMFX_ENGINE_ENABLE = 1
} osxMFX_Engine_State;

typedef enum
{
  OSXMFX_ENGINE_OUTPUT_NED = 0,
  OSXMFX_ENGINE_OUTPUT_ENU = 1
} osxMFX_Engine_Output_Ref_Sys;

typedef struct
{
  float ATime;                              /* merge rate to the accel */
  float MTime;                              /* merge rate to the mag */
  float FrTime;                             /* merge rate to the accel when external accelerations occours */
  unsigned char LMode;                      /* gyro bias learn mode: 1-static learning 2-dynamic learning */
  float gbias_mag_th_sc_6X;                 /* 6 axes scaler for the gyro bias mag threshold nominal */
  float gbias_acc_th_sc_6X;                 /* 6 axes scaler for the gyro bias acc threshold nominal */
  float gbias_gyro_th_sc_6X;                /* 6 axes scaler for the gyro bias gyro threshold nominal */
  float gbias_mag_th_sc_9X;                 /* 9 axes scaler for the gyro bias mag threshold nominal */
  float gbias_acc_th_sc_9X;                 /* 9 axes scaler for the gyro bias acc threshold nominal */
  float gbias_gyro_th_sc_9X;                /* 9 axes scaler for the gyro bias gyro threshold nominal */
  unsigned char modx;                       /* setting to indicate the decimation,
                                               set to 1 in smartphone/tablet
                                               set to >=1 in embedded solutions */
  char acc_orientation[QNUM_AXES];          /* accelerometer data orientation */
  char gyro_orientation[QNUM_AXES];         /* gyroscope data orientation */
  char mag_orientation[QNUM_AXES];          /* magnetometer data orientation */
  osxMFX_Engine_Output_Ref_Sys output_type; /* 0: NED, 1: ENU */
  int start_automatic_gbias_calculation;
} osxMFX_knobs;

typedef struct
{
  float mag[NUM_AXES];  /* calibrated mag [uT]/50 */
  float acc[NUM_AXES];  /* acc [g] */
  float gyro[NUM_AXES]; /* gyro [dps] */
} osxMFX_input;

typedef struct
{
  float rotation_9X[NUM_AXES];            /* 9 axes yaw, pitch and roll */
  float quaternion_9X[QNUM_AXES];         /* 9 axes quaternion */
  float gravity_9X[NUM_AXES];             /* 9 axes device frame gravity */
  float linear_acceleration_9X[NUM_AXES]; /* 9 axes device frame linear acceleration */
  float heading_9X;                       /* 9 axes heading */
  float rotation_6X[NUM_AXES];            /* 6 axes yaw, pitch and roll */
  float quaternion_6X[QNUM_AXES];         /* 6 axes quaternion */
  float gravity_6X[NUM_AXES];             /* 6 axes device frame gravity */
  float linear_acceleration_6X[NUM_AXES]; /* 6 axes device frame linear acceleration */
  float heading_6X;                       /* 6 axes heading */
} osxMFX_output;

typedef struct
{
  signed short magOffX; /* X axis Offset */
  signed short magOffY; /* Y axis Offset */
  signed short magOffZ; /* Z axis Offset */
  float magGainX;       /* X axis Gain  */
  float magGainY;       /* Y axis Gain  */
  float magGainZ;       /* Z axis Gain  */
  float expMagVect;     /* expected magnetic field */
} osxMFX_calibFactor;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/** @defgroup OSX_MOTION_FX_Exported_Functions OSX_MOTION_FX_Exported_Functions
 * @{
 */
/**
 * @brief  Initialize the MotionFX engine
 * @retval 1 in case of success, 0 otherwise
 */
uint8_t osx_MotionFX_initialize(void);

/**
 * @brief  Set the internal knobs
 * @param  knobs knobs structure
 * @retval None
 */
void osx_MotionFX_setKnobs(osxMFX_knobs *knobs);

/**
 * @brief  Get the current internal knobs
 * @param  knobs knobs structure
 * @retval None
 */
void osx_MotionFX_getKnobs(osxMFX_knobs *knobs);

/**
 * @brief  Get the status of the 6 axes library
 * @retval 1 if enabled, 0 if disabled
 */
osxMFX_Engine_State osx_MotionFX_getStatus_6X(void);

/**
 * @brief  Get the status of the 9 axes library
 * @retval 1 if enabled, 0 if disabled
 */
osxMFX_Engine_State osx_MotionFX_getStatus_9X(void);

/**
 * @brief  Enable or disable the 6 axes function (ACC + GYRO)
 * @param  enable 1 to enable, 0 to disable
 * @retval None
 */
void osx_MotionFX_enable_6X(osxMFX_Engine_State enable);

/**
 * @brief  Enable or disable the 9 axes function (ACC + GYRO + MAG)
 * @param  enable 1 to enable, 0 to disable
 * @retval None
 */
void osx_MotionFX_enable_9X(osxMFX_Engine_State enable);

/**
 * @brief  Set the initial gbias
 * @param  gbias pointer to a float array containing the 3 gbias values
 * @retval None
 */
void osx_MotionFX_setGbias(float *gbias);

/**
 * @brief  Get the initial gbias
 * @param  pointer to a float array containing the 3 gbias values
 * @retval None
 */
void osx_MotionFX_getGbias(float *gbias);

/**
 * @brief  Run the Kalman filter update
 * @param  data_out pointer to the osxMFX_output structure
 * @param  data_in pointer to the osxMFX_input structure
 * @param  eml_deltatime delta time between two propagate calls [sec]
 * @param  eml_q_update set to NULL
 * @retval None
 */
void osx_MotionFX_update(osxMFX_output *data_out, osxMFX_input *data_in, float eml_deltatime, float *eml_q_update);

/**
 * @brief  Run the Kalman filter propagate
 * @param  data_out pointer to the osxMFX_output structure
 * @param  data_in pointer to the osxMFX_input structure
 * @param  eml_deltatime delta time between two propagate calls [sec]
 * @retval None
 */
void osx_MotionFX_propagate(osxMFX_output *data_out, osxMFX_input *data_in, float eml_deltatime);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
int osx_MotionFX_getLibVersion(char *version);

/**
 * @brief  Initialize the compass calibration library
 * @retval None
 */
void osx_MotionFX_compass_init(void);

/**
 * @brief  Save accelerometer data ENU systems coordinate
 * @param  acc_x accelerometer x coordinate in [mg]
 * @param  acc_y accelerometer y coordinate in [mg]
 * @param  acc_z accelerometer z coordinate in [mg]
 * @retval None
 */
void osx_MotionFX_compass_saveAcc(int acc_x, int acc_y, int acc_z);

/**
 * @brief  Save magnetometer data ENU systems coordinate
 * @param  mag_x magnetometer x coordinate in [mG]
 * @param  mag_y magnetometer y coordinate in [mG]
 * @param  mag_z magnetometer z coordinate in [mG]
 * @retval None
 */
void osx_MotionFX_compass_saveMag(int mag_x, int mag_y, int mag_z);

/**
 * @brief  Run compass API at 25 Hz
 * @retval None
 */
int osx_MotionFX_compass_run(void);

/**
 * @brief  Check if calibration is needed
 * @retval calibration status
 */
unsigned char osx_MotionFX_compass_isCalibrated(void);

/**
 * @brief  Force new calibration
 * @retval None
 */
void osx_MotionFX_compass_forceReCalibration(void);

/**
 * @brief  Get calibration data
 * @param  CalibrationData  pointer to calibration data structure
 * @retval None
 */
void osx_MotionFX_getCalibrationData(osxMFX_calibFactor* CalibrationData);

/**
 * @brief  Set calibration data
 * @param  CalibrationData  pointer to calibration data structure
 * @retval None
 */
void osx_MotionFX_setCalibrationData(osxMFX_calibFactor* CalibrationData);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _OSX_MOTION_FX_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
