/*
 * matrix_dyngsy.h
 *
 *  Created on: 27.05.2019
 *      Author: Adam
 */

#ifndef INC_MATRIX_DYNGSY_H_
#define INC_MATRIX_DYNGSY_H_


#include "main.h"

extern void arm_mat_init_f32(
		  arm_matrix_instance_f32 * S,
		  uint16_t nRows,
		  uint16_t nColumns,
		  float32_t * pData);
extern arm_status arm_mat_mult_f32(
		  const arm_matrix_instance_f32 * pSrcA,
		  const arm_matrix_instance_f32 * pSrcB,
		  arm_matrix_instance_f32 * pDst);
extern   arm_status arm_mat_trans_f32(
		  const arm_matrix_instance_f32 * pSrc,
		  arm_matrix_instance_f32 * pDst);
#endif
