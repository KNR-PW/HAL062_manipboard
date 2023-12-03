#include "matrix_dyngsy.h"


void arm_mat_init_f32(
  arm_matrix_instance_f32 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float32_t * pData)
{
  /* Assign Number of Rows */
  S->numRows = nRows;

  /* Assign Number of Columns */
  S->numCols = nColumns;

  /* Assign Data pointer */
  S->pData = pData;
}
arm_status arm_mat_mult_f32(
  const arm_matrix_instance_f32 * pSrcA,
  const arm_matrix_instance_f32 * pSrcB,
  arm_matrix_instance_f32 * pDst)
{
  float32_t *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  float32_t *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  float32_t *pInA = pSrcA->pData;                /* input data matrix pointer A  */
  float32_t *pOut = pDst->pData;                 /* output data matrix pointer */
  float32_t *px;                                 /* Temporary output data matrix pointer */
  float32_t sum;                                 /* Accumulator */
  uint16_t numRowsA = pSrcA->numRows;            /* number of rows of input matrix A */
  uint16_t numColsB = pSrcB->numCols;            /* number of columns of input matrix B */
  uint16_t numColsA = pSrcA->numCols;            /* number of columns of input matrix A */

#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float32_t in1, in2, in3, in4;
  uint16_t col, i = 0u, j, row = numRowsA, colCnt;      /* loop counters */
  arm_status status;                             /* status of matrix multiplication */

#ifdef ARM_MATH_MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {

    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
    /* row loop */
    do
    {
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;

      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;

      /* For every row wise process, the pIn2 pointer is set
       ** to the starting address of the pSrcB data */
      pIn2 = pSrcB->pData;

      j = 0u;

      /* column loop */
      do
      {
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;

        /* Initiate the pointer pIn1 to point to the starting address of the column being processed */
        pIn1 = pInA;

        /* Apply loop unrolling and compute 4 MACs simultaneously. */
        colCnt = numColsA >> 2u;

        /* matrix multiplication        */
        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          in3 = *pIn2;
          pIn2 += numColsB;
          in1 = pIn1[0];
          in2 = pIn1[1];
          sum += in1 * in3;
          in4 = *pIn2;
          pIn2 += numColsB;
          sum += in2 * in4;

          in3 = *pIn2;
          pIn2 += numColsB;
          in1 = pIn1[2];
          in2 = pIn1[3];
          sum += in1 * in3;
          in4 = *pIn2;
          pIn2 += numColsB;
          sum += in2 * in4;
          pIn1 += 4u;

          /* Decrement the loop count */
          colCnt--;
        }

        /* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.
         ** No loop unrolling is used. */
        colCnt = numColsA % 0x4u;

        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;

          /* Decrement the loop counter */
          colCnt--;
        }

        /* Store the result in the destination buffer */
        *px++ = sum;

        /* Update the pointer pIn2 to point to the  starting address of the next column */
        j++;
        pIn2 = pSrcB->pData + j;

        /* Decrement the column loop counter */
        col--;

      } while(col > 0u);

#else

  /* Run the below code for Cortex-M0 */

  float32_t *pInB = pSrcB->pData;                /* input data matrix pointer B */
  uint16_t col, i = 0u, row = numRowsA, colCnt;  /* loop counters */
  arm_status status;                             /* status of matrix multiplication */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
     (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {

    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

  {
    /* The following loop performs the dot-product of each row in pInA with each column in pInB */
    /* row loop */
    do
    {
      /* Output pointer is set to starting address of the row being processed */
      px = pOut + i;

      /* For every row wise process, the column loop counter is to be initiated */
      col = numColsB;

      /* For every row wise process, the pIn2 pointer is set
       ** to the starting address of the pSrcB data */
      pIn2 = pSrcB->pData;

      /* column loop */
      do
      {
        /* Set the variable sum, that acts as accumulator, to zero */
        sum = 0.0f;

        /* Initialize the pointer pIn1 to point to the starting address of the row being processed */
        pIn1 = pInA;

        /* Matrix A columns number of MAC operations are to be performed */
        colCnt = numColsA;

        while(colCnt > 0u)
        {
          /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
          sum += *pIn1++ * (*pIn2);
          pIn2 += numColsB;

          /* Decrement the loop counter */
          colCnt--;
        }

        /* Store the result in the destination buffer */
        *px++ = sum;

        /* Decrement the column loop counter */
        col--;

        /* Update the pointer pIn2 to point to the  starting address of the next column */
        pIn2 = pInB + (numColsB - col);

      } while(col > 0u);

#endif /* #ifndef ARM_MATH_CM0 */

      /* Update the pointer pInA to point to the  starting address of the next row */
      i = i + numColsB;
      pInA = pInA + numColsA;

      /* Decrement the row loop counter */
      row--;

    } while(row > 0u);
    /* Set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;
  }

  /* Return to application */
  return (status);
}

arm_status arm_mat_trans_f32(
      const arm_matrix_instance_f32 * pSrc,
      arm_matrix_instance_f32 * pDst)
    {
      float32_t *pIn = pSrc->pData;                  /* input data matrix pointer */
      float32_t *pOut = pDst->pData;                 /* output data matrix pointer */
      float32_t *px;                                 /* Temporary output data matrix pointer */
      uint16_t nRows = pSrc->numRows;                /* number of rows */
      uint16_t nColumns = pSrc->numCols;             /* number of columns */

    #ifndef ARM_MATH_CM0

      /* Run the below code for Cortex-M4 and Cortex-M3 */

      uint16_t blkCnt, i = 0u, row = nRows;          /* loop counters */
      arm_status status;                             /* status of matrix transpose  */


    #ifdef ARM_MATH_MATRIX_CHECK


      /* Check for matrix mismatch condition */
      if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
      {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
      }
      else
    #endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

      {
        /* Matrix transpose by exchanging the rows with columns */
        /* row loop     */
        do
        {
          /* Loop Unrolling */
          blkCnt = nColumns >> 2;

          /* The pointer px is set to starting address of the column being processed */
          px = pOut + i;

          /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
           ** a second loop below computes the remaining 1 to 3 samples. */
          while(blkCnt > 0u)        /* column loop */
          {
            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Decrement the column loop counter */
            blkCnt--;
          }

          /* Perform matrix transpose for last 3 samples here. */
          blkCnt = nColumns % 0x4u;

          while(blkCnt > 0u)
          {
            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Decrement the column loop counter */
            blkCnt--;
          }

    #else

      /* Run the below code for Cortex-M0 */

      uint16_t col, i = 0u, row = nRows;             /* loop counters */
      arm_status status;                             /* status of matrix transpose  */


    #ifdef ARM_MATH_MATRIX_CHECK

      /* Check for matrix mismatch condition */
      if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
      {
        /* Set status as ARM_MATH_SIZE_MISMATCH */
        status = ARM_MATH_SIZE_MISMATCH;
      }
      else
    #endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

      {
        /* Matrix transpose by exchanging the rows with columns */
        /* row loop     */
        do
        {
          /* The pointer px is set to starting address of the column being processed */
          px = pOut + i;

          /* Initialize column loop counter */
          col = nColumns;

          while(col > 0u)
          {
            /* Read and store the input element in the destination */
            *px = *pIn++;

            /* Update the pointer px to point to the next row of the transposed matrix */
            px += nRows;

            /* Decrement the column loop counter */
            col--;
          }

    #endif /* #ifndef ARM_MATH_CM0 */

          i++;

          /* Decrement the row loop counter */
          row--;

        } while(row > 0u);          /* row loop end  */

        /* Set status as ARM_MATH_SUCCESS */
        status = ARM_MATH_SUCCESS;
      }

      /* Return to application */
      return (status);
    }


