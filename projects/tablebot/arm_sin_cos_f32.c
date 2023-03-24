/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_sin_cos_f32.c    
*    
* Description:	Sine and Cosine calculation for floating-point values.   
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the 
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.   
* -------------------------------------------------------------------- */

#include "arm_math.h"

/** This imports several code pieces from CMSIS v4.5 **/

#define FAST_MATH_TABLE_SIZE  512

/**   
 * \par    
 * Example code for the generation of the floating-point sine table:
 * <pre>
 * tableSize = 512;    
 * for(n = 0; n < (tableSize + 1); n++)    
 * {    
 *  sinTable[n]=sin(2*pi*n/tableSize);    
 * }</pre>    
 * \par    
 * where pi value is  3.14159265358979    
 */

const float32_t sinTable_f32[FAST_MATH_TABLE_SIZE + 1] = {
   0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
   0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
   0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
   0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
   0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
   0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
   0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
   0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
   0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
   0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
   0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
   0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
   0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
   0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
   0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
   0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
   0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
   0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
   0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
   0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
   0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
   0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
   0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
   0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
   0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
   0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
   0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
   0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
   0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
   0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
   0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
   0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
   0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
   0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
   0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
   0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
   0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
   0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
   0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
   0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
   0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
   0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
   0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
   -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
   -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
   -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
   -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
   -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
   -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
   -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
   -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
   -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
   -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
   -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
   -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
   -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
   -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
   -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
   -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
   -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
   -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
   -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
   -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
   -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
   -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
   -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
   -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
   -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
   -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
   -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
   -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
   -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
   -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
   -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
   -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
   -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
   -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
   -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
   -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
   -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
   -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
   -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
   -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
   -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
   -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
   -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
   -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
   -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
   -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
   -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
   -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
   -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
   -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
   -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};

/**    
 * @ingroup groupController    
 */

/**    
 * @defgroup SinCos Sine Cosine   
 *    
 * Computes the trigonometric sine and cosine values using a combination of table lookup   
 * and linear interpolation.     
 * There are separate functions for Q31 and floating-point data types.   
 * The input to the floating-point version is in degrees while the   
 * fixed-point Q31 have a scaled input with the range   
 * [-1 0.9999] mapping to [-180 +180] degrees.   
 *
 * The floating point function also allows values that are out of the usual range. When this happens, the function will
 * take extra time to adjust the input value to the range of [-180 180].
 *   
 * The implementation is based on table lookup using 360 values together with linear interpolation.   
 * The steps used are:   
 *  -# Calculation of the nearest integer table index.   
 *  -# Compute the fractional portion (fract) of the input.   
 *  -# Fetch the value corresponding to \c index from sine table to \c y0 and also value from \c index+1 to \c y1.      
 *  -# Sine value is computed as <code> *psinVal = y0 + (fract * (y1 - y0))</code>.    
 *  -# Fetch the value corresponding to \c index from cosine table to \c y0 and also value from \c index+1 to \c y1.      
 *  -# Cosine value is computed as <code> *pcosVal = y0 + (fract * (y1 - y0))</code>.    
 */

 /**    
 * @addtogroup SinCos    
 * @{    
 */

/**    
 * @brief  Floating-point sin_cos function.   
 * @param[in]  theta    input value in degrees    
 * @param[out] *pSinVal points to the processed sine output.    
 * @param[out] *pCosVal points to the processed cos output.    
 * @return none.   
 */

void arm_sin_cos_f32(
  float32_t theta,
  float32_t * pSinVal,
  float32_t * pCosVal)
{
  float32_t fract, in;                             /* Temporary variables for input, output */
  uint16_t indexS, indexC;                         /* Index variable */
  float32_t f1, f2, d1, d2;                        /* Two nearest output values */
  int32_t n;
  float32_t findex, Dn, Df, temp;

  /* input x is in degrees */
  /* Scale the input, divide input by 360, for cosine add 0.25 (pi/2) to read sine table */
  in = theta * 0.00277777777778f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if (in < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1) */
  in = in - (float32_t) n;
  if (in >= 1.0f)
  {
    in = 0.0f;
  }

  /* Calculation of index of the table */
  findex = (float32_t) FAST_MATH_TABLE_SIZE * in;
  indexS = ((uint16_t)findex) & 0x1ff;
  indexC = (indexS + (FAST_MATH_TABLE_SIZE / 4)) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float32_t) indexS;

  /* Read two nearest values of input value from the cos & sin tables */
  f1 = sinTable_f32[indexC];
  f2 = sinTable_f32[(indexC+1) % FAST_MATH_TABLE_SIZE];
  d1 = -sinTable_f32[indexS];
  d2 = -sinTable_f32[(indexS+1) % FAST_MATH_TABLE_SIZE];

  Dn = 0.0122718463030f; // delta between the two points (fixed), in this case 2*pi/FAST_MATH_TABLE_SIZE
  Df = f2 - f1; // delta between the values of the functions
  temp = Dn*(d1 + d2) - 2*Df;
  temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
  temp = fract*temp + d1*Dn;

  /* Calculation of cosine value */
  *pCosVal = fract*temp + f1;

  /* Read two nearest values of input value from the cos & sin tables */
  f1 = sinTable_f32[indexS];
  f2 = sinTable_f32[(indexS+1) % FAST_MATH_TABLE_SIZE];
  d1 = sinTable_f32[indexC];
  d2 = sinTable_f32[(indexC+1) % FAST_MATH_TABLE_SIZE];

  Df = f2 - f1; // delta between the values of the functions
  temp = Dn*(d1 + d2) - 2*Df;
  temp = fract*temp + (3*Df - (d2 + 2*d1)*Dn);
  temp = fract*temp + d1*Dn;
  
  /* Calculation of sine value */
  *pSinVal = fract*temp + f1;
}
/**    
 * @} end of SinCos group    
 */
 