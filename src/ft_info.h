/* Copyright 2024 Litz Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/***************************************************************************************
 * File         : ft_info.h
 * Author       : Mingsheng Xu <mxu61@ucsc.edu>
 * Date         : 06/16/2024
 * Description  :
 ***************************************************************************************/

#ifndef __FT_INFO_H__
#define __FT_INFO_H__

typedef enum FT_Ended_By_enum {
  FT_NOT_ENDED,
  FT_ICACHE_LINE_BOUNDARY,
  FT_TAKEN_BRANCH,
  FT_BAR_FETCH,
  FT_APP_EXIT
} FT_Ended_By;

typedef struct FT_Info_Static_struct {
  // the PC of the first inst in this FT
  Addr start;
  // the FT size in bytes, counting from the first byte of the first inst to the last byte of the last inst
  Addr length;
  // the number of uops in this FT
  int n_uops;
} FT_Info_Static;

// two fts with the same static info may have different dynamic info
typedef struct FT_Info_Dynamic_struct {
  // the termination of the FT
  FT_Ended_By ended_by;
  // if the first op of this FT is off-path
  Flag first_op_off_path;
} FT_Info_Dynamic;

struct FT_Info_struct {
  FT_Info_Static static_info;
  FT_Info_Dynamic dynamic_info;
};

#endif