/* Copyright 2020 HPS/SAFARI Research Groups */
#ifndef __PREF_MLOP_PARAM_H__
#define __PREF_MLOP_PARAM_H__

#include "globals/global_types.h"

#define DEF_PARAM(name, variable, type, func, def, const) extern const type variable;
#include "pref_mlop.param.def"
#undef DEF_PARAM

#endif  // __PREF_MLOP_PARAM_H__
