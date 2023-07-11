//Created by Carlos Álvarez Cía 2023

#include "Common.h"

COMMON::Common common_local;
COMMON::Common &COMMON::get_common(){
    return common_local;
}