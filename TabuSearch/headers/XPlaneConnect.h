#ifndef TS_XPLANE_CONNECT_H
#define TS_XPLANE_CONNECT_H

// This header includes all needed XPlaneConnect headers

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../XPlaneConnect/C/src/xplaneConnect.h"

#ifdef WIN32
#include <Windows.h>
#define sleep(n) Sleep(n * 1000)
#endif

#endif
