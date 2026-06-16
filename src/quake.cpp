//
// Quake map viewer
//
// Author: Johan Gardhage <johan.gardhage@gmail.com>
//
#include <stdio.h>
#include <math.h>
#include "lib/retro.h"
#include "lib/retromain.h"

//
// One-time setup: initialize rendering state
//
void DEMO_Initialize(void)
{
	// Initialize palette entry 0 to black
	RETRO_SetColor(0, 0, 0, 0);
}

//
// Per-frame update: keep the screen black
//
void DEMO_Render(double deltatime)
{
	// Nothing to draw yet; RETRO_Clear() leaves the screen black
}
