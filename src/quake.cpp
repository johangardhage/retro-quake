//
// Quake map viewer
//
// Author: Johan Gardhage <johan.gardhage@gmail.com>
//
#include <stdio.h>
#include <math.h>
#include "lib/retro.h"
#include "lib/retromain.h"
#include "lib/retrobsp.h"

RETRO_BSP map;

//
// One-time setup: load map header and verify binary statistics
//
void DEMO_Initialize(void)
{
	printf("=== Loading assets/start.bsp ===\n");

	// Load the BSP map file into memory
	if (!RETRO_LoadBSPMap(&map, "assets/start.bsp")) {
		printf("[ERROR] Failed to load assets/start.bsp\n");
		exit(-1);
	}

	printf("BSP Map loaded successfully!\n");
	printf("Version: %d (Expected: %d)\n", map.header->version, BSP_VERSION);

	// Extract and calculate statistics from file lump lengths
	int numVertices = map.getLump(LUMP_VERTEXES)->filelen / sizeof(dvertex_t);
	int numEdges = map.getLump(LUMP_EDGES)->filelen / sizeof(dedge_t);
	int numFaces = map.getNumSurfaces();
	int numTextures = map.getNumTextures();
	int numLeaves = map.getLump(LUMP_LEAFS)->filelen / sizeof(dleaf_t);
	int numModels = map.getLump(LUMP_MODELS)->filelen / sizeof(dmodel_t);

	printf("\n=== BSP Statistics ===\n");
	printf("Number of Models:    %d\n", numModels);
	printf("Number of Textures:  %d\n", numTextures);
	printf("Number of Vertices:  %d\n", numVertices);
	printf("Number of Edges:     %d\n", numEdges);
	printf("Number of Faces:     %d\n", numFaces);
	printf("Number of Leaves:    %d\n", numLeaves);
	printf("======================\n\n");

	// Free map resources since we are only printing stats at this stage
	RETRO_FreeBSP(&map);

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
