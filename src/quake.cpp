//
// Quake map viewer
//
// Author: Johan Gardhage <johan.gardhage@gmail.com>
//
#include <stdio.h>
#include <math.h>
#include <float.h>
#include "lib/retro.h"
#include "lib/retromain.h"
#include "lib/retrobsp.h"
#include "lib/retromath.h"
#include "lib/retrocamera.h"
#include "lib/retropoly2.h"

#define MOVEMENT_SPEED		8.0

// A surface queued for drawing this frame, with its front-to-back sort key
struct visiblesurface_t
{
	int surface;		// surface (face) index
	float distance;		// squared distance from the camera
};

struct world_t
{
	RETRO_BSP *map = NULL;
	primdesc_t *surfacePrimitives = NULL;
	texture_t *textures = NULL;
	int numTextures = 0;
	visiblesurface_t *sortedVisibleSurfaces = NULL;
	int numMaxEdgesPerSurface = 0;
	int framebufferWidth = 0;
	int framebufferHeight = 0;
	unsigned char *framebuffer = NULL;
	float *depthbuffer = NULL;
	float frustumRatio = 1.0f;
	vec3_t viewOrigin;
	vec3_t viewForward;
	vec3_t viewSide;
	vec3_t viewUp;
	double textureTime = 0.0;
};

world_t world;
RETRO_BSP map;
RETRO_Camera camera;

// Deterministic fallback palette colour derived from an id, used when a surface
// has no usable texture
unsigned char SurfaceColor(int surface)
{
	unsigned int hash = (unsigned int)surface * 1103515245u + 12345u;
	return 32 + ((hash >> 8) & 0x7f);
}

// True if two "+" animated texture names belong to the same sequence
// (identical past the leading "+N" frame marker)
bool IsSameTextureAnimation(const char *a, const char *b)
{
	if (!a || !b || a[0] != '+' || b[0] != '+') {
		return false;
	}
	for (int i = 2; i < 16; i++) {
		if (a[i] != b[i]) {
			return false;
		}
		if (a[i] == '\0') {
			return true;
		}
	}
	return true;
}

// Frame number 0-9 of a "+N..." animated texture, or -1 if it is not animated
int TextureAnimationFrame(const char *name)
{
	if (!name || name[0] != '+') {
		return -1;
	}
	if (name[1] >= '0' && name[1] <= '9') {
		return name[1] - '0';
	}
	return -1;
}

//
// Clear software color and depth buffers
//
void ClearBuffers(world_t *world)
{
	int numPixels = world->framebufferWidth * world->framebufferHeight;
	for (int i = 0; i < numPixels; i++) {
		world->framebuffer[i] = 0;
		world->depthbuffer[i] = 0.0f;	// w-buffer: 0 = infinitely far, larger 1/w is nearer
	}
}

//
// Setup the CPU-side viewing basis
//
void UpdateView(world_t *world, RETRO_Camera *camera)
{
	float worldUp[3] = { 0.0f, 0.0f, 1.0f };

	world->viewOrigin[0] = camera->head[0];
	world->viewOrigin[1] = camera->head[1];
	world->viewOrigin[2] = camera->head[2];

	world->viewForward[0] = camera->view[0];
	world->viewForward[1] = camera->view[1];
	world->viewForward[2] = camera->view[2];
	Normalize(world->viewForward);

	Cross(world->viewForward, worldUp, world->viewSide);
	Normalize(world->viewSide);
	Cross(world->viewSide, world->viewForward, world->viewUp);
}

//
// Transform a world-space vertex to clip coordinates
//
void TransformVertex(world_t *world, const vec3_t vertex, float clip[4])
{
	const float nearPlane = 1.0f;
	const float farPlane = 5000.0f;

	// Project the vertex onto the view basis to get eye-space coordinates
	// (eyeZ is negated so the view direction points down -Z).
	float delta[3] = {
		vertex[0] - world->viewOrigin[0],
		vertex[1] - world->viewOrigin[1],
		vertex[2] - world->viewOrigin[2]
	};

	float eyeX = DotProduct(delta, world->viewSide);
	float eyeY = DotProduct(delta, world->viewUp);
	float eyeZ = -DotProduct(delta, world->viewForward);

	// Perspective projection. clip[3] (= eye distance) is the w for the perspective
	// divide; clip[2] maps depth into the near/far range.
	clip[0] = nearPlane * eyeX;
	clip[1] = (nearPlane / world->frustumRatio) * eyeY;
	clip[2] = -((farPlane + nearPlane) / (farPlane - nearPlane)) * eyeZ
			- ((2.0f * farPlane * nearPlane) / (farPlane - nearPlane));
	clip[3] = -eyeZ;
}

//
// Traverse the BSP tree to find the leaf containing visible surfaces from a specific position
//
dleaf_t *FindLeaf(world_t *world, RETRO_Camera *camera)
{
	dleaf_t *leaf = NULL;

	// Fetch the start node
	dnode_t *node = world->map->getStartNode();

	while (!leaf) {
		short nextNodeId;

		// Get a pointer to the plane which intersects the node
		dplane_t *plane = world->map->getPlane(node->planenum);

		// Calculate distance to the intersecting plane
		float distance = DotProduct(plane->normal, camera->head);

		// If the camera is in front of the plane, traverse the right (front) node, otherwise traverse the left (back) node
		if (distance > plane->dist) {
			nextNodeId = node->children[0];
		} else {
			nextNodeId = node->children[1];
		}

		// If next node >= 0, traverse the node, otherwise use the inverse of the node as the index to the leaf (and we are done!)
		if (nextNodeId >= 0) {
			node = world->map->getNode(nextNodeId);
		} else {
			leaf = world->map->getLeaf(~nextNodeId);
		}
	}

	return leaf;
}

//
// Check if the camera is on the drawable side of a BSP surface
//
bool IsSurfaceVisible(world_t *world, int surface)
{
	dface_t *bspSurface = world->map->getSurface(surface);
	dplane_t *plane = world->map->getPlane(bspSurface->planenum);
	float distance = DotProduct(plane->normal, world->viewOrigin) - plane->dist;

	// side selects which half-space the face points into: the face normal equals the
	// plane normal for side 0 and is reversed for side 1.
	if (bspSurface->side) {
		return distance <= 0.0f;
	}
	return distance >= 0.0f;
}

//
// Calculate approximate front-to-back surface distance from the camera
//
float CalculateSurfaceDistance(world_t *world, int surface)
{
	// Squared distance from the camera to the face's first vertex - cheap, and good
	// enough as a front-to-back sort key.
	primdesc_t *primitives = &world->surfacePrimitives[world->numMaxEdgesPerSurface * surface];
	vec3_t delta = {
		primitives[0].v[0] - world->viewOrigin[0],
		primitives[0].v[1] - world->viewOrigin[1],
		primitives[0].v[2] - world->viewOrigin[2]
	};
	return DotProduct(delta, delta);
}

//
// Gather the back-face-culled surfaces of one leaf into the visible surface list
//
void CollectLeafSurfaces(world_t *world, int leafIndex, int &numVisibleSurfaces)
{
	dleaf_t *leaf = world->map->getLeaf(leafIndex);
	int firstSurface = leaf->firstmarksurface;
	int lastSurface = firstSurface + leaf->nummarksurfaces;
	int maxVisibleSurfaces = world->map->getNumSurfaceLists();
	for (int k = firstSurface; k < lastSurface; k++) {
		int surface = world->map->getSurfaceList(k);
		if (numVisibleSurfaces < maxVisibleSurfaces && IsSurfaceVisible(world, surface)) {
			world->sortedVisibleSurfaces[numVisibleSurfaces].surface = surface;
			world->sortedVisibleSurfaces[numVisibleSurfaces].distance = CalculateSurfaceDistance(world, surface);
			numVisibleSurfaces++;
		}
	}
}

//
// Sort visible surfaces front-to-back
//
void SortVisibleSurfaces(visiblesurface_t *surfaces, int numSurfaces)
{
	// Insertion sort by distance, nearest first.
	for (int i = 1; i < numSurfaces; i++) {
		visiblesurface_t current = surfaces[i];
		int j = i - 1;
		while (j >= 0 && surfaces[j].distance > current.distance) {
			surfaces[j + 1] = surfaces[j];
			j--;
		}
		surfaces[j + 1] = current;
	}
}

//
// Estimate one stable mip level for a whole BSP surface
//
int EstimateSurfaceMipLevel(world_t *world, primdesc_t *primitives, int numEdges, texture_t *texture)
{
	const float nearPlane = 1.0f;
	if (!texture || !texture->levels || texture->numLevels <= 1 || numEdges < 2) {
		return 0;
	}

	// Find the worst-case texture density (texels per screen pixel) along the
	// surface edges, then pick the mip level that matches it.
	float maxTexelsPerPixel = 1.0f;
	for (int i = 0; i < numEdges; i++) {
		primdesc_t *a = &primitives[i];
		primdesc_t *b = &primitives[(i + 1) % numEdges];
		float clipA[4];
		float clipB[4];
		TransformVertex(world, a->v, clipA);
		TransformVertex(world, b->v, clipB);

		if (clipA[3] < nearPlane || clipB[3] < nearPlane) {
			continue;
		}

		float invWA = 1.0f / clipA[3];
		float invWB = 1.0f / clipB[3];
		float ax = (clipA[0] * invWA * 0.5f + 0.5f) * (float)(world->framebufferWidth - 1);
		float ay = (0.5f - clipA[1] * invWA * 0.5f) * (float)(world->framebufferHeight - 1);
		float bx = (clipB[0] * invWB * 0.5f + 0.5f) * (float)(world->framebufferWidth - 1);
		float by = (0.5f - clipB[1] * invWB * 0.5f) * (float)(world->framebufferHeight - 1);
		float dx = bx - ax;
		float dy = by - ay;
		float screenLength = sqrtf(dx * dx + dy * dy);
		if (screenLength < 1.0f) {
			continue;
		}

		float du = (b->t[0] - a->t[0]) * (float)texture->levels[0].width;
		float dv = (b->t[1] - a->t[1]) * (float)texture->levels[0].height;
		float texelLength = sqrtf(du * du + dv * dv);
		float texelsPerPixel = texelLength / screenLength;
		if (texelsPerPixel > maxTexelsPerPixel) {
			maxTexelsPerPixel = texelsPerPixel;
		}
	}

	// Each mip step halves resolution, so the level is log2 of the density.
	int level = (int)floorf(log2f(maxTexelsPerPixel));
	if (level < 0) {
		level = 0;
	} else if (level >= texture->numLevels) {
		level = texture->numLevels - 1;
	}
	return level;
}

//
// Resolve a BSP texture animation frame for this render time
//
texture_t *ResolveTextureAnimation(world_t *world, texture_t *texture)
{
	if (!texture || texture->animTotal <= 1) {
		return texture;
	}

	// Quake animates textures at 10 frames per second
	int frame = ((int)(world->textureTime * 10.0)) % texture->animTotal;
	int textureIndex = texture->animFrames[frame];
	if (textureIndex < 0) {
		textureIndex = texture->animFrames[0];
	}
	if (textureIndex < 0 || textureIndex >= world->numTextures) {
		return texture;
	}
	return &world->textures[textureIndex];
}

//
// Draw the surface
//
void DrawSurface(world_t *world, int surface, const drawcontext_t *ctx)
{
	// Get the surface primitive
	primdesc_t *primitives = &world->surfacePrimitives[world->numMaxEdgesPerSurface * surface];
	texinfo_t *textureInfo = world->map->getTextureInfo(surface);
	texture_t *texture = NULL;
	int numEdges = world->map->getNumEdges(surface);
	unsigned char color = SurfaceColor(textureInfo->miptex);

	if (numEdges < 3) {
		return;
	}

	if (textureInfo->miptex >= 0 && textureInfo->miptex < world->numTextures && world->textures[textureInfo->miptex].levels) {
		texture = &world->textures[textureInfo->miptex];
	}
	texture = ResolveTextureAnimation(world, texture);

	int mipLevel = EstimateSurfaceMipLevel(world, primitives, numEdges, texture);

	// Fan-triangulate the face (BSP faces are convex) and draw each triangle.
	rendervertex_t first;
	rendervertex_t previous;
	TransformVertex(world, primitives[0].v, first.v);
	first.t[0] = primitives[0].t[0];
	first.t[1] = primitives[0].t[1];
	TransformVertex(world, primitives[1].v, previous.v);
	previous.t[0] = primitives[1].t[0];
	previous.t[1] = primitives[1].t[1];

	for (int i = 2; i < numEdges; i++) {
		rendervertex_t current;
		TransformVertex(world, primitives[i].v, current.v);
		current.t[0] = primitives[i].t[0];
		current.t[1] = primitives[i].t[1];
		DrawTriangle(ctx, first, previous, current, texture, mipLevel, color);
		previous = current;
	}
}

//
// Calculate which other leaves are visible from the specified leaf, fetch the associated surfaces and draw them
//
void DrawLeafVisibleSet(world_t *world, dleaf_t *pLeaf, const drawcontext_t *ctx)
{
	int numVisibleSurfaces = 0;
	// Leaves are numbered 1..numLeaves; bit (i-1) of the PVS maps to leaf i.
	int numLeaves = world->map->getNumLeaves();

	if (pLeaf->visofs < 0) {
		// No visibility information for this leaf: treat every leaf as potentially visible.
		for (int i = 1; i <= numLeaves; i++) {
			CollectLeafSurfaces(world, i, numVisibleSurfaces);
		}
	} else {
		// Decompress the run-length encoded PVS. A zero byte means "skip the next
		// (8 * following byte) leaves"; any other byte holds 8 visibility bits,
		// least-significant bit first.
		unsigned char *visibilityList = world->map->getVisibilityList(pLeaf->visofs);
		for (int i = 1; i <= numLeaves; ) {
			if (*visibilityList == 0) {
				i += 8 * visibilityList[1];
				visibilityList += 2;
			} else {
				for (int bit = 1; bit < 256 && i <= numLeaves; bit <<= 1, i++) {
					if (*visibilityList & bit) {
						CollectLeafSurfaces(world, i, numVisibleSurfaces);
					}
				}
				visibilityList++;
			}
		}
	}

	// Draw the copied surfaces
	SortVisibleSurfaces(world->sortedVisibleSurfaces, numVisibleSurfaces);
	for (int i = 0; i < numVisibleSurfaces; i++) {
		DrawSurface(world, world->sortedVisibleSurfaces[i].surface, ctx);
	}
}

//
// Draw the scene
//
void DrawScene(world_t *world, RETRO_Camera *camera, unsigned char *framebuffer, double deltaTime)
{
	world->framebuffer = framebuffer;
	world->textureTime += deltaTime;
	ClearBuffers(world);

	// Setup the CPU-side view transform used by DrawSurface.
	UpdateView(world, camera);

	// Find the leaf the camera is in
	dleaf_t *leaf = FindLeaf(world, camera);

	// Setup the drawing context
	drawcontext_t ctx = {
		world->framebuffer,
		world->depthbuffer,
		world->framebufferWidth,
		world->framebufferHeight,
		world->textureTime
	};

	// Render the scene
	DrawLeafVisibleSet(world, leaf, &ctx);
}

//
// Copy BSP texture data into CPU indexed mipmaps
//
bool DecodeTextures(world_t *world)
{
	world->numTextures = world->map->getNumTextures();
	world->textures = new texture_t [world->numTextures];

	// First pass: decode each BSP texture's four mip levels into CPU buffers.
	for (int i = 0; i < world->numTextures; i++) {
		miptex_t *mipTexture = world->map->getMipTexture(i);

		world->textures[i].numLevels = 0;
		world->textures[i].levels = NULL;
		world->textures[i].isSky = false;
		world->textures[i].isTurbulent = false;
		world->textures[i].animTotal = 0;
		for (int frame = 0; frame < 10; frame++) {
			world->textures[i].animFrames[frame] = -1;
		}

		// NULL textures exist, and a zero mip offset means the slot has no texel data.
		if (!mipTexture || !mipTexture->name[0] || mipTexture->offsets[0] == 0) {
			continue;
		}

		int width = mipTexture->width;
		int height = mipTexture->height;
		unsigned int mipOffsets[4] = {
			mipTexture->offsets[0],
			mipTexture->offsets[1],
			mipTexture->offsets[2],
			mipTexture->offsets[3]
		};
		int numLevels = 4;

		world->textures[i].numLevels = numLevels;
		world->textures[i].levels = new miplevel_t [numLevels];
		for (int level = 0; level < numLevels; level++) {
			world->textures[i].levels[level].width = 0;
			world->textures[i].levels[level].height = 0;
			world->textures[i].levels[level].pixels = NULL;
		}

		// Copy each mip level (full, 1/2, 1/4, 1/8) out of the BSP into its own buffer.
		for (int level = 0; level < numLevels; level++) {
			miplevel_t *current = &world->textures[i].levels[level];
			current->width = width >> level;
			current->height = height >> level;
			if (current->width < 1) current->width = 1;
			if (current->height < 1) current->height = 1;
			current->pixels = new unsigned char [current->width * current->height];

			unsigned char *rawTexture = (unsigned char *)mipTexture + mipOffsets[level];
			for (int y = 0; y < current->height; y++) {
				for (int x = 0; x < current->width; x++) {
					current->pixels[x + y * current->width] = rawTexture[x + y * current->width];
				}
			}
		}
	}

	// Second pass: collect the "+0".."+9" frames of each animation into its "+0" texture.
	for (int i = 0; i < world->numTextures; i++) {
		miptex_t *baseTexture = world->map->getMipTexture(i);
		if (!baseTexture) {
			continue;
		}
		// Only the "+0" texture of a sequence owns the frame list.
		int baseFrame = TextureAnimationFrame(baseTexture->name);
		if (baseFrame != 0) {
			continue;
		}

		for (int j = 0; j < world->numTextures; j++) {
			miptex_t *frameTexture = world->map->getMipTexture(j);
			if (!frameTexture || !IsSameTextureAnimation(baseTexture->name, frameTexture->name)) {
				continue;
			}

			int frame = TextureAnimationFrame(frameTexture->name);
			if (frame >= 0 && frame < 10) {
				world->textures[i].animFrames[frame] = j;
				if (frame + 1 > world->textures[i].animTotal) {
					world->textures[i].animTotal = frame + 1;
				}
			}
		}
	}

	return true;
}

//
// Calculate primitive surfaces
//
bool DecodeSurfaces(world_t *world)
{
	// Allocate memory for the sorted visible surfaces array
	world->sortedVisibleSurfaces = new visiblesurface_t [world->map->getNumSurfaceLists()];

	// Calculate max number of edges per surface
	world->numMaxEdgesPerSurface = 0;
	for (int i = 0; i < world->map->getNumSurfaces(); i++) {
		if (world->numMaxEdgesPerSurface < world->map->getNumEdges(i)) {
			world->numMaxEdgesPerSurface = world->map->getNumEdges(i);
		}
	}

	// Allocate memory for the surface primitive array
	world->surfacePrimitives = new primdesc_t [world->map->getNumSurfaces() * world->numMaxEdgesPerSurface];

	// Loop through all surfaces to fetch their vertices and compute texture coords
	for (int i = 0; i < world->map->getNumSurfaces(); i++) {
		int numEdges = world->map->getNumEdges(i);
		dface_t *surface = world->map->getSurface(i);

		// Get a pointer to texinfo for this surface
		texinfo_t *textureInfo = world->map->getTextureInfo(i);
		// Get a pointer to the surface's miptextures
		miptex_t *mipTexture = world->map->getMipTexture(textureInfo->miptex);
		// Fall back to a unit size when the texture slot has no data (missing texture)
		float texWidth = (mipTexture && mipTexture->width) ? (float)mipTexture->width : 1.0f;
		float texHeight = (mipTexture && mipTexture->height) ? (float)mipTexture->height : 1.0f;

		// Point to a surface primitive array
		primdesc_t *primitives = &world->surfacePrimitives[i * world->numMaxEdgesPerSurface];

		for (int j = 0; j < numEdges; j++, primitives++) {
			// Get an edge id from the surface. Fetch the correct edge by using the id in the Edge List.
			// The winding is backwards!
			int edgeId = world->map->getEdgeList(surface->firstedge + (numEdges - 1 - j));
			// Positive surfedge -> edge used forwards (start vertex); otherwise reversed (end vertex)
			int vertexId = ((edgeId >= 0) ? world->map->getEdge(edgeId)->v[0] : world->map->getEdge(-edgeId)->v[1]);

			// Store the vertex in the primitive array
			vec3_t *vertex = world->map->getVertex(vertexId);
			primitives->v[0] = ((float *)vertex)[0];
			primitives->v[1] = ((float *)vertex)[1];
			primitives->v[2] = ((float *)vertex)[2];

			// Calculate the vertex's texture coords and store it in the primitive array
			float s = DotProduct(textureInfo->vecs[0], primitives->v) + textureInfo->vecs[0][3];
			float t = DotProduct(textureInfo->vecs[1], primitives->v) + textureInfo->vecs[1][3];
			primitives->t[0] = s / texWidth;
			primitives->t[1] = t / texHeight;
			primitives->l[0] = s;
			primitives->l[1] = t;
		}
	}

	return true;
}

//
// One-time setup: load the map, upload its palette, and build the world renderer
//
void DEMO_Initialize(void)
{
	map = RETRO_LoadBSP("assets/start.bsp", "assets/palette.lmp", "assets/colormap.lmp");
	if (!map.bsp) {
		RETRO_RageQuit("[ERROR] Quake::DEMO_Initialize() Unable to load BSP\n");
	}

	// Upload the BSP palette into the framebuffer's colour table.
	for (int i = 0; i < RETRO_COLORS; i++) {
		unsigned int color = map.palette[i];
		RETRO_SetColor(i, color & 0xff, (color >> 8) & 0xff, (color >> 16) & 0xff);
	}

	camera.SetPosition(540.0f, 260.0f, 100.0f);
	camera.SetYaw(90.0f);
	camera.SetPitch(0.0f);
	camera.SetMovementSpeed(MOVEMENT_SPEED);

	world.map = &map;
	world.framebufferWidth = RETRO_WIDTH;
	world.framebufferHeight = RETRO_HEIGHT;
	world.depthbuffer = new float [world.framebufferWidth * world.framebufferHeight];
	world.frustumRatio = (float)RETRO_HEIGHT / (float)RETRO_WIDTH;

	if (!DecodeTextures(&world)) {
		RETRO_RageQuit("[ERROR] Quake::DEMO_Initialize() Unable to decode world textures\n");
	}

	if (!DecodeSurfaces(&world)) {
		RETRO_RageQuit("[ERROR] Quake::DEMO_Initialize() Unable to decode world surfaces\n");
	}
}

//
// One-time teardown: release map and world resources
//
void DEMO_Deinitialize(void)
{
	RETRO_FreeBSP(&map);
	if (world.surfacePrimitives) delete[] world.surfacePrimitives;
	if (world.textures) {
		for (int i = 0; i < world.numTextures; i++) {
			if (world.textures[i].levels) {
				for (int j = 0; j < world.textures[i].numLevels; j++) {
					if (world.textures[i].levels[j].pixels) delete[] world.textures[i].levels[j].pixels;
				}
				delete[] world.textures[i].levels;
			}
		}
		delete[] world.textures;
	}
	if (world.sortedVisibleSurfaces) delete[] world.sortedVisibleSurfaces;
	if (world.depthbuffer) delete[] world.depthbuffer;
}

//
// Per-frame input: handle keyboard and mouse, then move the camera
//
void DEMO_Input(double deltatime)
{
	// Keyboard handling. Scale movement and turn rates by frame time so they are
	// independent of framerate.
	float scale = (float)(deltatime * RETRO_INPUT_FRAMERATE);
	if (RETRO_KeyState(SDL_SCANCODE_W) || RETRO_KeyState(SDL_SCANCODE_UP)) {
		camera.MoveForward(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_S) || RETRO_KeyState(SDL_SCANCODE_DOWN)) {
		camera.MoveBackward(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_RIGHT)) {
		camera.TurnRight(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_LEFT)) {
		camera.TurnLeft(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_D)) {
		camera.StrafeRight(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_A)) {
		camera.StrafeLeft(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_PAGEUP)) {
		camera.PitchUp(scale);
	}
	if (RETRO_KeyState(SDL_SCANCODE_PAGEDOWN)) {
		camera.PitchDown(scale);
	}

	// Mouse handling
	float xrel = 0.0f;
	float yrel = 0.0f;
	RETRO_MouseMotion(&xrel, &yrel);
	camera.Yaw(xrel);
	camera.Pitch(yrel);

	camera.UpdatePosition();
}

//
// Per-frame update: render the scene
//
void DEMO_Render(double deltatime)
{
	DrawScene(&world, &camera, RETRO_FrameBuffer(), deltatime);
}
