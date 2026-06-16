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

struct world_t
{
	RETRO_BSP *map = NULL;
	primdesc_t *surfacePrimitives = NULL;
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
// Draw the surface
//
void DrawSurface(world_t *world, int surface, const drawcontext_t *ctx)
{
	// Get the surface primitive
	primdesc_t *primitives = &world->surfacePrimitives[world->numMaxEdgesPerSurface * surface];
	int numEdges = world->map->getNumEdges(surface);
	unsigned char color = SurfaceColor(surface);

	if (numEdges < 3) {
		return;
	}

	// Fan-triangulate the face (BSP faces are convex) and draw each triangle.
	rendervertex_t first;
	rendervertex_t previous;
	TransformVertex(world, primitives[0].v, first.v);
	TransformVertex(world, primitives[1].v, previous.v);

	for (int i = 2; i < numEdges; i++) {
		rendervertex_t current;
		TransformVertex(world, primitives[i].v, current.v);
		DrawTriangle(ctx, first, previous, current, color);
		previous = current;
	}
}

//
// Draw the scene
//
void DrawScene(world_t *world, RETRO_Camera *camera, unsigned char *framebuffer, double deltaTime)
{
	world->framebuffer = framebuffer;
	ClearBuffers(world);

	// Setup the CPU-side view transform used by DrawSurface.
	UpdateView(world, camera);

	// Find the leaf the camera is in
	dleaf_t *leaf = FindLeaf(world, camera);
	(void)leaf;

	// Setup the drawing context
	drawcontext_t ctx = {
		world->framebuffer,
		world->depthbuffer,
		world->framebufferWidth,
		world->framebufferHeight
	};

	// Render the scene
	for (int i = 0; i < world->map->getNumSurfaces(); i++) {
		DrawSurface(world, i, &ctx);
	}
}

//
// Calculate primitive surfaces
//
bool DecodeSurfaces(world_t *world)
{
	// Calculate max number of edges per surface
	world->numMaxEdgesPerSurface = 0;
	for (int i = 0; i < world->map->getNumSurfaces(); i++) {
		if (world->numMaxEdgesPerSurface < world->map->getNumEdges(i)) {
			world->numMaxEdgesPerSurface = world->map->getNumEdges(i);
		}
	}

	// Allocate memory for the surface primitive array
	world->surfacePrimitives = new primdesc_t [world->map->getNumSurfaces() * world->numMaxEdgesPerSurface];

	// Loop through all surfaces to fetch their vertices
	for (int i = 0; i < world->map->getNumSurfaces(); i++) {
		int numEdges = world->map->getNumEdges(i);
		dface_t *surface = world->map->getSurface(i);

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
