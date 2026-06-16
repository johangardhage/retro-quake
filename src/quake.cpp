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
#include "lib/retromath.h"
#include "lib/retrocamera.h"

#define MOVEMENT_SPEED		8.0

// Simplified renderer world context
struct world_t
{
	RETRO_BSP *map = NULL;
	int framebufferWidth = 0;
	int framebufferHeight = 0;
	float frustumRatio = 1.0f;
	vec3_t viewOrigin;
	vec3_t viewForward;
	vec3_t viewSide;
	vec3_t viewUp;
};

world_t world;
RETRO_BSP map;
RETRO_Camera camera;

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
	world.frustumRatio = (float)RETRO_HEIGHT / (float)RETRO_WIDTH;
}

//
// One-time teardown: release map and world resources
//
void DEMO_Deinitialize(void)
{
	RETRO_FreeBSP(&map);
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
	UpdateView(&world, &camera);

	// Clear the framebuffer to black
	unsigned char *fb = RETRO_FrameBuffer();
	for (int i = 0; i < RETRO_WIDTH * RETRO_HEIGHT; i++) {
		fb[i] = 0;
	}

	// Project and draw BSP vertices as a point cloud
	int numVertices = world.map->getLump(LUMP_VERTEXES)->filelen / sizeof(dvertex_t);
	for (int i = 0; i < numVertices; i++) {
		vec3_t *vertex = world.map->getVertex(i);
		float clip[4];
		TransformVertex(&world, *vertex, clip);

		// Only draw vertices in front of the camera (w > 1.0)
		if (clip[3] > 1.0f) {
			float invW = 1.0f / clip[3];
			float ndcX = clip[0] * invW;
			float ndcY = clip[1] * invW;

			int x = (int)((ndcX * 0.5f + 0.5f) * (float)(RETRO_WIDTH - 1));
			int y = (int)((0.5f - ndcY * 0.5f) * (float)(RETRO_HEIGHT - 1));

			if (x >= 0 && x < RETRO_WIDTH && y >= 0 && y < RETRO_HEIGHT) {
				fb[y * RETRO_WIDTH + x] = 255;
			}
		}
	}
}
