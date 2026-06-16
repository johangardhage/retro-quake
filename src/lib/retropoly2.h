//
// Retro graphics library
//
// Author: Johan Gardhage <johan.gardhage@gmail.com>
//

#ifndef _RETROPOLY2_H_
#define _RETROPOLY2_H_

#include <math.h>
#include "retrobsp.h"
#include "retromath.h"

// A vertex passing through clipping: clip-space position
struct rendervertex_t
{
	float v[4];	// clip-space position (x, y, z, w)
};

// A triangle vertex projected to screen space. 1/w is stored so it interpolates linearly.
struct screenvertex_t
{
	float x;
	float y;
	float invW;
};

// Context structure decoupling the rasterizer from the engine's main world struct
struct drawcontext_t
{
	unsigned char *framebuffer;
	float *depthbuffer;
	int framebufferWidth;
	int framebufferHeight;
};

//
// Rasterize a clipped triangle into the software color and depth buffers
//
inline void RasterizeTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, unsigned char fallbackColor)
{
	const float wEpsilon = 0.0001f;
	const float areaEpsilon = 0.0001f;
	screenvertex_t v[3];
	const rendervertex_t *clip[3] = { &a, &b, &c };

	// Perspective-divide each vertex to NDC, map to screen, and store attributes
	// pre-divided by w so they interpolate linearly across the triangle.
	for (int i = 0; i < 3; i++) {
		if (clip[i]->v[3] <= wEpsilon) {
			return;
		}
		float invW = 1.0f / clip[i]->v[3];
		float ndcX = clip[i]->v[0] * invW;
		float ndcY = clip[i]->v[1] * invW;

		v[i].x = (ndcX * 0.5f + 0.5f) * (float)(ctx->framebufferWidth - 1);
		v[i].y = (0.5f - ndcY * 0.5f) * (float)(ctx->framebufferHeight - 1);
		v[i].invW = invW;
	}

	// Twice the signed screen area; near-zero means a degenerate or edge-on triangle.
	float area = EdgeFunction(v[0].x, v[0].y, v[1].x, v[1].y, v[2].x, v[2].y);
	if (area > -areaEpsilon && area < areaEpsilon) {
		return;
	}

	// Rasterize within the triangle's screen bounding box, clamped to the viewport.
	float minX = fminf(v[0].x, fminf(v[1].x, v[2].x));
	float maxX = fmaxf(v[0].x, fmaxf(v[1].x, v[2].x));
	float minY = fminf(v[0].y, fminf(v[1].y, v[2].y));
	float maxY = fmaxf(v[0].y, fmaxf(v[1].y, v[2].y));

	int x0 = (int)floorf(minX);
	int x1 = (int)ceilf(maxX);
	int y0 = (int)floorf(minY);
	int y1 = (int)ceilf(maxY);

	if (x0 < 0) x0 = 0;
	if (y0 < 0) y0 = 0;
	if (x1 >= ctx->framebufferWidth) x1 = ctx->framebufferWidth - 1;
	if (y1 >= ctx->framebufferHeight) y1 = ctx->framebufferHeight - 1;
	if (x0 > x1 || y0 > y1) {
		return;
	}

	for (int y = y0; y <= y1; y++) {
		for (int x = x0; x <= x1; x++) {
			float pixelX = (float)x + 0.5f;
			float pixelY = (float)y + 0.5f;
			// Barycentric weights; the pixel is covered when all share the area's sign.
			float w0 = EdgeFunction(v[1].x, v[1].y, v[2].x, v[2].y, pixelX, pixelY);
			float w1 = EdgeFunction(v[2].x, v[2].y, v[0].x, v[0].y, pixelX, pixelY);
			float w2 = EdgeFunction(v[0].x, v[0].y, v[1].x, v[1].y, pixelX, pixelY);

			if ((area > 0.0f && (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f))
					|| (area < 0.0f && (w0 > 0.0f || w1 > 0.0f || w2 > 0.0f))) {
				continue;
			}

			float invW_unscaled = w0 * v[0].invW + w1 * v[1].invW + w2 * v[2].invW;
			float invW = invW_unscaled / area;
			int index = x + y * ctx->framebufferWidth;
			// w-buffer: a larger 1/w is nearer. Precision is distributed far more
			// uniformly than NDC-z, which removes distant coplanar z-fighting.
			if (invW > ctx->depthbuffer[index]) {
				ctx->depthbuffer[index] = invW;
				ctx->framebuffer[index] = fallbackColor;
			}
		}
	}
}

//
// Clip a triangle against the near plane and draw the resulting triangle fan
//
inline void DrawTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, unsigned char fallbackColor)
{
	const float nearPlane = 1.0f;
	const float clipEpsilon = 0.0001f;
	rendervertex_t input[3] = { a, b, c };
	rendervertex_t output[4];	// clipping a triangle against one plane yields at most 4 vertices
	int outputCount = 0;

	// Sutherland-Hodgman clip of the triangle against the near plane.
	for (int i = 0; i < 3; i++) {
		rendervertex_t *current = &input[i];
		rendervertex_t *next = &input[(i + 1) % 3];
		bool currentInside = current->v[3] >= nearPlane + clipEpsilon;
		bool nextInside = next->v[3] >= nearPlane + clipEpsilon;

		if (currentInside && nextInside) {
			output[outputCount++] = *next;
		} else if (currentInside && !nextInside) {
			float denominator = next->v[3] - current->v[3];
			if (denominator > -clipEpsilon && denominator < clipEpsilon) {
				continue;
			}
			float t = (nearPlane - current->v[3]) / denominator;
			for (int j = 0; j < 4; j++) {
				output[outputCount].v[j] = current->v[j] + t * (next->v[j] - current->v[j]);
			}
			outputCount++;
		} else if (!currentInside && nextInside) {
			float denominator = next->v[3] - current->v[3];
			if (denominator > -clipEpsilon && denominator < clipEpsilon) {
				output[outputCount++] = *next;
				continue;
			}
			float t = (nearPlane - current->v[3]) / denominator;
			for (int j = 0; j < 4; j++) {
				output[outputCount].v[j] = current->v[j] + t * (next->v[j] - current->v[j]);
			}
			outputCount++;
			output[outputCount++] = *next;
		}
	}

	// Fan-triangulate the clipped polygon.
	for (int i = 1; i < outputCount - 1; i++) {
		RasterizeTriangle(ctx, output[0], output[i], output[i + 1], fallbackColor);
	}
}

#endif
