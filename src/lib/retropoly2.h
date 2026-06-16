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

// One mip level of a CPU-side texture (8-bit palette indices)
struct miplevel_t
{
	int width;
	int height;
	unsigned char *pixels;
};

// A BSP texture decoded for software sampling: mip levels plus type/animation info
struct texture_t
{
	int numLevels;
	miplevel_t *levels;
	bool isSky;				// rendered through the sky dome
	bool isTurbulent;		// liquid: sampled with a rippling sine warp
	int animTotal;			// frames in the "+0".."+N" animation (0/1 = not animated)
	int animFrames[10];		// texture index of each animation frame
};

// A surface's lightmap: luxel dimensions, the light styles affecting it, and one
// block of samples per style
struct lightmap_t
{
	int width;
	int height;
	int numStyles;								// number of active light styles
	unsigned char styles[MAXLIGHTMAPS];			// light-style index of each block
	unsigned char *samples[MAXLIGHTMAPS];		// samples per style (into the BSP lighting lump)
};

// A vertex passing through clipping: clip-space position plus texture/lightmap coords
struct rendervertex_t
{
	float v[4];	// clip-space position (x, y, z, w)
	float t[2];	// texture coordinate
	float l[2];	// lightmap (luxel) coordinate
};

// A triangle vertex projected to screen space. Texture and lightmap coordinates,
// plus 1/w, are stored divided by w so they interpolate linearly in screen space.
struct screenvertex_t
{
	float x;
	float y;
	float invW;
	float uOverW;
	float vOverW;
	float lightSOverW;
	float lightTOverW;
};

// A vertex projected to screen space specifically for depth interpolation
struct screenskyvertex_t
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
	const unsigned char *lightTable;
	const int *lightStyles;
	double time;
	float skyStart[3];
	float skyDx[3];
	float skyDy[3];
};

//
// Sample the two-layer scrolling sky along the world-space view ray of one pixel
//
inline unsigned char SampleSky(const drawcontext_t *ctx, texture_t *texture, int x, int y)
{
	if (!texture || !texture->levels || texture->numLevels <= 0) {
		return 0;
	}

	miplevel_t *mip = &texture->levels[0];
	if (!mip->pixels || mip->width <= 0 || mip->height <= 0) {
		return 0;
	}

	// Reconstruct the world-space ray through this pixel using the precalculated
	// linear combinations of view basis vectors and frustum ratio.
	float dir[3];
	for (int i = 0; i < 3; i++) {
		dir[i] = ctx->skyStart[i] + (float)x * ctx->skyDx[i] + (float)y * ctx->skyDy[i];
	}

	// Project the ray onto the Quake sky dome: flatten Z, then scale so the
	// horizontal direction wraps across one 128-texel sky layer.
	dir[2] *= 3.0f;
	float length = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
	if (length < 0.0001f) {
		length = 0.0001f;
	}
	float scale = (6.0f * 63.0f) / length;

	int skyWidth = mip->width >= 256 ? mip->width / 2 : mip->width;
	int skyOffset = skyWidth * 2 <= mip->width ? skyWidth : 0;

	// The solid back layer scrolls slowly; the cloud layer scrolls twice as fast.
	float backScroll = (float)(ctx->time * 8.0);
	float frontScroll = (float)(ctx->time * 16.0);

	int backX = WrapTexel((int)floorf(backScroll + dir[0] * scale), skyWidth);
	int backY = WrapTexel((int)floorf(backScroll + dir[1] * scale), mip->height);
	unsigned char color = mip->pixels[(backX + skyOffset) + backY * mip->width];

	if (skyWidth * 2 <= mip->width) {
		int cloudX = WrapTexel((int)floorf(frontScroll + dir[0] * scale), skyWidth);
		int cloudY = WrapTexel((int)floorf(frontScroll + dir[1] * scale), mip->height);
		unsigned char cloud = mip->pixels[cloudX + cloudY * mip->width];
		if (cloud != 0) {
			color = cloud;
		}
	}
	return color;
}

//
// Sample one indexed texel from a BSP mip level
//
inline unsigned char SampleTexture(const drawcontext_t *ctx, texture_t *texture, float u, float v, int level)
{
	if (!texture || !texture->levels || texture->numLevels <= 0) {
		return 0;
	}
	if (level < 0) {
		level = 0;
	} else if (level >= texture->numLevels) {
		level = texture->numLevels - 1;
	}

	miplevel_t *mip = &texture->levels[level];
	if (!mip->pixels || mip->width <= 0 || mip->height <= 0) {
		return 0;
	}

	float s, t;
	// Liquids ripple their texels with a time-varying sine warp.
	if (texture->isTurbulent) {
		float time = (float)ctx->time;
		// Compute warp in base texture dimensions (mip level 0) so it scales consistently
		float baseW = (float)texture->levels[0].width;
		float baseH = (float)texture->levels[0].height;
		float s0 = u * baseW;
		float t0 = v * baseH;

		float warpS = sinf((t0 + time * 96.0f) * 0.125f) * 4.0f;
		float warpT = sinf((s0 + time * 80.0f) * 0.125f) * 4.0f;

		s = (s0 + warpS) * ((float)mip->width / baseW);
		t = (t0 + warpT) * ((float)mip->height / baseH);
	} else {
		s = u * (float)mip->width;
		t = v * (float)mip->height;
	}

	int x = WrapTexel((int)floorf(s), mip->width);
	int y = WrapTexel((int)floorf(t), mip->height);
	return mip->pixels[x + y * mip->width];
}

//
// Sample the nearest surface lightmap texel
//
inline unsigned char SampleLightmap(const drawcontext_t *ctx, lightmap_t *lightmap, float s, float t)
{
	if (!lightmap || lightmap->numStyles <= 0 || lightmap->width <= 0 || lightmap->height <= 0) {
		return 255;
	}

	if (s < 0.0f) s = 0.0f;
	if (t < 0.0f) t = 0.0f;
	if (s > (float)(lightmap->width - 1)) s = (float)(lightmap->width - 1);
	if (t > (float)(lightmap->height - 1)) t = (float)(lightmap->height - 1);

	// Round to the nearest luxel.
	int sampleS = (int)floorf(s + 0.5f);
	int sampleT = (int)floorf(t + 0.5f);
	if (sampleS >= lightmap->width) sampleS = lightmap->width - 1;
	if (sampleT >= lightmap->height) sampleT = lightmap->height - 1;

	// Sum every light style affecting this surface, each scaled by its current
	// animated brightness (264 == normal full strength).
	float light = 0.0f;
	for (int i = 0; i < lightmap->numStyles; i++) {
		unsigned char *samples = lightmap->samples[i];
		if (!samples) {
			continue;
		}

		float sample = (float)samples[sampleS + sampleT * lightmap->width];
		int style = lightmap->styles[i];
		int styleValue = (style < 64) ? ctx->lightStyles[style] : 264;
		light += sample * ((float)styleValue / 264.0f);
	}

	int lightValue = (int)(light + 0.5f);
	if (lightValue < 0) lightValue = 0;
	if (lightValue > 255) lightValue = 255;
	return (unsigned char)lightValue;
}

//
// Rasterize a clipped triangle into the software color and depth buffers
//
inline void RasterizeTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, texture_t *texture, lightmap_t *lightmap, int mipLevel, unsigned char fallbackColor)
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
		v[i].uOverW = clip[i]->t[0] * invW;
		v[i].vOverW = clip[i]->t[1] * invW;
		v[i].lightSOverW = clip[i]->l[0] * invW;
		v[i].lightTOverW = clip[i]->l[1] * invW;
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
				unsigned char color = fallbackColor;
				if (invW_unscaled != 0.0f && texture) {
					// Recover perspective-correct coords directly using unscaled values to avoid dividing by area.
					float uOverW_unscaled = w0 * v[0].uOverW + w1 * v[1].uOverW + w2 * v[2].uOverW;
					float vOverW_unscaled = w0 * v[0].vOverW + w1 * v[1].vOverW + w2 * v[2].vOverW;
					color = SampleTexture(ctx, texture, uOverW_unscaled / invW_unscaled, vOverW_unscaled / invW_unscaled, mipLevel);
					if (lightmap) {
						float lightS_unscaled = w0 * v[0].lightSOverW + w1 * v[1].lightSOverW + w2 * v[2].lightSOverW;
						float lightT_unscaled = w0 * v[0].lightTOverW + w1 * v[1].lightTOverW + w2 * v[2].lightTOverW;
						unsigned char light = SampleLightmap(ctx, lightmap, lightS_unscaled / invW_unscaled, lightT_unscaled / invW_unscaled);
						color = ctx->lightTable[color * 256 + light];
					}
				}
				ctx->depthbuffer[index] = invW;
				ctx->framebuffer[index] = color;
			}
		}
	}
}

//
// Rasterize a sky triangle (using only screen coordinates and depth testing)
//
inline void RasterizeSkyTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, texture_t *texture)
{
	const float wEpsilon = 0.0001f;
	const float areaEpsilon = 0.0001f;
	screenskyvertex_t v[3];
	const rendervertex_t *clip[3] = { &a, &b, &c };

	// Perspective-divide each vertex to NDC, map to screen, and store depth.
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

			float invW = (w0 * v[0].invW + w1 * v[1].invW + w2 * v[2].invW) / area;
			int index = x + y * ctx->framebufferWidth;
			// w-buffer: a larger 1/w is nearer. Precision is distributed far more
			// uniformly than NDC-z, which removes distant coplanar z-fighting.
			if (invW > ctx->depthbuffer[index]) {
				unsigned char color = 0;
				if (invW != 0.0f) {
					color = SampleSky(ctx, texture, x, y);
				}
				ctx->depthbuffer[index] = invW;
				ctx->framebuffer[index] = color;
			}
		}
	}
}

//
// Clip a triangle against the near plane and draw the resulting triangle fan
//
inline void DrawTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, texture_t *texture, lightmap_t *lightmap, int mipLevel, unsigned char fallbackColor)
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
			for (int j = 0; j < 2; j++) {
				output[outputCount].t[j] = current->t[j] + t * (next->t[j] - current->t[j]);
				output[outputCount].l[j] = current->l[j] + t * (next->l[j] - current->l[j]);
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
			for (int j = 0; j < 2; j++) {
				output[outputCount].t[j] = current->t[j] + t * (next->t[j] - current->t[j]);
				output[outputCount].l[j] = current->l[j] + t * (next->l[j] - current->l[j]);
			}
			outputCount++;
			output[outputCount++] = *next;
		}
	}

	// Fan-triangulate the clipped polygon.
	for (int i = 1; i < outputCount - 1; i++) {
		RasterizeTriangle(ctx, output[0], output[i], output[i + 1], texture, lightmap, mipLevel, fallbackColor);
	}
}

//
// Clip a sky triangle against the near plane and draw the resulting triangle fan
//
inline void DrawSkyTriangle(const drawcontext_t *ctx, const rendervertex_t &a, const rendervertex_t &b, const rendervertex_t &c, texture_t *texture)
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
			for (int j = 0; j < 2; j++) {
				output[outputCount].t[j] = current->t[j] + t * (next->t[j] - current->t[j]);
				output[outputCount].l[j] = current->l[j] + t * (next->l[j] - current->l[j]);
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
			for (int j = 0; j < 2; j++) {
				output[outputCount].t[j] = current->t[j] + t * (next->t[j] - current->t[j]);
				output[outputCount].l[j] = current->l[j] + t * (next->l[j] - current->l[j]);
			}
			outputCount++;
			output[outputCount++] = *next;
		}
	}

	// Fan-triangulate the clipped polygon.
	for (int i = 1; i < outputCount - 1; i++) {
		RasterizeSkyTriangle(ctx, output[0], output[i], output[i + 1], texture);
	}
}

#endif
