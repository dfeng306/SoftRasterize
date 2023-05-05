#pragma once
#include "geometry.h"

const int GRID_SIZE = 2;

void swap(int*, int*);
mat<4, 4> getLocal2WorldMatrix(const vec3, const vec3, const vec3);
mat<4, 4> viewport(const int x, const int y, const int w, const int h);
mat<4, 4> projection(const double coeff = 0); // coeff = -1/c
mat<4, 4> lookat(const vec3 eye, const vec3 center, const vec3 up);
mat<4, 4> othroProjection(const float l, const float r, const float t, const float b, const float n, const float f);
mat<4, 4> perspectiveProjection(const float l, const float r, const float t, const float b, const float n, const float f);


void SetPixel(int x, int y, COLORREF col);
void drawline_bresenham(int, int, int, int);
void drawline_WuXiaoLin(int, int, int, int); 
void drawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, COLORREF col);
vec3 barycentric(const vec2 tri[3], const vec2 p); 
void Triangle(const vec4 clipVerts[3], const int W, const int H, const mat<4, 4> Viewport, float* zBuffer);