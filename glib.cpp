#include <graphics.h>
#include "glib.h"


void swap(int* a, int* b) {
	int temp = *a;
	*a = *b;
	*b = temp;
}

mat<4, 4> getLocal2WorldMatrix(const vec3 origin,const vec3 forward,const vec3 up) {
	vec3 _forward = forward;
	_forward.normalize();
	vec3 _right = cross(_forward, up);
	_right.normalize();
	vec3 _up = cross(_right, _forward);
	_up.normalize();

	mat<4, 4> mat = { {
		{_right.x,_up.x,_forward.x,origin.x},
		{_right.y,_up.y,_forward.y,origin.y},
		{_right.z,_up.z,_forward.z,origin.z},
		{0,0,0,1}
	} };
	return mat;
}

mat<4, 4> viewport(const int x, const int y, const int w, const int h) {
	return  { {{w / 2., 0, 0, x + w / 2.}, {0, h / 2., 0, y + h / 2.}, {0,0,1,0}, {0,0,0,1}} };
}

mat<4, 4> projection(const double f) { // check https://en.wikipedia.org/wiki/Camera_matrix
	return { {{1,0,0,0}, {0,-1,0,0}, {0,0,1,0}, {0,0,-1 / f,0}} };
}

mat<4, 4> othroProjection(const float l, const float r, const float t, const float b, const float n, const float f) {
	return { {{2 / (r - l),0,0,-(r + l) / (r - l)},{0,2 / (t - b),0,-(t + b) / (t - b)},{0,0,2 / (f - n),-(f + n) / (f - n)},{0,0,0,1}} };
}

mat<4, 4> perspectiveProjection(const float l, const float r, const float t, const float b, const float n, const float f) {

	return { {{2 * n / (r - l),0,(r + l) / (r - l),0},{0,2 * n / (t - b),(t + b) / (t - b),0},{0,0,-(f + n) / (f - n),-2 * n * f / (f - n)},{0,0,-1,0}} };
}

mat<4, 4> lookat(const vec3 eye, const vec3 center, const vec3 up) { // check https://github.com/ssloy/tinyrenderer/wiki/Lesson-5-Moving-the-camera
	vec3 z = (eye-center).normalized();
	vec3 x = cross(z, up).normalized();
	vec3 y = cross(x, z).normalized();
	mat<4, 4> Minv = { {{x.x,x.y,x.z,0},   {y.x,y.y,y.z,0},   {z.x,z.y,z.z,0},   {0,0,0,1}} };
	mat<4, 4> Tr = { {{1,0,0,-eye.x}, {0,1,0,-eye.y}, {0,0,1,-eye.z}, {0,0,0,1}} };
	return Minv * Tr;
}


void SetPixel(int x, int y, COLORREF col) {
	int xMin = x * GRID_SIZE;
	int yMin = y * GRID_SIZE;
	int xMax = xMin + GRID_SIZE;
	int yMax = yMin + GRID_SIZE;
	for (int i = xMin; i <= xMax; i++)
		for (int j = yMin; j <= yMax; j++)
			putpixel(i, j, col);
}

void drawline_bresenham(int x0, int y0, int x1, int y1) {
	bool steep = false;
	if (abs(x0 - x1) < abs(y0 - y1)) {
		swap(&x0, &y0);
		swap(&x1, &y1);
		steep = true;
	}
	if (x0 > x1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}
	int dx = x1 - x0;
	int dy = y1 - y0;
	int derror2 = abs(dy) * 2;
	int error2 = 0;
	int y = y0;
	for (int x = x0; x <= x1; x++) {
		if (steep)
			putpixel(y, x, WHITE);
		else
			putpixel(x, y, WHITE);

		error2 += derror2;
		if (error2 > dx) {
			y += (y1 > y0 ? 1 : -1);
			error2 -= dx * 2;
		}
	}
}

void drawline_WuXiaoLin(int x0, int y0, int x1, int y1) {
	bool steep = false;
	if (abs(x0 - x1) < abs(y0 - y1)) {
		swap(&x0, &y0);
		swap(&x1, &y1);
		steep = true;
	}
	if (x0 > x1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}

	int dx = x1 - x0;
	int dy = y1 - y0;
	double k = dx == 0 ? 1 : (double)dy / dx;
	double e = y0, f = 0;

	if (steep) {
		for (int x = x0; x < x1; x++) {
			f = 1 - (e - (int)e);
			putpixel(int(e), x, RGB(255 * f, 255 * f, 255 * f));
			putpixel(int(e) + 1, x, RGB(255 * (1 - f), 255 * (1 - f), 255 * (1 - f)));
			e += k;
		}
	}
	else {
		for (int x = x0; x < x1; x++) {
			f = 1 - (e - (int)e);
			putpixel(x, int(e), RGB(255 * f, 255 * f, 255 * f));
			putpixel(x, int(e) + 1, RGB(255 * (1 - f), 255 * (1 - f), 255 * (1 - f)));
			e += k;
		}
	}
}

void drawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, COLORREF col) {
	if (y0 > y1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}
	if (y0 > y2) {
		swap(&x0, &x2);
		swap(&y0, &y2);
	}
	if (y1 > y2) {
		swap(&x1, &x2);
		swap(&y1, &y2);
	}
	int totalH = y2 - y0;
	int segmentH = y1 - y0;
	if (segmentH > 0) {
		for (int y = y0; y <= y1; y++) {
			float alpha = (float)(y - y0) / totalH;
			float beta = (float)(y - y0) / segmentH;
			int xa = (int)(x0 + (x2 - x0) * alpha);
			int xb = (int)(x0 + (x1 - x0) * beta);
			if (xa > xb)
				swap(&xa, &xb);
			for (int j = xa; j <= xb; j++)
				putpixel(j, y, col);
		}
	}
	segmentH = y2 - y1;
	if (segmentH > 0) {
		for (int y = y1; y <= y2; y++) {
			float alpha = (float)(y - y0) / totalH;
			float beta = (float)(y - y1) / segmentH;
			int xa = (int)(x0 + (x2 - x0) * alpha);
			int xb = (int)(x1 + (x2 - x1) * beta);
			if (xa > xb)
				swap(&xa, &xb);
			for (int j = xa; j <= xb; j++)
				putpixel(j, y, col);
		}
	}
}

vec3 barycentric(const vec2 v[3], const vec2 p) {
	float i = (-(p.x - v[2].x) * (v[0].y - v[2].y) + (p.y - v[2].y) * (v[0].x - v[2].x)) / (-(v[1].x - v[2].x) * (v[0].y - v[2].y) + (v[1].y - v[2].y) * (v[0].x - v[2].x));
	float j = (-(p.x - v[1].x) * (v[2].y - v[1].y) + (p.y - v[1].y) * (v[2].x - v[1].x)) / (-(v[0].x - v[1].x) * (v[2].y - v[1].y) + (v[0].y - v[1].y) * (v[2].x - v[1].x));
	return vec3{ i,j,1 - i - j };
}

void Triangle(const vec4 clipVerts[3],const int W,const int H,const mat<4,4> Viewport,float* zBuffer) {
	vec4 vVerts[3] = { Viewport * clipVerts[0], Viewport * clipVerts[1] , Viewport * clipVerts[2] };
	vec2 vVerts2[3] = {proj<2>(vVerts[0]/vVerts[0][3]),proj<2>(vVerts[1] / vVerts[1][3]) ,proj<2>(vVerts[2] / vVerts[2][3]) };
	int minX = W - 1;
	int minY = H - 1;
	int maxX = 0;
	int maxY = 0;
	for (int i = 0; i < 3; i++) {
		if (vVerts2[i].x < minX)
			minX = vVerts2[i].x;
		if (vVerts2[i].x > maxX)
			maxX = vVerts2[i].x;
		if (vVerts2[i].y < minY)
			minY = vVerts2[i].y;
		if (vVerts2[i].y > maxY)
			maxY = vVerts2[i].y;
	}
	minX = max(minX, 0);
	minY = max(minY, 0);
	maxX = min(W - 1, maxX);
	maxY = min(H - 1, maxY);

	for(int i=minX;i<=maxX;i++)
		for (int j = minY; j <= maxY; j++) {
			vec3 bc = barycentric(vVerts2, { float(i),float(j) });
			if (bc.x < 0 || bc.y < 0 || bc.z < 0)
				continue;
			float dp = bc.x * vVerts[0][2] / vVerts[0][3] + bc.y * vVerts[1][2] / vVerts[1][3] + bc.z * vVerts[2][2] / vVerts[2][3];
			zBuffer[i + j * W] = dp;
			putpixel(i, H-j, RGB(255 * dp, 255 * dp, 255 * dp));
			//if (dp < zBuffer[i + j * W]) {
			//	zBuffer[i + j * W] = dp;
			//	putpixel(i, j, WHITE);
			//}
		}
}