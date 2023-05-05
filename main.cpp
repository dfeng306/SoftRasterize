#include <iostream>
#include <graphics.h>
#include <ctime>
#include "glib.h"

#define PI 3.1415926
#define Deg2Rad  PI/180

using namespace std;

struct Input {
	bool holding = false;
	short startX;
	short startY;
};

const int WIN_WIDTH = 512;
const int WIN_HEIGHT = 512;

vec3 light_dir{ 1,1,-1 }; // light source
vec3       eye{ 0,100,-100 }; // camera position
vec3    center{ 0,0,0 }; // camera direction
vec3        up{ 0,1,0 }; // camera up vector

const float fov = 60;

const float rotate_speed = 60;
///-------input------------
ExMessage msg;
Input input;

vec3 eye_pre_pos, center_pre_pos;
bool cam_move_flag = false;
//--------buffer-----------
float* zBuffer;

mat<4, 4> Model;
mat<4, 4> View;
mat<4, 4> Projection;
mat<4, 4> VP;
mat<4, 4> Viewport;

vec3 vertics[] = { {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1} };
int vIndex[] = {0,1,2,0,2,3,0,7,4,0,3,7,3,2,6,3,6,7,1,5,6,1,6,2,0,5,1,0,4,5,5,4,6,4,7,6};
vec3 offset = {0,0,0};
vec3 scale = { 25,25,25 };

mat<4, 4> getRotateMatrixY(const float ang) {
	float _cos = cos(ang * Deg2Rad);
	float _sin = sin(ang * Deg2Rad);
	return { {{_cos,0,_sin,0},{0,1,0,0},{-_sin,0,_cos,0},{0,0,0,1}} };
}

mat<4, 4> getRotateMatrixX(const float ang) {
	float _cos = cos(ang * Deg2Rad);
	float _sin = sin(ang * Deg2Rad);
	return { {{1,0,0,0},{0,_cos,-_sin,0},{0,_sin,_cos,0},{0,0,0,1}} };
}

void UpdateInput() {
	peekmessage(&msg);
	if (msg.lbutton) {
		if (!input.holding) {
			input.startX = msg.x;
			input.startY = msg.y;
			input.holding = true;
		}
	}
	else if (msg.mbutton) {
		if (!input.holding) {
			input.startX = msg.x;
			input.startY = msg.y;
			input.holding = true;
		}
	}
	else if (msg.rbutton) {
		if (!input.holding) {
			input.startX = msg.x;
			input.startY = msg.y;
			input.holding = true;
		}
	}
	else {
		if (input.holding) {
			input.startX = 0;
			input.startY = 0;
			input.holding = false;
		}
	}
}

void UpdateCamera() {
	if (input.holding) {
		if (!cam_move_flag) {
			cam_move_flag = true;
			eye_pre_pos = eye;
			center_pre_pos = center;
		}
		vec2 dir = { msg.x - input.startX,msg.y - input.startY };
		vec3 forward = center - eye;
		vec3 right = cross(forward, up);
		up = cross(right, forward);
		forward.normalize();
		right.normalize();
		up.normalize();

		if (msg.lbutton) {
			eye = eye_pre_pos - forward * dir.y;
			//center = center_pre_pos - forward * dir.y;
		}
		else if (msg.mbutton) {
			eye = eye_pre_pos + right * dir.x + up * dir.y;
			center = center_pre_pos + right * dir.x + up * dir.y;
		}
		else if (msg.rbutton) {
			vec4 fwd = getRotateMatrixY(dir.x / (WIN_WIDTH * GRID_SIZE) * 180) * embed<4>((eye_pre_pos-center_pre_pos));
			fwd = getRotateMatrixX(dir.y / (WIN_HEIGHT * GRID_SIZE) * 180) * fwd;
			eye = center_pre_pos + vec3{ fwd[0],fwd[1],fwd[2] };
			up = vec3{ 0,1,0 };
		}
		View = lookat(eye, center, up);
	}
	else
	{
		cam_move_flag = false;
	}
}

int main() {
	TCHAR  buf[128];
	double time = 0;
	long frameCount = 0;
	long delta = 0;
	int fps = 0;
	clock_t t = clock();
	light_dir.normalize();
	initgraph(WIN_WIDTH, WIN_HEIGHT);
	
	zBuffer = new float[WIN_WIDTH * WIN_HEIGHT];
	for (int i = 0; i < WIN_WIDTH * WIN_HEIGHT; i++)
		zBuffer[i] = -1000;

	Viewport = viewport(0, 0, WIN_WIDTH, WIN_HEIGHT);
	float n = -1;
	float f = -200;
	float h = tan(fov / 2.0f * Deg2Rad) * abs(n);
	mat<4, 4> othroMatrix = othroProjection(-h, h, h, -h, n, f);
	mat<4, 4> perspective2Orthographic = { {{n,0,0,0},{0,n,0,0},{0,0,n + f,-n * f},{0,0,1,0}} };
	Projection = othroMatrix * perspective2Orthographic;

	mat<4, 4> transMatrix = { {{scale.x,0,0,offset.x},{0,scale.y,0,offset.y},{0,0,scale.z,offset.z},{0,0,0,1}} };

	while (true)
	{
		cleardevice();
		BeginBatchDraw();
		delta = t;
		t = clock();
		delta = t - delta;
		time = t / (double)CLOCKS_PER_SEC;
		swprintf_s(buf, _T("time:%0.2f"), time);
		outtextxy(0, 0, buf);
		if (frameCount % 50 == 0) {
			if (delta > 0)
				fps = int(1 / (delta / (float)CLOCKS_PER_SEC));
			else
				fps = 1000;
		}
		swprintf_s(buf, _T("fps:%d     delta:%ld"), fps, delta);
		outtextxy(0, 20, buf);

		frameCount++;

		//UpdateInput();
		//UpdateCamera();

		mat<4, 4> rotateMatrix2 = getRotateMatrixY(time * rotate_speed);

		View = lookat(eye, center, up);
		Model = transMatrix * rotateMatrix2;
		VP = (Projection * View);

		for (int i = 0; i < 12; i++) {
			int index = 3 * i;
			vec4 p1 = Model * (embed<4>(vertics[vIndex[index]]));
			vec4 p2 = Model * (embed<4>(vertics[vIndex[index + 1]]));
			vec4 p3 = Model * (embed<4>(vertics[vIndex[index + 2]]));

			vec3 normal = cross(proj<3>(p3) - proj<3>(p1), proj<3>(p2) - proj<3>(p1)).normalized();

			float f = light_dir * normal;
			int c = int(255 * f);
			COLORREF col = RGB(c, c, c);

			p1 = VP * p1;
			p2 = VP * p2;
			p3 = VP * p3;

			vec4 verts[3] = { p1,p2,p3 };

			if (true) {
				Triangle(verts, WIN_WIDTH, WIN_HEIGHT, Viewport, zBuffer);
			}
			else {
				p1 = Viewport * p1;
				p2 = Viewport * p2;
				p3 = Viewport * p3;
				p1 = p1 / p1[3];
				p2 = p2 / p2[3];
				p3 = p3 / p3[3];

				drawline_WuXiaoLin(int(p1[0]), WIN_HEIGHT - int(p1[1]), int(p2[0]), WIN_HEIGHT - int(p2[1]));
				drawline_WuXiaoLin(int(p2[0]), WIN_HEIGHT - int(p2[1]), int(p3[0]), WIN_HEIGHT - int(p3[1]));
				drawline_WuXiaoLin(int(p3[0]), WIN_HEIGHT - int(p3[1]), int(p1[0]), WIN_HEIGHT - int(p1[1]));
			}
		}

		//peekmessage(&msg);
		//swprintf_s(buf, _T("fps:%f"), zBuffer[msg.x + msg.y * WIN_HEIGHT]);
		//outtextxy(msg.x + 10, msg.y + 10, buf);
		//flushmessage();

		FlushBatchDraw();
		EndBatchDraw();
		//Sleep(10);
	}
	closegraph();
}
