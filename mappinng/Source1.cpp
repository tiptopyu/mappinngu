#define _USE_MATH_DEFINES

#include <stdio.h>
//#include <unistd.h>
#include <math.h>
#include <ypspur.h>

#ifdef __WIN32
#	include <windows.h>
#endif

#define TREAD 590
#define PI 3.14159265359
#define TIRE 290

void main(void){

	char key = 0x20;

	double vel = 0.2;

	double ang = 90;

	double x, y, theta;

	// Windows���ŕW���o�͂��o�b�t�@�����O����Ȃ��悤�ɐݒ�
	setvbuf(stdout, 0, _IONBF, 0);

	// ������
	if (Spur_init() < 0)
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return;
	}

		Spur_get_pos_GL(&x, &y, &theta);

}