/*
#define _USE_MATH_DEFINES

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.c"
#include "pcimage.h"

#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <Windows.h>
*/

using namespace std;

int open_urg_sensor(urg_t *urg, int argc, char *argv[]);


//�����ݒ�l
int COMID_urg = 4;
int COMID_Arduino_Encoder = 16;

float urgpos[3] = { 120.0, 0.0, 0.0 };//�Z���T�̒n�ʂ���̍����C�Z���T�̊�ʒu����̋����C����ѐ����ʂ���̘�p

float scaninterval = 0.0;//�v�������{����Œ�Ԋu[mm]

//URG�֌W�̃O���[�o���ϐ�
enum {
	CAPTURE_TIMES = 1,
};
urg_t urg;
long *data = NULL;
long time_stamp;

int urgconnect = 0;//URG�ւ̐ڑ��w��
int urgconnected = 0;//URG�ւ̐ڑ����

float chairpos_old = 0.0;//�Ԃ����̎ԗւ̒��O�̐i�s 
float chairpos = 0.0;//�Ԃ����̌��݂̐i�s 
float startpos[2] = { 0.0, 0.0 };
float startpos_old[2] = { 0.0, 0.0 };
float DIS_old = 0.0;
float movepos[2] = { 0.0, 0.0 };
float movedis = 0.0;
HANDLE hComm;
PCImage pcimage;

//�v���g�^�C�v�錾
HANDLE CommOpen(char *pport);
int CommClose(HANDLE hComm);
int Encoder(HANDLE hComm, float& dist, float& rad);


int CommClose(HANDLE hComm)
{
	if (hComm){
		CloseHandle(hComm);
	}

	return 1;
}

HANDLE CommOpen(char *pport)
{
	HANDLE			hComm;		// �ʐM�p�n���h��
	DCB				cDcb;		// �ʐM�ݒ�p
	COMMTIMEOUTS	cTimeouts;	// �ʐM�|�[�g�^�C���A�E�g�p


	// �ʐM�|�[�g���J��
	hComm = CreateFileA(pport,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);
	// �n���h���̊m�F
	if (hComm == INVALID_HANDLE_VALUE){
		hComm = NULL;
		goto FuncEnd;
	}


	// �ʐM�ݒ�
	if (!GetCommState(hComm, &cDcb)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:GetCommState error\n");
		CommClose(hComm);
		hComm = NULL;
		goto FuncEnd;
	}
	cDcb.BaudRate = 115200;				// �ʐM���x
	cDcb.ByteSize = 8;					// �f�[�^�r�b�g��
	cDcb.fParity = TRUE;				// �p���e�B�`�F�b�N
	cDcb.Parity = NOPARITY;			// �m�[�p���e�B
	cDcb.StopBits = ONESTOPBIT;			// 1�X�g�b�v�r�b�g

	if (!SetCommState(hComm, &cDcb)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:GetCommState error\n");
		CommClose(hComm);
		hComm = NULL;
		goto FuncEnd;
	}


	// �ʐM�ݒ�(�ʐM�^�C���A�E�g�ݒ�)
	// �����̓ǂݍ��ݑ҂�����(ms)
	cTimeouts.ReadIntervalTimeout = 50;
	// �ǂݍ��݂̂P����������̎���(ms)
	cTimeouts.ReadTotalTimeoutMultiplier = 50;
	// �ǂݍ��݂̒萔����(ms)
	cTimeouts.ReadTotalTimeoutConstant = 50;
	// �������݂̂P����������̎���(ms)
	cTimeouts.WriteTotalTimeoutMultiplier = 50;
	// �������݂̒萔����(ms)
	cTimeouts.WriteTotalTimeoutConstant = 50;

	if (!SetCommTimeouts(hComm, &cTimeouts)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:SetCommTimeouts error\n");
		CommClose(hComm);
		hComm = NULL;
		goto FuncEnd;
	}


	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);


FuncEnd:
	return hComm;
}


int Encoder(HANDLE hComm, float& dist, float& rad)
{
	unsigned char	sendbuf[1];
	unsigned char	receive_data[2];
	int				ret;
	float			DL, DR, DIS, ANG;
	unsigned long	len;

	// �n���h���`�F�b�N
	/*if( !hComm ){
	return -1;
	}*/


	// �o�b�t�@�N���A
	memset(sendbuf, 0x00, sizeof(sendbuf));

	// �p�P�b�g�쐬
	sendbuf[0] = (unsigned char)1;				// �w�b�_�[1

	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);

	// ���M
	ret = WriteFile(hComm, &sendbuf, 1, &len, NULL);

	memset(receive_data, 0x00, sizeof(receive_data));

	PurgeComm(hComm, PURGE_RXCLEAR);

	ret = ReadFile(hComm, &receive_data, 2, &len, NULL);
	//cout << static_cast<bitset<8>>(receive_data[0]) << "," << static_cast<bitset<8>>(receive_data[1] )<< endl;

	int data1 = 0, data2 = 0;
	/*
	if (receive_data[0] | 128 > 0)
	{
	data1 = (~data1 << 8) | receive_data[0];
	}
	else data2 = receive_data[0];
	if (receive_data[1] | 128 > 0)
	{
	data2 = ~data1 << 8 | receive_data[1];
	}
	else data2 = receive_data[1];
	*/
	if ((int)receive_data[0] == 255)
	{
		data1 = -256 | receive_data[0];
	}
	else data1 = receive_data[0];
	if ((int)receive_data[1] == 255)
	{
		data2 = -256 | receive_data[1];
	}
	else data2 = receive_data[1];

	//DL = receive_data[0] * 2.5;
	//DR = receive_data[1] * 2.5;
	DL = (signed int)data1 * 24.78367538;
	DR = (signed int)data2 * 24.78367538;

	DIS = (DL + DR) / 2;
	ANG = (DL - DR) / 526;

	dist += DIS;
	rad += ANG;

	printf("Distance = %d , Angle = %f \n", (int)DIS, ANG);

	return ret;
}

static void set_3D_surface(urg_t *urg, long data[], int data_n, long time_stamp)
{
	printf(" set_3D_surface data_n = %d \n", data_n);

	(void)time_stamp;

	int i;
	long min_distance;
	long max_distance;

	if (chairpos >= 0){//�ʒu�̎擾�����������Ƃ�

		if (abs(chairpos - chairpos_old) >= scaninterval){//�֎q�̈ʒu���ݒ�l�ȏ㓮�����Ƃ�

			// �S�Ẵf�[�^�� X-Y �̈ʒu���擾
			urg_distance_min_max(urg, &min_distance, &max_distance);

			for (i = 0; i < data_n; ++i) {
				long l = data[i];
				double radian;
				float x, y, z;
				float pointpos[3];

				if ((l <= min_distance) || (l >= max_distance)) {
					continue;
				}

				radian = urg_index2rad(urg, i);
				x = (float)(l * cos(radian));
				y = (float)(l * sin(radian));
				z = 120.0;

				//2�������ʂ̍��W�ϊ�
				pointpos[0] = cos(urgpos[2]) * x + sin(urgpos[2]) * y + cos(urgpos[2]) * (chairpos - DIS_old) + startpos[0];
				pointpos[1] = -sin(urgpos[2]) * x + cos(urgpos[2]) * y - sin(urgpos[2]) * (chairpos - DIS_old) + startpos[1];
				pointpos[2] = z;

				//���W��ۑ�
				pcimage.writePoint(pointpos[0] / 1000, pointpos[1] / 1000, startpos[0], startpos[1]);
			}
		}
	}
}

int connectURG(){

	if (open_urg_sensor(&urg, COMID_urg, 0) < 0) {
		return 1;
	}

	data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
	if (!data) {
		perror("urg_max_index()");
		return 1;
	}

	printf("URG connected : urg_max_data_size =  %d \n", urg_max_data_size(&urg));
	urgconnected = 1;
	return 0;

}

int disconnectURG(){

	//�ؒf
	free(data);
	urg_close(&urg);

	printf("URG disconnected \n");
	urgconnected = 0;
	return 0;

}

int getData4URG(float& dist, float& rad){

	int n;
	int i;
	HANDLE hComm;

	char	COM_PORT[256];
	wsprintf(COM_PORT, "COM%d", COMID_Arduino_Encoder);

	//�f�[�^�擾
#if 0
	//�f�[�^�̎擾�͈͂�ύX����ꍇ
	urg_set_scanning_parameter(&urg,
		urg_deg2step(&urg, -90),
		urg_deg2step(&urg, +90), 0);
#endif

	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);

	for (i = 0; i < CAPTURE_TIMES; ++i) {
		n = urg_get_distance(&urg, data, &time_stamp);
		if (n <= 0) {
			printf("urg_get_distance: %s\n", urg_error(&urg));
			free(data);
			urg_close(&urg);
			urgconnected = 0;
			return 1;
		}

		//printf("%f , %f\n", chairpos , urgpos[2] );

		//if (-100 < (chairpos - chairpos_old) || (chairpos - chairpos_old) < 100){

		set_3D_surface(&urg, data, n, time_stamp);

		chairpos = dist;
		urgpos[2] = rad;

		//printf("%f , %f\n", chairpos , urgpos[2] );

		//}
		//else{

		//chairpos = dist;
		//urgpos[2] = rad;

		//	printf("%f , %f\n", chairpos , urgpos[2] );

		//}

	}

	return 0;

}



int getURGdata()
{
	HANDLE	hComm;
	int		ret = 0;
	char	COM_PORT[256];
	wsprintf(COM_PORT, "COM%d", COMID_Arduino_Encoder);

	//printf(" %d , %d \n", urgconnect, urgconnected);


	//hComm = CommOpen(COM_PORT);

	connectURG();

	/*if (urgconnect == 1 && urgconnected == 0){
	//connectURG();
	initRingBuffer();
	int ret;
	int sp1, sp2, sp3, sp4;
	float encl, encr;
	FILE *fp1, *fp2, *fp3, *fp4;
	hComm = CommOpen(COM_PORT);
	if (!hComm){
	printf("ERROR:Com port open error\n");
	}
	CommClose(hComm);
	}
	else if (urgconnect == 0 && urgconnected == 1){
	disconnectURG();
	}*/

	float dist = 0;
	float rad = 0;
	/*hComm = CreateFile(
	(LPCWSTR)COM_PORT,
	GENERIC_READ | GENERIC_WRITE,
	FILE_SHARE_READ | FILE_SHARE_WRITE,//COM�|�[�g�𑗐M���A��M���ŋ��L
	NULL,
	OPEN_EXISTING,
	FILE_ATTRIBUTE_NORMAL,
	NULL
	);*/
	//if (!hComm){
	//	printf("ERROR:Com port open error\n");
	//}
	//hComm = CreateFileA("\\\\.\\COM16", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
	hComm = CreateFile("\\\\.\\COM9", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hComm == INVALID_HANDLE_VALUE){
		printf("�V���A���|�[�g���J�����Ƃ��ł��܂���ł����B");
		char z;
		z = getchar();
		return 0;
	}
	else
	{
		DCB lpTest;
		GetCommState(hComm, &lpTest);
		lpTest.BaudRate = 9600;
		lpTest.ByteSize = 8;
		lpTest.Parity = NOPARITY;
		lpTest.StopBits = ONESTOPBIT;
		SetCommState(hComm, &lpTest);
	}

	while (1){

		startpos[0] = startpos[0] + (cos(urgpos[2]) - sin(urgpos[2])) * urgpos[1];
		startpos[1] = startpos[1] + (cos(urgpos[2]) + sin(urgpos[2])) * urgpos[1];

		Encoder(hComm, dist, rad);
		cout << "\n\n dist = " << dist << ", rad = " << rad << endl << endl;

		getData4URG(dist, rad);

		startpos[0] = cos(urgpos[2]) * (chairpos - DIS_old) + startpos[0];
		startpos[1] = -sin(urgpos[2]) * (chairpos - DIS_old) + startpos[1];
		printf("startpos[0] = %f , startpos[1] = %f\n", startpos[0], startpos[1]);

		DIS_old = chairpos;

	}

	CommClose(hComm);
}