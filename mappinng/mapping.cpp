#include "urg_unko.h"
#include "SharedMemory.h"
#include <Timer.h>
#include <receiveAndroidSensors.h>

#include <ypspur.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#define PI 3.14159265359

//#define KAISUU 10

using namespace std;
using namespace cv;

rcvAndroidSensors rcvDroid(24);
float defaultOrientation[3];

//���l�\���p
Mat picture;
Mat arrowpic;
Mat rotatepic;
Mat affine_mat;

bool isInitialized = false;
extern int imgWidth, imgHeight, imgResolution;

//���E�ւ̃G���R�[�_���f�[�^�ώZ�p
int data_L = 0, data_R = 0;

//1�J�E���g������̋���
double encord_dis_L=12.2544, encord_dis_R=12.228;

//���l�̕\��
void meter(Mat pic, float data[] , string name[], int NumOfData)
{
	int baseline = 0;
	pic = Mat::zeros(500, 1000, CV_8UC3);
	Size textSize = getTextSize("OpenCV ", FONT_HERSHEY_SIMPLEX, 2, 2, &baseline);
	for (int i = 0; i < NumOfData; i++){
		putText(pic, name[i] + " : " + to_string(data[i]), cv::Point(50, 50 + textSize.height*i), FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 200,0 ), 2, CV_AA);
	}
	
	imshow("meter", pic);
}

//��]�p����ŕ\��
void showDirection(float radian , string showName)
{
	affine_mat = getRotationMatrix2D(Point(arrowpic.cols / 2, arrowpic.rows / 2), -radian / PI * 180, 1);
	warpAffine(arrowpic, rotatepic, affine_mat, arrowpic.size());
	putText(rotatepic, to_string((int)(-radian / (2 * PI))), cv::Point(20, 50), FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(100, 0, 230), 2, CV_AA);
	imshow("direction" + showName, rotatepic);
}

int CommClose(HANDLE hComm)
{
	if (hComm){
		CloseHandle(hComm);
	}

	return 1;
}

/*
*	�T�v:
*		Arduino���獶�E�ւ̕ω��ʂ��擾���C�ړ��ʁC��]�ʂ�ώZ����
*	�����F
*		HANDLE hComm	Arduino�̃n���h��
*		float& dist		�ړ��ʂ�ώZ����ϐ��ւ̎Q��
*		float& rad		��]�ʂ�ώZ����ϐ��ւ̎Q��
*	�Ԃ�l:
*		int ret	�����������ǂ���
*/
int Encoder(HANDLE hComm, float& dist, float& rad)
{
	unsigned char	sendbuf[1];
	unsigned char	receive_data[2];
	int				ret;
	float			DL, DR, DIS, ANG;
	unsigned long	len;

	float			droidOrientation[3];

	// �n���h���`�F�b�N
	if( !hComm )	return -1;
	// �o�b�t�@�N���A
	memset(sendbuf, 0x00, sizeof(sendbuf));
	// �p�P�b�g�쐬
	sendbuf[0] = (unsigned char)1;
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);
	// ���M
	ret = WriteFile(hComm, &sendbuf, 1, &len, NULL);

	// �o�b�t�@�N���A
	memset(receive_data, 0x00, sizeof(receive_data));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);
	// Arduino����f�[�^����M
	ret = ReadFile(hComm, &receive_data, 2, &len, NULL);
	//cout << static_cast<bitset<8>>(receive_data[0]) << "," << static_cast<bitset<8>>(receive_data[1] )<< endl;


	//����������Ă��Ȃ���Ώ�����(���߂̃f�[�^���̂Ă�)
	if (!isInitialized)
	{
		isInitialized = true;
		return 0;
	}

	//�擾�����l�𕄍����ɑ��
	signed char receive_char1, receive_char2;
	receive_char1 = receive_data[0];
	receive_char2 = receive_data[1];

	// �f�[�^��ώZ
	data_L += static_cast<int>(receive_char1);
	data_R += static_cast<int>(receive_char2);

	//���E�ւ̉�]�ʂ���ړ��ʂ��v�Z
	DL = receive_char1 * encord_dis_L;/////////////////////////////////////////////////////////////////////////////
	DR = receive_char2 * encord_dis_R;

	//�ړ������C��]�ʂ��v�Z
	DIS = (DL + DR) / 2;
	ANG = -(DL - DR) / 541;	//�E��]����

	//�ړ��ʁC��]�ʂ�ώZ�p�ϐ��֐ώZ
	dist += DIS;
	rad += ANG;
	//rcvDroid.getOrientationData(droidOrientation);
	//rad = droidOrientation[0] - defaultOrientation[0];

	return ret;
}

/*
 *	�T�v:
 *		Arduino�ƃV���A���ʐM���s�����߂̃n���h�����擾����
 *	�����F
 *		HANDLE&	hComm	�n���h���ϐ��ւ̎Q��
 *	�Ԃ�l:
 *		�Ȃ�
 */
void getArduinoHandle(int arduinoCOM , HANDLE& hComm)
{
	//�V���A���|�[�g���J���ăn���h�����擾
	string com = "\\\\.\\COM" + to_string(arduinoCOM);
	hComm = CreateFile( com.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hComm == INVALID_HANDLE_VALUE){
		printf("�V���A���|�[�g���J�����Ƃ��ł��܂���ł����B");
		char z;
		z = getchar();
		return;
	}
	//�|�[�g���J���Ă���ΒʐM�ݒ���s��
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
}

/*
*	�T�v:
*		�}�b�v�쐬�p�̃��C�����[�v
*		'q'�ŏI��
*		�K�؂ɏI�����Ȃ��Ɖ摜���ۑ�����Ȃ�
*	�����F
*		int URG_COM[]	�ڑ�����URG��COM�|�[�g�̔z��
*		int ARDUINO_COM	�ڑ�����Arduino��COM�|�[�g

*	�Ԃ�l:
*		�Ȃ�
*/
void getDataUNKOOrigin(int URG_COM[], float URGPOS[][4], int ARDUINO_COM, int NumOfURG)
{
	/**********************
	 *�@���@�ϐ��̐錾�@��
	 **********************/

	HANDLE handle_ARDUINO;	//Arduino�p�n���h��

	urg_mapping *unkoArray = new urg_mapping[NumOfURG];	//urg_unko�^�ϐ��̔z��

	Timer	timer; //���[�v�̊Ԋu����&�C���^�[�o������p�^�C�}�[
	int		interval;
	timer.Start();

	float currentCoord[2] = {};	//����J�n�ʒu���猩�����݂̈ʒu

	float chairdist = 0.0;//�Ԃ����̈ړ���
	float chairdist_old = 0.0;

	float dist = 0.0;	//�ړ������̐ώZ�p�ϐ�
	float rad = 0.0;	//��]�ʂ̐ώZ�p�ϐ�
	float spur_x = 0.0, spur_y = 0.0;//spur�p���{�b�g���W
	float spur_x_old = 0.0, spur_y_old = 0.0;//spur�p���{�b�g���W
	double rob_x,rob_y,rob_rad;
	/*
	float spur_x_old=0.0,spur_y_old=0.0,spur_x,spur_y;
	
	*/

	// ���l�\���p�̕ϐ��B
	string meterName[] = {"dataL","dataR", "Difference of encoder value(L-R)", "Ratio of encoder value(L/R[%])", 
							"Current coordinates X", "Current coordinates Y", "Moving distance[mm]", "Angle variation[deg]",
							"Interval[millisec]"};
	float		meterData[9] = {};

	// csForm�Ƃ̌�����
	// ���[�v������^�C�~���O�Ƃ��̂����p
	SharedMemory<int> shMemInt("MappingFormInt");
	enum {ISEND , INTERVALTIME};
	shMemInt.setShMemData(false, ISEND);

	/****************************
	*�@���@�������̏������@��
	*****************************/

	// �p���\���p���̓ǂݍ���
	arrowpic = imread("arrow.jpg");
	if (arrowpic.empty()) arrowpic = imread("arrow.jpg");
	if (arrowpic.empty()) cout << "No arrow image" << endl;
	arrowpic = ~arrowpic;
	//rcvDroid.getOrientationData(defaultOrientation);



	//Arduino�ƃV���A���ʐM���s�����߂̃n���h�����擾
	getArduinoHandle(ARDUINO_COM,handle_ARDUINO);
	//�G���R�[�_�̏�����
	Encoder(handle_ARDUINO, dist, rad);

	//yp-spur�C�j�V�����C�Y

	// Windows���ŕW���o�͂��o�b�t�@�����O����Ȃ��悤�ɐݒ�
	setvbuf(stdout, 0, _IONBF, 0);

	// ������
	if (Spur_init() < 0)
	{
		fprintf(stderr, "ERROR : cannot open spur.\n");
		return;
	}


	PCImage::isColor = false;
	PCImage::BGR color[2] = { PCImage::B, PCImage::G };
	
	urg_mapping::initPCImage(imgWidth,imgHeight,imgResolution);
	urg_mapping::setPCImageOrigin(imgWidth / 2, imgHeight / 2);

	//�ڑ�����URG�̐�����urg_unko�^�I�u�W�F�N�g��������
	for (int i = 0; i < NumOfURG; i++)
	{
		unkoArray[i].init(URG_COM[i], URGPOS[i]);
		unkoArray[i].setWriteLine(false);
		unkoArray[i].setPCDDir();
	}

	/*********************
	*�@���@���C�������@��
	**********************/

	//�}�b�v�쐬���s�����[�v
	//'q'����͂���ƃ��[�v�𔲂���
#ifndef KAISUU
	while (true){
#else
	for (int i = 0; i < KAISUU; i++){
#endif
		// �����̊Ԋu���w�莞�Ԃ�����
		if (timer.getLapTime(1, Timer::millisec, false) < shMemInt.getShMemData(INTERVALTIME)) continue;
		interval = timer.getLapTime();


		/*
		//�G���R�[�_����ړ��ʁC��]�ʂ��擾
		Encoder(handle_ARDUINO, dist, rad);
		*/
		
		Spur_get_pos_GL(&rob_x, &rob_y, &rob_rad);
		
		spur_x = (float)rob_x;
		spur_y = (float)rob_y;
		rad = (float)rob_rad;

		/*
		//�ώZ�����������i�[
		chairdist = dist;
		*/
		
		chairdist += sqrt(pow(spur_x, 2) + pow(spur_y, 2)) - sqrt(pow(spur_x_old, 2) + pow(spur_y_old, 2));
		

		//URG����f�[�^���擾���C�G���R�[�_�̒l����Ƀ}�b�v�Cpcd�t�@�C�����쐬
		for (int i = 0; i < NumOfURG; i++)
		{
			unkoArray[i].updateCurrentCoord(currentCoord);
			unkoArray[i].setPCImageColor(color[i]);
			unkoArray[i].writeMap(dist,chairdist_old, rad,spur_x,spur_y);
			unkoArray[i].saveRawPCD(dist,rad);
		}
		/*
		//���݂̈ʒu���X�V
		//����J�n���_�����
		//		x�̐��F�O
		//		y�̐��F��
		currentCoord[0] += cos(rad) * (chairdist - chairdist_old);
		currentCoord[1] -= sin(rad) * (chairdist - chairdist_old);
		*/

		currentCoord[0] = spur_x;
		currentCoord[1] = spur_y;
		
		/*
		//���݂̈ړ��ʂ�ۑ�
		chairdist_old = chairdist;
		*/
		
		spur_x_old = spur_x;
		spur_y_old = spur_y;
		

		//'q'�����͂��ꂽ�烋�[�v�𔲂���
		// �������͋��L��������0�Ԓn��0�����͂�(ry
		if (cv::waitKey(1) == 'q' || shMemInt.getShMemData(0)) break;

		// ���[�^�[�̕\����ݒ�
		{
			meterData[0] = data_L;
			meterData[1] = data_R;
			meterData[2] = data_L - data_R;
			if (data_R)	meterData[3] = (float)data_L / (float)data_R * 100;
			else meterData[3] = 0;
			meterData[4] = currentCoord[0];
			meterData[5] = currentCoord[1];
			meterData[6] = dist;
			meterData[7] = rad / PI *180;
			meterData[8] = interval;

			meter(picture, meterData, meterName, 9);
			showDirection( -rad , ":Encoder" );
		}

	}

	//New�Ŋm�ۂ����z��̉��
	//delete[] unkoArray;
	// �\�����Ă���摜�����
	destroyAllWindows();
	//Arduino�̃n���h�������
	CommClose(handle_ARDUINO);

	return;
}
