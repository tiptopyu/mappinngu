#include "receiveAndroidSensors.h"
#include <iostream>
#include <string>

using namespace std;

// 2015/10/09

rcvAndroidSensors::rcvAndroidSensors() : shMem(shMemName)
{
	minSaveInterval = 5000;
}

rcvAndroidSensors::rcvAndroidSensors(int comport) : shMem(shMemName)
{
	setAndroidSensors(comport);
	minSaveInterval = 5000;
}

rcvAndroidSensors::~rcvAndroidSensors()
{
	comClose();
}

void rcvAndroidSensors::setAndroidSensors(int comport)
{
	COM = comport;
	comOpen();
}

void rcvAndroidSensors::comOpen()
{
	DCB				cDcb;		// �ʐM�ݒ�p
	COMMTIMEOUTS	cTimeouts;	// �ʐM�|�[�g�^�C���A�E�g�p

	// �ʐM�|�[�g���J��
	string com = "\\\\.\\COM" + to_string(COM);
	hComm = CreateFile(com.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	// �n���h���̊m�F
	if (hComm == INVALID_HANDLE_VALUE){
		hComm = NULL;
		LPVOID lpMsgBuf;
		FormatMessage(				//�G���[�\��������쐬
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL, GetLastError(),
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR)&lpMsgBuf, 0, NULL);

		MessageBox(NULL, (const char*)lpMsgBuf, NULL, MB_OK);	//���b�Z�[�W�\��
		return;
	}


	// �ʐM�ݒ�
	if (!GetCommState(hComm, &cDcb)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:GetCommState error\n");
		comClose();
		hComm = NULL;
	}
	//cDcb.BaudRate = 115200;				// �ʐM���x
	cDcb.BaudRate = 9600;				// �ʐM���x
	cDcb.ByteSize = 8;					// �f�[�^�r�b�g��
	cDcb.fParity = TRUE;				// �p���e�B�`�F�b�N
	cDcb.Parity = NOPARITY;			// �m�[�p���e�B
	cDcb.StopBits = ONESTOPBIT;			// 1�X�g�b�v�r�b�g

	if (!SetCommState(hComm, &cDcb)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:GetCommState error\n");
		comClose();
		hComm = NULL;
	}


	// �ʐM�ݒ�(�ʐM�^�C���A�E�g�ݒ�)
	// �����̓ǂݍ��ݑ҂�����(ms)
	cTimeouts.ReadIntervalTimeout = 50;
	// �ǂݍ��݂̂P����������̎���(ms)
	cTimeouts.ReadTotalTimeoutMultiplier = 50;
	// �ǂݍ��݂̒萔����(ms)
	cTimeouts.ReadTotalTimeoutConstant = 50;
	// �������݂̂P����������̎���(ms)
	cTimeouts.WriteTotalTimeoutMultiplier = 0;

	if (!SetCommTimeouts(hComm, &cTimeouts)){
		// �ʐM�ݒ�G���[�̏ꍇ
		printf("ERROR:SetCommTimeouts error\n");
		comClose();
		hComm = NULL;
	}

	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);
}

void rcvAndroidSensors::comClose()
{
	if (hComm){
		CloseHandle(hComm);
	}
	cout << "Port closed" << endl;
}

void rcvAndroidSensors::PreparationGPSCSV()
{
	timerGPS.Start();
	timeCountGPS = 0;

	ofsGPS.open("GPSdata_" + timerGPS.getNowTime() + ".csv");
	ofsOrientation << timerGPS.getNowTime() << ","
		<< "Latitude,"
		<< "Longitude,"
		<< "Accuracy," << endl;

	isPreparationGPSofs = true;
}
void rcvAndroidSensors::PreparationOrientationCSV()
{
	timerOrientation.Start();
	timeCountOrientation = 0;

	ofsOrientation.open("OrientationData_" + timerOrientation.getNowTime() + ".csv");
	ofsOrientation << timerOrientation.getNowTime() << ","
		<< "Azimuth,"
		<< "Pitch,"
		<< "Roll," << endl;

	isPreparationOrientationofs = true;
}

void rcvAndroidSensors::setIsSaveGPSCSV(bool isSaveCSV)
{
	isSaveGPSCSV = isSaveCSV;

	if (isSaveGPSCSV && !isPreparationGPSofs)
	{
		PreparationGPSCSV();
	}
}
void rcvAndroidSensors::setIsSaveOrientationCSV(bool isSaveCSV)
{
	isSaveOrientationCSV = isSaveCSV;

	if (isSaveOrientationCSV && !isPreparationOrientationofs)
	{
		PreparationOrientationCSV();
	}
}


void rcvAndroidSensors::getSensorData()
{
	unsigned char	sendbuf[128] = { (byte)0x01, (unsigned char)1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, };
	unsigned char	readbuf[128] = {};
	unsigned char	sum;
	int				i;
	int				ret;
	unsigned long	len, readlen;
	float			dLatitude, dLongitude;


	// �n���h���`�F�b�N
	if (!hComm){
		return;
	}
	{
		// �o�b�t�@�N���A
		//memset(sendbuf, 0x01, sizeof(sendbuf));

		// �ʐM�o�b�t�@�N���A
		PurgeComm(hComm, PURGE_RXCLEAR);

		// ���M
		ret = WriteFile(hComm, &sendbuf, 10, &len, NULL);
	}
	// �ǂݍ���
	memset(readbuf, 0x00, sizeof(readbuf));
	readlen = 11;
	len = 0;

	ret = ReadFile(hComm, readbuf, readlen, &len, NULL);

	// GPS�̃f�[�^
	if (readbuf[0] == 1){
		//�f�[�^����
		dLatitude = (readbuf[1] << 16) + (readbuf[2] << 8) + readbuf[3];
		dLongitude = (readbuf[4] << 16) + (readbuf[5] << 8) + readbuf[6];
		mAccuracy = (readbuf[7] << 8) + readbuf[8];

		//�������킹
		if ((readbuf[9] & 1) > 0) dLatitude = -dLatitude;
		if ((readbuf[9] & 2) > 0) dLongitude = -dLongitude;

		//�����킹
		dLatitude /= 100000;
		dLongitude /= 100000;
		mAccuracy /= 10;

		// �Ԃ��Ă���͕̂ψʂȂ̂ŐώZ
		mLatitude += dLatitude;
		mLongitude += dLongitude;

		printf("--GPS�ψ�--\n %.6f , %.6f \n", dLatitude, dLongitude);
		printf("--GPS��Βl--\n %.6f , %.6f , %.1f \n\n", mLatitude, mLongitude, mAccuracy);

		// �w�肳�ꂽ�Ԋu�ŕۑ�
		timeCountGPS += timerGPS.getLapTime();
		if (timeCountGPS > minSaveInterval && isSaveGPSCSV)
		{
			ofsGPS << timerGPS.getNowTime() << ","
				<< mLatitude << ","
				<< mLongitude << ","
				<< mAccuracy << "," << endl;
		}

	}
	// �p���̃f�[�^
	else if (readbuf[0] == 2){
		// �f�[�^�̕���
		mAzimuth = (readbuf[1] << 8) + readbuf[2];
		mPitch = (readbuf[3] << 8) + readbuf[4];
		mRoll = (readbuf[5] << 8) + readbuf[6];

		// �����킹��-180�`180�ɕϊ�
		mAzimuth = mAzimuth / 100 - 180;
		mPitch = mPitch / 100 - 180;
		mRoll = mRoll / 100 - 180;

		//printf("--orientation--\n %.2f , %.2f , %.2f \n", mAzimuth, mPitch, mRoll);

		// �w��Ԋu�ŕۑ�
		timeCountOrientation += timerOrientation.getLapTime();
		if (timeCountOrientation > minSaveInterval && isSaveOrientationCSV)
		{
			ofsOrientation << timerOrientation.getNowTime() << ","
				<< mAzimuth	<< ","
				<< mPitch	<< ","
				<< mRoll	<< "," << endl;
			cout << "������i �O�ցO�j�� �ۑ������ް�" << endl;
		}

	}
	// GPS�̏����l
	else if (readbuf[0] == 3){
		mLatitude = (readbuf[1] << 24) + (readbuf[2] << 16) + (readbuf[3] << 8) + readbuf[4];
		mLongitude = (readbuf[5] << 24) + (readbuf[6] << 16) + (readbuf[7] << 8) + readbuf[8];
		mAccuracy = (readbuf[9] << 8) + readbuf[10];

		mLatitude = mLatitude / 1000000;
		mLongitude = mLongitude / 1000000;
		mAccuracy = mAccuracy / 10;

		printf("--GPS--\n %.6f , %.6f , %.1f \n\n", mLatitude, mLongitude, mAccuracy);
		timeCountGPS += timerGPS.getLapTime();
		if (timeCountGPS > minSaveInterval && isSaveGPSCSV)
		{
			ofsGPS << timerGPS.getNowTime() << ","
				<< mLatitude	<< ","
				<< mLongitude	<< ","
				<< mAccuracy	<< "," << endl;
		}

	}
}

void rcvAndroidSensors::setSaveMinInterval(int interval)
{
	minSaveInterval = interval;
}

void	rcvAndroidSensors::getOrientationData(float retArray[3])
{
	unsigned char	sendbuf[1];
	unsigned char	readbuf[20];
	int				ret;
	unsigned long	len, readlen;

	// �n���h���`�F�b�N
	if (!hComm){
		cout << "No handle" << endl;
		return;
	}
	
	// �o�b�t�@�N���A
	memset(sendbuf, 0x02, sizeof(sendbuf));
	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);
	// ���M
	ret = WriteFile(hComm, &sendbuf, 1, &len, NULL);

	// �ǂݍ���
	memset(readbuf, 0x00, sizeof(readbuf));
	readlen = 11;
	len = 0;
	ret = ReadFile(hComm, readbuf, readlen, &len, NULL);

	// �p���̃f�[�^
	if (readbuf[0] == 2){
		// �f�[�^�̕���
		mAzimuth = (readbuf[1] << 8) + readbuf[2];
		mPitch = (readbuf[3] << 8) + readbuf[4];
		mRoll = (readbuf[5] << 8) + readbuf[6];

		// �����킹��-180�`180�ɕϊ�
		mAzimuth = mAzimuth / 100 - 180;
		mPitch = mPitch / 100 - 180;
		mRoll = mRoll / 100 - 180;

		retArray[0] = mAzimuth;
		retArray[1] = mPitch;
		retArray[2] = mRoll;

		//printf("--orientation--\n %.2f , %.2f , %.2f \n", mAzimuth, mPitch, mRoll);

		// �w��Ԋu�ŕۑ�
		if(isSaveOrientationCSV) timeCountOrientation += timerOrientation.getLapTime();
		if (timeCountOrientation > minSaveInterval && isSaveOrientationCSV)
		{
			ofsOrientation << timerOrientation.getNowTime() << ","
				<< mAzimuth << ","
				<< mPitch << ","
				<< mRoll << "," << endl;
			cout << "������i �O�ցO�j�� �ۑ������ް�" << endl;
		}

		if (isSaveSharedMemory)
		{
			shMem.setShMemData(mAzimuth, AZIMUTH);
			shMem.setShMemData(mPitch, PITCH);
			shMem.setShMemData(mRoll, ROLL);
		}

	}
	else
	{
		cout << "Not Orientation data" << endl;
		//retArray[0] = 0;
		//retArray[1] = 0;
		//retArray[2] = 0;
	}
}

void rcvAndroidSensors::getGPSData(float retArray[3])
{
	unsigned char	sendbuf[1];
	unsigned char	readbuf[20];
	int				ret;
	unsigned long	len, readlen;

	// �n���h���`�F�b�N
	if (!hComm){
		cout << "No handle" << endl;
		return;
	}

	// �o�b�t�@�N���A
	memset(sendbuf, 0x01, sizeof(sendbuf));

	// �ʐM�o�b�t�@�N���A
	PurgeComm(hComm, PURGE_RXCLEAR);

	// ���M
	ret = WriteFile(hComm, &sendbuf, 1, &len, NULL);

	// �ǂݍ���
	memset(readbuf, 0x00, sizeof(readbuf));
	readlen = 11;
	len = 0;

	ret = ReadFile(hComm, readbuf, readlen, &len, NULL);

	// GPS�̏����l
	if (readbuf[0] == 3){
		mLatitude = (readbuf[1] << 24) + (readbuf[2] << 16) + (readbuf[3] << 8) + readbuf[4];
		mLongitude = (readbuf[5] << 24) + (readbuf[6] << 16) + (readbuf[7] << 8) + readbuf[8];
		mAccuracy = (readbuf[9] << 8) + readbuf[10];

		mLatitude = mLatitude / 1000000;
		mLongitude = mLongitude / 1000000;
		mAccuracy = mAccuracy / 10;

		retArray[0] = mLatitude;
		retArray[1] = mLongitude;
		retArray[2] = mAccuracy;

		//printf("--GPS--\n %.6f , %.6f , %.1f \n\n", mLatitude, mLongitude, mAccuracy);

		// �w��Ԋu�ŕۑ�
		if (isSaveGPSCSV) timeCountGPS += timerGPS.getLapTime();
		if (timeCountGPS > minSaveInterval && isSaveGPSCSV)
		{
			ofsGPS << timerGPS.getNowTime() << ","
				<< mLatitude << ","
				<< mLongitude << ","
				<< mAccuracy << "," << endl;
		}
		if (isSaveSharedMemory)
		{
			shMem.setShMemData(mLatitude, LATITUDE);
			shMem.setShMemData(mLongitude, LONGITUDE);
			shMem.setShMemData(mAccuracy, ACCURACY);
		}

	}
	else
	{
		cout << "Not GPS data" << endl;
		//retArray[0] = 0;
		//retArray[1] = 0;
		//retArray[2] = 0;
	}

}

