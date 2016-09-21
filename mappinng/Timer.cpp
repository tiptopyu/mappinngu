#include "Timer.h"

// 2015/10/05

#include <iostream>
#include <fstream>
#include <windows.h>

//������
void Timer::Start()
{
	rawLap.push_back(std::chrono::system_clock::now());
}

//���茋�ʂ�ۑ�
void Timer::Save( string filename,  tUnit unit)
{
	_time	now = std::chrono::system_clock::now();
	int		count = 0;

	ofstream ofs(filename, ios::app);
	for (int i = 0; i < lapTime.size(); i++)
	{
		ofs << "No." << count << " , " << lapTime[i] << " , " << unitname(unit) << endl;
	}

	cout << "Saved" << endl;
}

// �P�ʂ��w�肵�Čo�ߎ��Ԃ��擾
int Timer::getTime(std::chrono::time_point<std::chrono::system_clock> start, std::chrono::time_point<std::chrono::system_clock> end, tUnit unit)
{
	switch (unit)
	{
	case sec:
		return std::chrono::duration_cast<std::chrono::seconds>(end-start).count();
	case millisec:
		return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	case microsec:
		return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	case nanosec:
		return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
	default:
		return -1;
	}

}

// �P�ʂ𕶎���Ŏ擾
string Timer::unitname(tUnit unit)
{
	switch (unit)
	{
	case sec:
		return " seconds";
	case millisec:
		return " milliseconds";
	case microsec:
		return " microseconds";
	case nanosec:
		return " nanoseconds";
	default:
		return " ";
	}
}

// �w��񐔑O�̃��b�v�^�C������̌o�ߎ��Ԃ��擾
int Timer::getLapTime(int Criteria, tUnit unit , bool isSavaLap)
{
	int retTime;
	_time	now = std::chrono::system_clock::now();

	switch (Criteria)
	{
	case -1:
		retTime = getTime(rawLap[0], now, millisec);
		break;
	case 0 :
		return -1;
	default:
		retTime = getTime(rawLap[rawLap.size() - Criteria], now, millisec);
		break;
	}

	if (isSavaLap) rawLap.push_back(now);
	lapTime.push_back(retTime);

	//cout << "Lap time : " << retTime << " " << unitname(unit) << endl;
	return retTime;

}

//���݂̎����𕶎���Ŏ擾����
string Timer::getNowTime(){
	SYSTEMTIME st;
	GetSystemTime(&st);
	char szTime[25] = { 0 };
	// wHour���X���ԑ����āA���{���Ԃɂ���
	wsprintf(szTime, "%04d%02d%02d%02d%02d%02d",
		st.wYear, st.wMonth, st.wDay,
		st.wHour + 9, st.wMinute, st.wSecond);
	return (string)szTime;
}