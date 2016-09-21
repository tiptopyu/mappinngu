#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>

/*

エラー	1	error LNK2019: 未解決の外部シンボル "public: __thiscall urg_unko::urg_unko(void)" (??0urg_unko@@QAE@XZ) が関数 
"public: __thiscall urg_mapping::urg_mapping(void)" (??0urg_mapping@@QAE@XZ) で参照されました。	c:\Users\user\documents\visual studio 2013\Projects\mappinng\mappinng\Source2.obj	mappinng


int main(void)
{
	urg_t urg;
	int ret;
	long *length_data;
	int length_data_size;
	const int scan_times = 123;
	int i;

	// \~japanese "COM1" は、センサが認識されているデバイス名にする必要がある
	const char connect_device[] = "COM1";
	const long connect_baudrate = 115200;

	// \~japanese センサに対して接続を行う。
	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
	// \todo check error code

	// \~japanese データ受信のための領域を確保する
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	// \todo check length_data is not NULL

	// \~japanese 距離データの計測開始。
	ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	// \todo check error code

	// \~japanese センサから距離データを取得する。
	for (i = 0; i < scan_times; ++i) {
		length_data_size = urg_get_distance(&urg, length_data, NULL);
	}	// \todo process length_data array

	// \~japanese センサとの接続を閉じる。
	urg_close(&urg);

	return 0;
}*/
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int
main(int argc, char *argv[])
{


	urg_t urg;
	int ret;
	long *length_data;
	int length_data_size;
	const int scan_times = 3;
	int i;

	// \~japanese "COM1" は、センサが認識されているデバイス名にする必要がある
	const char connect_device[] = "COM11";
	const long connect_baudrate = 115200;

	// \~japanese センサに対して接続を行う。
	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
	// \todo check error code

	// \~japanese データ受信のための領域を確保する
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	// \todo check length_data is not NULL

	// \~japanese 距離データの計測開始。
	
	// \todo check error code

	// \~japanese センサから距離データを取得する。
	
		
		// \todo process length_data array

	
	

	cv::Mat img = cv::Mat::zeros(800, 800, CV_8UC3);
	cv::Mat cpy_img = cv::Mat::zeros(800, 800, CV_8UC3);

	
	/*
	// Red，太さ3，4近傍連結
	cv::line(img, cv::Point(0, 0), cv::Point(400, 105), cv::Scalar(0, 0, 200), 3, 4);
	// Green，太さ5，8近傍連結
	cv::line(img, cv::Point(100, 200), cv::Point(400, 205), cv::Scalar(0, 200, 0), 5, 8);
	// Blue，太さ10，アンチエイリアス
	cv::line(img, cv::Point(100, 300), cv::Point(400, 305), cv::Scalar(0, 255, 255), 2, CV_AA);
*/
	cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	

	int c=0;
	int rb_x = 400, rb_y = 400;
	long min_distance;
	long max_distance;
	int ch = img.channels();
	int a;
	while (1) {
		ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
		length_data_size = urg_get_distance(&urg, length_data, NULL);
		urg_distance_min_max(&urg, &min_distance, &max_distance);
		for (i = 0; i < length_data_size; ++i) {
			
			// \~japanese その距離データのラジアン角度を求め、X, Y の座標値を計算する
			double radian;
			long length;
			int x;
			int y;

			radian = urg_index2rad(&urg, i);
			length = length_data[i];
			// \todo check length is valid

			x = (long)(length * cos(radian))/8+rb_x;
			y = (long)(length * sin(radian))/8+rb_y;
			if ((length > min_distance) && (length < max_distance))
			{
				
				cv::line(img, cv::Point(rb_x, rb_y), cv::Point(x, y), cv::Scalar(255, 0, 0), 0, 8);
				cv::circle(img, cv::Point(x, y), 0, cv::Scalar(0, 255, 255), -1, CV_AA);
				a = img.step*y + (x*img.elemSize());
				if (a>0)
				{
					img.data[a + 0] = 255;
					img.data[a + 1] = 255;
					img.data[a + 2] = 0;
				}
				
				
			}
			
		}
		cpy_img = img.clone();
		cv::circle(cpy_img, cv::Point(rb_x, rb_y), 30, cv::Scalar(0, 0, 200), 3, 4);
		cv::imshow("drawing", cpy_img);
		c = cvWaitKey(1);
		if (GetAsyncKeyState('Q'))
		{
			// Esc入力で終了 
			break;
		}
		else if (GetAsyncKeyState('S'))
		{ // 's'キー入力
			printf("'s'キーが押された\n");
			
		}
		else if (GetAsyncKeyState(VK_LEFT))
		{
			rb_x-=10;
		}
		else if (GetAsyncKeyState(VK_UP))
		{
			rb_y -= 10;
		}
		else if (GetAsyncKeyState(VK_RIGHT))
		{
			rb_x += 10;
		}
		else if (GetAsyncKeyState(VK_DOWN))
		{
			rb_y += 10;
		}
		
	}
	
	// \~japanese センサとの接続を閉じる。
	urg_close(&urg);
	return 0;
	cv::waitKey(0);
}