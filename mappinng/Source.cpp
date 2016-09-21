#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>

/*

�G���[	1	error LNK2019: �������̊O���V���{�� "public: __thiscall urg_unko::urg_unko(void)" (??0urg_unko@@QAE@XZ) ���֐� 
"public: __thiscall urg_mapping::urg_mapping(void)" (??0urg_mapping@@QAE@XZ) �ŎQ�Ƃ���܂����B	c:\Users\user\documents\visual studio 2013\Projects\mappinng\mappinng\Source2.obj	mappinng


int main(void)
{
	urg_t urg;
	int ret;
	long *length_data;
	int length_data_size;
	const int scan_times = 123;
	int i;

	// \~japanese "COM1" �́A�Z���T���F������Ă���f�o�C�X���ɂ���K�v������
	const char connect_device[] = "COM1";
	const long connect_baudrate = 115200;

	// \~japanese �Z���T�ɑ΂��Đڑ����s���B
	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
	// \todo check error code

	// \~japanese �f�[�^��M�̂��߂̗̈���m�ۂ���
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	// \todo check length_data is not NULL

	// \~japanese �����f�[�^�̌v���J�n�B
	ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	// \todo check error code

	// \~japanese �Z���T���狗���f�[�^���擾����B
	for (i = 0; i < scan_times; ++i) {
		length_data_size = urg_get_distance(&urg, length_data, NULL);
	}	// \todo process length_data array

	// \~japanese �Z���T�Ƃ̐ڑ������B
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

	// \~japanese "COM1" �́A�Z���T���F������Ă���f�o�C�X���ɂ���K�v������
	const char connect_device[] = "COM11";
	const long connect_baudrate = 115200;

	// \~japanese �Z���T�ɑ΂��Đڑ����s���B
	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
	// \todo check error code

	// \~japanese �f�[�^��M�̂��߂̗̈���m�ۂ���
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	// \todo check length_data is not NULL

	// \~japanese �����f�[�^�̌v���J�n�B
	
	// \todo check error code

	// \~japanese �Z���T���狗���f�[�^���擾����B
	
		
		// \todo process length_data array

	
	

	cv::Mat img = cv::Mat::zeros(800, 800, CV_8UC3);
	cv::Mat cpy_img = cv::Mat::zeros(800, 800, CV_8UC3);

	
	/*
	// Red�C����3�C4�ߖT�A��
	cv::line(img, cv::Point(0, 0), cv::Point(400, 105), cv::Scalar(0, 0, 200), 3, 4);
	// Green�C����5�C8�ߖT�A��
	cv::line(img, cv::Point(100, 200), cv::Point(400, 205), cv::Scalar(0, 200, 0), 5, 8);
	// Blue�C����10�C�A���`�G�C���A�X
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
			
			// \~japanese ���̋����f�[�^�̃��W�A���p�x�����߁AX, Y �̍��W�l���v�Z����
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
			// Esc���͂ŏI�� 
			break;
		}
		else if (GetAsyncKeyState('S'))
		{ // 's'�L�[����
			printf("'s'�L�[�������ꂽ\n");
			
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
	
	// \~japanese �Z���T�Ƃ̐ڑ������B
	urg_close(&urg);
	return 0;
	cv::waitKey(0);
}