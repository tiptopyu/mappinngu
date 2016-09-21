#include <urg_unko.h>
using namespace std;

int imgWidth, imgHeight, imgResolution;

int main(int argc, char* argv[])
{
	//URGのCOMポートを指定
	int URG_COM[] = { 11 };

	//URGの位置を指定(回転中心が原点)
	//urgPOS[][4]={x,y,z,rot}
	float urgPOS[][4] = { 383.0, 375, 0, 0 };
	//float urgPOS[][4] = { 685.0, -325.0, 110.0, -1.5708,
	//	685.0, 325.0, 110.0, 1.5708 };

	//ArduinoのCOMポートを指定
	int ARDUINO_COM = 6;

	//pcimageの引数
	imgWidth = 1000;
	imgHeight = 1000;
	imgResolution = 5;


	cout << argc << endl;
	for (int i = 0; i < argc; i++)
	{
		cout << argv[i] << endl;
	}

	// コマンドライン引数からパラメータを受け取る
	// URGは右のパラメータから受け取る
	if (argc == 15)
	{
		//URGのCOMポートを指定
		URG_COM[0] = atoi(argv[1]);
		URG_COM[1] = atoi(argv[2]);
		//ArduinoのCOMポートを指定
		ARDUINO_COM = atoi(argv[3]);

		//URGの位置を指定
		for (int i = 0; i < 4; i++) urgPOS[0][i] = atof(argv[i + 4]);
		for (int i = 0; i < 4; i++) urgPOS[1][i] = atof(argv[i + 8]);
		//pcimageの引数
		imgWidth = atoi(argv[12]);
		imgHeight = atoi(argv[13]);
		imgResolution = atoi(argv[14]);

		cout << "csFormから起動" << endl;
	}


	cout << "\n　接続したURGの個数：" << sizeof(URG_COM) / sizeof(URG_COM[0]) << endl << endl;

	getDataUNKO(URG_COM, urgPOS, ARDUINO_COM);

	//z = getchar();

	return 0;
}