#include <urg_unko.h>
using namespace std;

int imgWidth, imgHeight, imgResolution;

int main(int argc, char* argv[])
{
	//URG��COM�|�[�g���w��
	int URG_COM[] = { 11 };

	//URG�̈ʒu���w��(��]���S�����_)
	//urgPOS[][4]={x,y,z,rot}
	float urgPOS[][4] = { 383.0, 375, 0, 0 };
	//float urgPOS[][4] = { 685.0, -325.0, 110.0, -1.5708,
	//	685.0, 325.0, 110.0, 1.5708 };

	//Arduino��COM�|�[�g���w��
	int ARDUINO_COM = 6;

	//pcimage�̈���
	imgWidth = 1000;
	imgHeight = 1000;
	imgResolution = 5;


	cout << argc << endl;
	for (int i = 0; i < argc; i++)
	{
		cout << argv[i] << endl;
	}

	// �R�}���h���C����������p�����[�^���󂯎��
	// URG�͉E�̃p�����[�^����󂯎��
	if (argc == 15)
	{
		//URG��COM�|�[�g���w��
		URG_COM[0] = atoi(argv[1]);
		URG_COM[1] = atoi(argv[2]);
		//Arduino��COM�|�[�g���w��
		ARDUINO_COM = atoi(argv[3]);

		//URG�̈ʒu���w��
		for (int i = 0; i < 4; i++) urgPOS[0][i] = atof(argv[i + 4]);
		for (int i = 0; i < 4; i++) urgPOS[1][i] = atof(argv[i + 8]);
		//pcimage�̈���
		imgWidth = atoi(argv[12]);
		imgHeight = atoi(argv[13]);
		imgResolution = atoi(argv[14]);

		cout << "csForm����N��" << endl;
	}


	cout << "\n�@�ڑ�����URG�̌��F" << sizeof(URG_COM) / sizeof(URG_COM[0]) << endl << endl;

	getDataUNKO(URG_COM, urgPOS, ARDUINO_COM);

	//z = getchar();

	return 0;
}