#include "pcimage.h"

#include <fstream>
#include <iostream>
#include <direct.h>
#include <windows.h>

using namespace cv;
using namespace std;

bool PCImage::isColor = true;

/*------------------------------
*--��--PCImage�N���X�̒�`--��--
*-------------------------------*/

/*
*�@�R���X�g���N�^(�����L) 
*  ����:
*	int width �@�c
*	int height�@��
*	int resolution�@1pix��cm�l���ɂ��邩
*/
PCImage::PCImage() : pcimage(imageNum, *this)
{
	dirname = "";
}

//�f�X�g���N�^
//�������Ă��Ȃ��摜�̈悪����Εۑ����Ă���
PCImage::~PCImage()
{
	for (int i = 0; i < imageNum; i++)
		if (!pcimage[i].empty())
		{
			pcimage[i].release();
		}
}

PCImage& PCImage::operator = (PCImage& pci)
{
	return *this;
}

void PCImage::initPCImage(int width, int height, int resolution)
{
	//-----�����o�̏�����-----
	img_width = width;
	img_height = height;
	coefficient = 100 / resolution;
	imgval_increment = 25;
	limit = 10;
	limitpix = limit * coefficient;

	origin_x = limitpix;
	origin_y = img_height / 2;

	for (int i = 0; i < sizeof(color); i++) color[i] = false;
	//prepareArrow();

	//�N���������b�Ŗ��������f�B���N�g�����쐬
	if (dirname == ""){
		getNowTime(dirname);
		if (_mkdir(dirname.c_str()) == 0){
			cout << "Made a directory named:" << dirname << endl << endl;
		}
		else{
			cout << "Failed to make the directory" << endl;
		}
	}

	//pcimage[0]����������
	nowimage = 0;
	pcimage[nowimage].setPCI(0, 0);
	pcimage[nowimage] = initImage(width, height);


	cout << "Width:" << pcimage[nowimage].cols
		<< "\nHeight:" << pcimage[nowimage].rows << endl;

	Sleep(2000);
}
void PCImage::initPCImage()
{
	this->PCImage::initPCImage(1000, 1000, 5);
}
void PCImage::initPCImage(int resolution)
{
	this->PCImage::initPCImage(1000, 1000, resolution);
}

PCImage PCImage::instantiate()
{
	return *this;
}
void PCImage::setOrigin(int x, int y)
{
	origin_x = x;
	origin_y = y;
}
void PCImage::getImage(cv::Mat& m, int num )
{
	if (num == -1) num = nowimage;
	m = pcimage[num].clone();
}

void PCImage::prepareArrow()
{
	arrowpic = imread("arrow.jpg");
	arrowpic = ~arrowpic;
	resize(arrowpic, arrowpic, Size(arrowpic.cols / 2, arrowpic.rows / 2));

	Mat mask = arrowpic.clone();
	cvtColor(mask, mask, CV_BGR2GRAY);
	threshold(arrowpic, arrowpic, 100, 255, THRESH_BINARY);

	// �f�މ摜���`�����l��(RGB)���Ƃɕ�������vector�Ɋi�[����
	vector<Mat> mv;
	split(arrowpic, mv);

	// vector�̍Ō���Ƀ}�X�N�摜�̒��ڗ̈��ǉ�����
	mv.push_back(mask);

	// vector���������ĉ��H��̉摜�Ƃ���
	merge(mv, arrowpic);
}

void PCImage::showArrow()
{
	Mat pic(Size(pcimage[nowimage].cols, pcimage[nowimage].rows), CV_8UC3);
	cvtColor(pcimage[nowimage], pic, CV_GRAY2BGR);
	Mat roi(pcimage[nowimage], cv::Rect(0, 0, arrowpic.cols, arrowpic.rows));
	arrowpic.copyTo(roi);
	imshow(dirname, pic);
	waitKey(1);
}

/*���ݒn�̕`��
* PCImage::showNowPoint(float x_val, float y_val)
*float x_val
*float y_val
*/
void PCImage::showNowPoint(float x_val, float y_val)
{
	Mat showpic(Size(pcimage[nowimage].cols, pcimage[nowimage].rows), CV_8UC3);
	//cvtColor(pcimage[nowimage], showpic, CV_GRAY2BGR);
	showpic = pcimage[nowimage].clone();

	//x,y�̒l���w�肵���𑜓x�ɍ��킹��
	int xi = int(x_val * coefficient);
	int yi = int(y_val * coefficient);

	//���݂̉摜��XY
	int XY[2];

	pcimage[nowimage].getImageNumber(XY);		//���S�摜��X,Y�ԍ����擾

	// ���W���摜���ɍ��킹��
	xi = xi - XY[0] * img_width + origin_x;
	yi = yi - XY[1] * img_height + origin_y;

	circle(showpic, cv::Point(xi, yi), 10, cv::Scalar(0, 0, 255), 3);

	imshow(dirname, showpic);
	waitKey(1);
}
	
/*
*�@�T�v�F�w����W�ɓ_����������
*�@����:
*	float x_val x���W(m)
*	float y_val y���W(m)
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::writePoint(float x_val, float y_val)
{
	pcimage[nowimage].writePoint(x_val, y_val);
}
	
/*
*�@�T�v�F�w����W�Ǝ��Ȉʒu�̊Ԃɒ�����`��
*�@����:
*	float x_val �������ޓ_��x���W(m)
*	float y_val �������ޓ_��y���W(m)
*	float pos_x	���Ȉʒu��x���W(m)
*	float pos_y	���Ȉʒu��y���W(m)
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::writeLine(float x_val, float y_val, float pos_x, float pos_y)
{
	int XY[2];
	pcimage[nowimage].getImageNumber(XY);		//���S�摜��X,Y�ԍ����擾

	//x,y�̒l���w�肵���𑜓x�ɍ��킹��
	x_val *= coefficient;
	y_val *= coefficient;
	x_val = (int)x_val - XY[0] * img_width + origin_x;
	y_val = (int)y_val - XY[1] * img_height + origin_y;

	pos_x *= coefficient;
	pos_y *= coefficient;
	pos_x = (int)pos_x - XY[0] * img_width + origin_x;
	pos_y = (int)pos_y - XY[1] * img_height + origin_y;

	//�擾����[x,y]�ƌ��ݒn����Ō���
	pcimage[nowimage].line(Point(x_val, y_val), Point(pos_x, pos_y), 100);

}

/*
*�@�T�v�F�w����W(��΍��W)�ɓ_����������
*�@����:
*	float x_val �������ޓ_��x���W(m)
*	float y_val �������ޓ_��y���W(m)
*	float pos_x	���Ȉʒu��x���W(m)
*	float pos_y	���Ȉʒu��y���W(m)
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::writePoint(float x_val, float y_val, float pos_x, float pos_y)
{
	if(isWriteLine)	writeLine(x_val, y_val , pos_x ,pos_y );

	int ret;
	ret = pcimage[nowimage].writePoint(x_val, y_val);
	if (ret)
	{
		pcimage[ret - 1].writePoint(x_val, y_val);
	}

	// ���Ȉʒu���ω����Ă��Ȃ���Ώ�����Ԃ�
	if (selfPos_x == pos_x && selfPos_y == pos_y) return;

	// ���Ȉʒu�ɉ������������s��
	this->checkPosition(pos_x, pos_y);

	showNowPoint(pos_x,pos_y);

	selfPos_x = pos_x;
	selfPos_y = pos_y;

}

int PCImage::readPoint(int x_val, int y_val)
{
	//�w�肵��[x,y]�̉�f�l��Ԃ�
	return pcimage[nowimage].data[y_val * pcimage[nowimage].cols + x_val];
}

/*
*�@�T�v�F�摜��ۑ�
*�@����:
*�@	
*�@�Ԃ�l:
*	0
*/
void PCImage::savePCImage(int x, int y)
{
	for (int i = 0; i < imageNum; i++)
	{
		if (pcimage[i].isCoordinates(x, y)) pcimage[i].savePCImage();
	}
}
void PCImage::savePCImage()
{
	pcimage[nowimage].release();
}
void PCImage::savePCImage(int num, string savename)
{
	pcimage[num].savePCImage(savename);
}

std::string PCImage::getDirname()
{
	return dirname;
}

//������i �O�ցO�j�� �ް�
//���݂̎����𕶎���Ŏ擾����
void PCImage::getNowTime(string& nowstr){
	SYSTEMTIME st;
	GetSystemTime(&st);
	char szTime[25] = { 0 };
	// wHour���X���ԑ����āA���{���Ԃɂ���
	wsprintf(szTime, "%04d%02d%02d%02d%02d%02d",
		st.wYear, st.wMonth, st.wDay,
		st.wHour + 9, st.wMinute, st.wSecond);
	nowstr = szTime;
}

/*
*
*  ������i �O�ցO�j��     �摜�̈�Ǘ��̒���     ����i�O�ցO �j����
*
*�@�T�v�F�摜���̎��Ȉʒu���`�F�b�N���ĕK�v�ȏ������s��
*		�@�摜�̗p�ӂƕۑ��C���S�摜�̈ڍs
*�@����:
*	float pos_x	���Ȉʒu��x���W(m)
*	float pos_y	���Ȉʒu��y���W(m)
*�@�Ԃ�l:
*	0
*/
int PCImage::checkPosition(float pos_x, float pos_y)
{
	//x,y�̒l���w�肵���𑜓x�ɍ��킹��
	int xi = int(pos_x * coefficient);
	int yi = int(pos_y * coefficient);

	//���݂̉摜��XY
	int XY[2];

	pcimage[nowimage].getImageNumber(XY);		//���S�摜��X,Y�ԍ����擾

	// ���W���摜���ɍ��킹��
	xi = xi - XY[0] * img_width + origin_x;
	yi = yi - XY[1] * img_height + origin_y;

	cout << "Position:" << xi << "," << yi << endl;

	// ���Ȉʒu���w��͈͂𒴂��Ă�����...
	if (xi < limitpix || img_width - limitpix < xi || yi < limitpix || img_height - limitpix < yi){
		this->outsideProcess(xi, yi, XY);
	}
	// �w��͈͂𒴂��Ă��Ȃ��̂ɉ������Ă��Ȃ��摜������Ή��
	else
	{
		for (int i = 0; i < imageNum; i++)
		{
			if (i == nowimage) continue;
			if (!pcimage[i].empty()) pcimage[i].release();
		}
	}

	return 0;
}

void PCImage::outsideProcess(int pos_x, int pos_y, int XY[2])
{
	int flag_x = 0;
	int flag_y = 0;

	//��������̃��~�b�g�`�F�b�N��
	if (pos_y < limitpix)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_y = -1;
	}
	//���������̃��~�b�g�`�F�b�N��
	else if (img_width - limitpix < pos_y)
	{
		//���~�b�g�𒴂��Ă����ꍇ
		flag_y = 1;
	}

	//���E�����̃��~�b�g�`�F�b�N��
	if (img_height - limitpix < pos_x)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_x = 1;
	}
	//���������̃��~�b�g�`�F�b�N��
	else if (pos_x < limitpix)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_x = -1;
	}

	// X,Y���Ƀ��~�b�g�O
	if (flag_x && flag_y)
	{
		prepareImage(XY[0] + flag_x, XY[1] + flag_y);
		prepareImage(XY[0] + flag_x, XY[1]);
		prepareImage(XY[0], XY[1] + flag_y);
	}
	// �ǂ������������~�b�g
	else
	{
		prepareImage(XY[0] + flag_x, XY[1] + flag_y);

		for (int i = 0; i < imageNum; i++)
		{
			if (i == nowimage) continue;
			if (!pcimage[i].empty() && !pcimage[i].isCoordinates(XY[0] + flag_x, XY[1] + flag_y)) pcimage[i].release();
		}
	}

	// �摜�O�Ȃ炻�����̉摜�ɃV�t�g����(���\�b�h�̕�������ς���ƃX�b�L������Ǝv���邯�Ǔ��������炢����)
	flag_x = 0;
	flag_y = 0;
	//��������̃��~�b�g�`�F�b�N��
	if (pos_y < 0)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_y = -1;
	}
	//���������̃��~�b�g�`�F�b�N��
	else if (pcimage[nowimage].rows  < pos_y)
	{
		//���~�b�g�𒴂��Ă����ꍇ
		flag_y = 1;
	}

	//���E�����̃��~�b�g�`�F�b�N��
	if (pcimage[nowimage].cols < pos_x)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_x = 1;
	}
	//���������̃��~�b�g�`�F�b�N��
	else if (pos_x < 0)
	{
		// ���~�b�g�𒴂��Ă����ꍇ
		flag_x = -1;
	}
	//flag_x��������flag_y��0�ȊO�Ȃ�摜�V�t�g�����s
	if (flag_x || flag_y) shiftCenterImage(flag_x, flag_y);
}


/*
*�@�T�v�F�摜��ǂݍ���
*�@����:
*�@	int emptyImageNum �摜�ԍ�
*�@�Ԃ�l:
*	0
*/
int PCImage::loadPCImage(int emptyImageNum)
{
	// �摜��ǂݍ���
	pcimage[emptyImageNum] = imread(pcimage[emptyImageNum].getName(),0);
	// �摜�����݂��Ă��Ȃ������ꍇ�͐V�K�쐬
	if (pcimage[emptyImageNum].empty())
	{
		pcimage[emptyImageNum] = initImage(img_width,img_height);
	}
	return 0;
}

/*
*�@�T�v�F���̉摜��p�ӂ���
*�@����:
*
*�@�Ԃ�l:
*	�����@0
*	���s�@-1
*/
int PCImage::prepareImage(int X, int Y)
{
	// ���ɗp�ӂ���Ă����珈����Ԃ�
	if (checkPrepare(X, Y)) return 1;

	int emptyImageNum;
	int xy[2];

	pcimage[nowimage].getImageNumber(xy);		//���S�摜��X,Y�ԍ����擾

	emptyImageNum = getEmptyImage();						//�󂢂Ă���摜�̔ԍ����擾
	pcimage[emptyImageNum].setPCI(X, Y);					//�摜��p��
	loadPCImage(emptyImageNum);								//���ɍ쐬����Ă���ꍇ�͓ǂݍ���
	return 0;

}

/*
*�@�T�v�F�󂢂Ă���摜�̔ԍ���Ԃ�
*�@����:
*	�Ȃ�
*�@�Ԃ�l:
*	int	i�@�󂢂Ă���摜�̔ԍ�
*	-1		�󂢂Ă���摜����
*/
int PCImage::getEmptyImage()
{
	for (int i = 0; i < imageNum; i++){
		if (pcimage[i].empty()) return i;
	}
	return -1;
}

/*
*�@�T�v�F���S�摜���w������ɃV�t�g����
*�@����:
*	int x�@�摜�̈���W�ɂ����錻�݉摜�����x�����ψ�(-1~1)
*	int y�@�摜�̈���W�ɂ����錻�݉摜�����y�����ψ�(-1~1)
*�@�Ԃ�l:
*	�Ȃ�
*/
int PCImage::shiftCenterImage(int X, int Y)
{
	cout << "Shift center image" << endl;

	int nowXY[2];
	pcimage[nowimage].getImageNumber(nowXY);	//���݉摜�̍��W���擾

	//�w�肵�����W�̉摜������Ή摜�ԍ���nowimage�ɑ��
	for (int i = 0; i < imageNum; i++)
	{
		if (pcimage[i].isCoordinates(nowXY[0] + X, nowXY[1] + Y))
		{
			nowimage = i;
			return 0;
		}
	}
	return -1;
}

/*
*�@�T�v�F�w�肵���摜�̈���W�̉摜���p�ӂ���Ă���Ɛ^��Ԃ�
*�@����:
*	int X�@�摜�̈���W�ɂ����錻�݉摜�����x�����ψ�(-1~1)
*	int Y�@�摜�̈���W�ɂ����錻�݉摜�����y�����ψ�(-1~1)
*�@�Ԃ�l:
*	True or False
*/
bool PCImage::checkPrepare(int X, int Y)
{
	for (int i = 0; i < imageNum; i++)
	{
		if (pcimage[i].isCoordinates(X, Y)) return true;
	}
	return false;
}

Mat PCImage::initImage(int width, int height)
{
	if (isColor)
		return Mat(Size(width, height), CV_8UC3, Scalar::all(0));
	else
		return Mat(Size(width, height), CV_8U, Scalar::all(0));
}

void PCImage::setColor(BGR bgr)
{
	if (isColor)
	{
		if (bgr & B) color[0] = true;
		else color[0] = false;
		if (bgr & G) color[1] = true;
		else color[1] = false;
		if (bgr & R) color[2] = true;
		else color[2] = false;
	}
}

/*------------------------------
*----��--PCI�N���X�̒�`--��----
*-------------------------------*/
/*
*�@�T�v�F=���Z�q�̃I�[�o�[���[�h
*		 Mat�N���X�̂��̂��Ē�`
*�@����:
*	cv::Mat& mat�@Mat�N���X�̎Q��
*�@�Ԃ�l:
*	*this�@���g�̎Q��
*/
PCImage::PCI& PCImage::PCI::operator=(cv::Mat& mat)
{
	this->Mat::operator=(mat);
	return *this;
}

PCImage::PCI::PCI(PCImage& pcimage_outer) : pciOut(pcimage_outer)
{
	//�O�̂��ߗ̈��������Ă���
	Mat::release();
	name = "";
}

/*
*�@�T�v�F�����o�̏�����
*�@����:
*	int x	�摜�ʒu��x���W
*	int y	�摜�ʒu��y���W
*	PCImage::Direction dir	�摜�̏��(CENTER����݂Ăǂ̕�����)
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::PCI::setPCI(int X, int Y)
{
	imageNumXY[0] = X;
	imageNumXY[1] = Y;
	name = "./" + pciOut.dirname + "/" + to_string(imageNumXY[0]) + "_" + to_string(imageNumXY[1]) + ".jpg";
}

/*
*�@�T�v�F�摜�̈ʒu(x,y)��Ԃ�
*�@����:
*	int xy[]�@x,y���W���i�[����z��
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::PCI::getImageNumber(int xy[])
{
	xy[0] = imageNumXY[0];
	xy[1] = imageNumXY[1];
}

/*
*�@�T�v�F�摜����Ԃ�[./(directoryname)/(filename).jpg]
*�@����:
*	�Ȃ�
*�@�Ԃ�l:
*	string�@name�@�摜��
*/
string PCImage::PCI::getName()
{
	return name;
}

/*
*�@�T�v�F�w����W�ɓ_����������
*�@����:
*	float x_val x���W(m)
*	float y_val y���W(m)
*�@�Ԃ�l:
*	�Ȃ�
*/
int PCImage::PCI::writePoint(float x_val, float y_val)
{
	//x,y�̒l���w�肵���𑜓x�ɍ��킹��
	x_val *= pciOut.coefficient;
	y_val *= pciOut.coefficient;

	//x,y�̒l���摜�̈ʒu�ɍ��킹��
	x_val = x_val - imageNumXY[0] * pciOut.img_width + pciOut.origin_x;
	y_val = y_val - imageNumXY[1] * pciOut.img_height + pciOut.origin_y;

	//���摜�̈���̓_���m�F���ē��摜�̈�O�̏ꍇ�͊Y���̈��ID��Ԃ�
	int x_coord = 0;
	int y_coord = 0;

	checkOverRange(x_val, y_val, x_coord, y_coord);

	if (x_coord || y_coord)
	{
		for (int i = 0; i < imageNum; i++)
		{
			if (pciOut.pcimage[i].isCoordinates(imageNumXY[0] + x_coord, imageNumXY[1] + y_coord)) return i + 1;
		}
		return -1;
	}

	//�擾����[x,y]�̉�f�l�𑝉�������
	write((int)x_val, (int)y_val);

	return 0;
}

/*
*�@�T�v�F�摜��ۑ����ĉ摜�̈�����
*�@����:
*	�Ȃ�
*�@�Ԃ�l:
*	�Ȃ�
*/
void PCImage::PCI::savePCImage()
{
	imwrite(name, *this);
}
void PCImage::PCI::savePCImage(string savename)
{
	imwrite(savename + ".jpg", *this);
}
void PCImage::PCI::release()
{
	imwrite(name, *this);
	this->Mat::release();
	name = "";
	imageNumXY[0] = 10000;
	imageNumXY[1] = 10000;


}

bool PCImage::PCI::isCoordinates(int x, int y)
{
	if (imageNumXY[0] == x && imageNumXY[1] == y) return true;
	return false;
}
bool PCImage::PCI::isCoordinates(int xy[])
{
	if (imageNumXY[0] == xy[0] && imageNumXY[1] == xy[1]) return true;
	return false;
}
void PCImage::PCI::line(cv::Point start, cv::Point end, int color)
{
	int ret[2] = { 0 };
	checkOverRange(start.x, start.y, ret[0], ret[1]);
	if (ret[0] || ret[1]) return;
	checkOverRange(end.x, end.y, ret[0], ret[1]);
	if (ret[0] || ret[1]) return;

	int x = start.x;
	int y = start.y;
	int dx = abs(end.x - start.x);
	int dy = abs(end.y - start.y);
	int sx = (end.x>start.x) ? 1 : -1;
	int sy = (end.y>start.y) ? 1 : -1;

	if (dx >= dy) {
		int err = 2 * dy - dx;
		int i = 0;
		for (i = 0; i <= dx; ++i) {
			if (!data[y * cols + x]){
				data[y * cols + x] = 100;
			}

			x += sx;
			err += 2 * dy;
			if (err >= 0) {
				y += sy;
				err -= 2 * dx;
			}
		}
	}
	else{
		int err = 2 * dx - dy;
		int i = 0;
		for (i = 0; i <= dy; ++i) {
			if (!data[y * cols + x]){
				data[y * cols + x] = 100;
			}

			y += sy;
			err += 2 * dx;
			if (err >= 0) {
				x += sx;
				err -= 2 * dy;
			}
		}
	}
}
void PCImage::PCI::checkOverRange(int x_coord, int y_coord, int& ret_x, int& ret_y)
{
	if (x_coord < 0)
	{
		ret_x = -1;
	}
	else if (x_coord >= cols)
	{
		ret_x = 1;
	}
	if (y_coord < 0)
	{
		ret_y = -1;
	}
	else if (y_coord >= rows)
	{
		ret_y = 1;
	}
}

void PCImage::PCI::write(int x, int y)
{
	if (!pciOut.isColor){
		if (data[y * cols + x] < (pciOut.imgval_increment * (255 / pciOut.imgval_increment))){
			data[y * cols + x] += pciOut.imgval_increment;
		}
		else data[y * cols + x] = 255;
	}
	else
	{
		for (int c = 0; c < this->channels(); c++){
			if (data[y * this->step[0] + x * this->elemSize() + c] < (pciOut.imgval_increment * (255 / pciOut.imgval_increment))){
				data[y * this->step[0] + x * this->elemSize() + c] += pciOut.imgval_increment * pciOut.color[c];
			}
			else data[y * this->step + x * this->elemSize() + c] = 255 * pciOut.color[c];
		}
	}
}
