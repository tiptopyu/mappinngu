#include "urg_unko.h"

using namespace std;

PCImage urg_mapping::pcimage;


urg_mapping::~urg_mapping()
{

}
void urg_mapping::setWriteLine(bool isLine)
{
	pcimage.isWriteLine = isLine;
}

std::string	urg_mapping::getDirName()
{
	return pcimage.getDirname();
}
void urg_mapping::setPCDDir(std::string dirname)
{
	if (dirname == "") pcd.setDirName(getDirName());
	else pcd.setDirName(dirname);
}

void urg_mapping::initPCImage(PCImage& pci)
{
	pcimage = pci;
}
void urg_mapping::initPCImage(int width, int height, int resolution)
{
	pcimage.initPCImage(width, height, resolution);
}

void urg_mapping::setPCImageColor(PCImage::BGR bgr)
{
	pcimage.setColor(bgr);
}
void urg_mapping::writeMap(float dist, float old, float rad,float spur_x,float spur_y)
{
	getData4URG(dist, old, rad,spur_x,spur_y);
	for (int i = 0; i < data_n; i++){
		if (pointpos[0][i] && pointpos[1][i])
			pcimage.writePoint(pointpos[0][i] / 1000, pointpos[1][i] / 1000, spur_x / 1000, spur_y / 1000);
	}
}
void urg_mapping::setPCImageOrigin(int x, int y)
{
	pcimage.setOrigin(x, y);
}
void urg_mapping::getPCImage(cv::Mat& m, int num)
{
	pcimage.getImage(m, num);
}