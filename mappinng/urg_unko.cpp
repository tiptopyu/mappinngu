#define _USE_MATH_DEFINES

#include "urg_unko.h"
#include "open_urg_sensor.c"

using namespace std;

/*
*	概要:
*		コンストラクタ
*		pcimage，COMport，pcdnumを初期化する
*	引数：
*		なし
*	返り値:
*		なし
*/
urg_unko::urg_unko() 
{
	COMport = 0;
}

/*
*	概要:
*		オブジェクトの初期化処理
*	引数：
*		int COM		URGのCOMポート番号
*		float pos[]	NCWCの回転中心から見たURGの位置
*	返り値:
*		なし
*/
void urg_unko::init(int COM, float pos[])
{
	// 引数のCOMをメンバのCOMportに格納
	COMport = COM;

	//指定されたCOMポートのURGと接続
	connectURG();

	//以下，メンバの初期化
	for (int i = 0; i < 4; i++)
	{
		urgpos[i] = pos[i];
	}
	for (int i = 0; i < sizeof(pointpos); i++)
	{
		this->pointpos[i] = NULL;
	}
}

void urg_unko::init3D(int COM, float pos[])
{
	// 引数のCOMをメンバのCOMportに格納
	COMport3D = COM;

	//指定されたCOMポートのURGと接続
	connectURG();

	//以下，メンバの初期化
	for (int i = 0; i < 4; i++)
	{
		urgpos3D[i] = pos[i];
	}
	for (int i = 0; i < sizeof(pointpos); i++)
	{
		this->pointpos[i] = NULL;
	}
}

/*
*	概要:
*		URGの接続を切断
*	引数：
*		なし
*	返り値:
*		0
*/
int urg_unko::disconnectURG(){

	//切断
	delete data;
	urg_close(&urg);
	data = NULL;

	printf("URG disconnected \n");
	return 0;

}

/*
*	概要:
*		デストラクタ
*		URGの切断，画像の保存を行う
*	引数：
*		なし
*	返り値:
*		なし
*/
urg_unko::~urg_unko()
{
	disconnectURG();
}

/*
*	概要:
*		URGと接続する
*		init(int COM , float pos[])で指定したURGと接続する
*	引数：
*		なし
*	返り値:
*		0
*/
int urg_unko::connectURG(){

	//urgオブジェクトの取得
	if (open_urg_sensor(&urg, COMport) < 0) {
		return 1;
	}

	//データ取得用のメモリを確保
	//data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
	data = new long[urg_max_data_size(&urg)];
	if (!data) {
		perror("urg_max_index()");
		return 1;
	}

	printf("URG connected : urg_max_data_size =  %d \n", urg_max_data_size(&urg));
	return 0;
}

/*
*	概要:
*		URGからデータを取得する
*	引数：
*		float& dist	積算する距離データ用変数の参照
*		float& rad	積算する回転角データ用変数の参照
*	返り値:
*		0
*/
int urg_unko::getData4URG(float dist, float old, float rad, float spur_x, float spur_y){
	//データ取得
#if 1
	//データの取得範囲を変更する場合
	urg_set_scanning_parameter(&urg,
		urg_deg2step(&urg, -100),
		urg_deg2step(&urg, +100), 0);
#endif

	//積算した距離,回転角を格納
	distance_old = old;
	distance = dist;
	rob_x = spur_x;
	rob_y = spur_y;
	radian = rad;

	//測定の開始
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);

	for (int i = 0; i < CAPTURE_TIMES; ++i) {
		//測定データの取得
		data_n = urg_get_distance(&urg, data, &time_stamp);
		
		if (data_n <= 0) {
			printf("urg_get_distance: %s\n", urg_error(&urg));
			delete data;
			data = NULL;
			urg_close(&urg);
			return 1;
		}

		//測定データからマップ，pcdファイルを作成
		calcSurface2D();
	}

	return 0;

}

/*
*	概要:
*		取得したデータから実際の二次元情報を計算してマップ，pcdファイルへの書き込みを行う
*	引数：
*		int data_n	取得したデータの数
*	返り値:
*		なし
*/
void urg_unko::calcSurface2D()
{
	(void)time_stamp;
	int count = 0;
	long min_distance;
	long max_distance;

	if (abs(distance - distance_old) >= scaninterval){//位置が設定値以上動いたとき

		// 全てのデータの X-Y の位置を取得
		//正常に取得されるデータの最大値，最小値を取得
		urg_distance_min_max(&urg, &min_distance, &max_distance);

		float droidOrientation[3] = {};
		float droidGPS[3] = {};
		//rcvDroid.getOrientationData(droidOrientation);
		//rcvDroid.getGPSData(droidGPS);

		for (int i = 0; i < 2; i++)
		{
			if (this->pointpos[i] != NULL)	delete[] this->pointpos[i];
			this->pointpos[i] = new float[data_n];
			if (this->rawpointpos[i] != NULL)	delete[] this->rawpointpos[i];
			this->rawpointpos[i] = new float[data_n];
		}
		//データの数だけ実際の座標を計算してマップ，pcdファイルに書き込む
		for (int i = 0; i < data_n; ++i) {
			long l = data[i];	//取得した点までの距離
			double radian;
			float x, y, z;
			float pointpos[3];

			//異常値ならとばす
			if ((l <= min_distance) || (l >= max_distance)) {
				this->pointpos[0][i] = 0;
				this->pointpos[1][i] = 0;
				this->rawpointpos[0][i] = 0;
				this->rawpointpos[1][i] = 0;
				continue;
			}

			//点までの角度を取得してxyに変換
			//(極座標で取得されるデータをデカルト座標に変換)
			radian = urg_index2rad(&urg, i);
			
			// センサ平面時の取得座標
			/*x = (float)(l * cos(radian));
			y = (float)(l * sin(radian));
			z = urgpos[0];*/

			// センサ空間時の取得座標
			//x:進行方向
			x = rawpointpos[0][i] = (float)(l * cos(radian + urgpos[3]));
			//y:進行方向に対して横方向
			y = rawpointpos[1][i] = (float)(l * sin(radian + urgpos[3]));

			// 2次元平面の座標変換
			//pointpos[0] = +cos(this->radian) * x + sin(this->radian) * y + cos(this->radian) * (distance - distance_old + urgpos[1]) + currentCoord_x;
			//pointpos[1] = -sin(this->radian) * x + cos(this->radian) * y - sin(this->radian) * (distance - distance_old + urgpos[1]) + currentCoord_y;

			// 2次元平面の座標変換
			this->pointpos[0][i] = +cos(this->radian + urgpos[3]) * x + sin(this->radian + urgpos[3]) * y + cos(this->radian) * (distance - distance_old + urgpos[1]) + sin(this->radian) * urgpos[2] + rob_x;
			this->pointpos[1][i] = -sin(this->radian + urgpos[3]) * x + cos(this->radian + urgpos[3]) * y - sin(this->radian) * (distance - distance_old + urgpos[1]) + cos(this->radian) * urgpos[2] + rob_y;

			// 3次元空間の座標変換
			//this->pointpos[0][i] = -sin(this->radian) * y + cos(this->radian) * ((distance - distance_old) * i / data_n) - sin(this->radian) * urgpos[1] + sin(this->radian) * urgpos[2] + currentCoord_x;
			//this->pointpos[1][i] = +cos(this->radian) * y + sin(this->radian) * ((distance - distance_old) * i / data_n) + cos(this->radian) * urgpos[1] + cos(this->radian) * urgpos[2] + currentCoord_y;
			//this->pointpos[2][i] = x + urgpos[0];

		}
	}
}
void urg_unko::calcSurface3D()
{
	(void)time_stamp;
	int count = 0;
	long min_distance;
	long max_distance;

	if (abs(distance - distance_old) >= scaninterval){//位置が設定値以上動いたとき

		// 全てのデータの X-Y の位置を取得
		//正常に取得されるデータの最大値，最小値を取得
		urg_distance_min_max(&urg, &min_distance, &max_distance);

		float droidOrientation[3] = {};
		float droidGPS[3] = {};
		//rcvDroid.getOrientationData(droidOrientation);
		//rcvDroid.getGPSData(droidGPS);

		for (int i = 0; i < 2; i++)
		{
			if (this->pointpos[i] != NULL)	delete[] this->pointpos[i];
			this->pointpos[i] = new float[data_n];
			if (this->rawpointpos[i] != NULL)	delete[] this->rawpointpos[i];
			this->rawpointpos[i] = new float[data_n];
		}
		//データの数だけ実際の座標を計算してマップ，pcdファイルに書き込む
		for (int i = 0; i < data_n; ++i) {
			long l = data[i];	//取得した点までの距離
			double radian;
			float x, y, z;
			float pointpos[3];

			//異常値ならとばす
			if ((l <= min_distance) || (l >= max_distance)) {
				this->pointpos[0][i] = 0;
				this->pointpos[1][i] = 0;
				this->rawpointpos[0][i] = 0;
				this->rawpointpos[1][i] = 0;
				continue;
			}

			//点までの角度を取得してxyに変換
			//(極座標で取得されるデータをデカルト座標に変換)
			radian = urg_index2rad(&urg, i);

			// センサ平面時の取得座標
			/*x = (float)(l * cos(radian));
			y = (float)(l * sin(radian));
			z = urgpos[0];*/

			// センサ空間時の取得座標
			//x:進行方向
			x = rawpointpos[0][i] = (float)(l * cos(radian + urgpos[3]));
			//y:進行方向に対して横方向
			y = rawpointpos[1][i] = (float)(l * sin(radian + urgpos[3]));

			// 2次元平面の座標変換
			//pointpos[0] = +cos(this->radian) * x + sin(this->radian) * y + cos(this->radian) * (distance - distance_old + urgpos[1]) + currentCoord_x;
			//pointpos[1] = -sin(this->radian) * x + cos(this->radian) * y - sin(this->radian) * (distance - distance_old + urgpos[1]) + currentCoord_y;

			// 2次元平面の座標変換
			this->pointpos[0][i] = +cos(this->radian + urgpos[3]) * x + sin(this->radian + urgpos[3]) * y + cos(this->radian) * (distance - distance_old + urgpos[1]) + sin(this->radian) * urgpos[2] + rob_x;
			this->pointpos[1][i] = -sin(this->radian + urgpos[3]) * x + cos(this->radian + urgpos[3]) * y - sin(this->radian) * (distance - distance_old + urgpos[1]) + cos(this->radian) * urgpos[2] + rob_y;

			// 3次元空間の座標変換
			//this->pointpos[0][i] = -sin(this->radian) * y + cos(this->radian) * ((distance - distance_old) * i / data_n) - sin(this->radian) * urgpos[1] + sin(this->radian) * urgpos[2] + currentCoord_x;
			//this->pointpos[1][i] = +cos(this->radian) * y + sin(this->radian) * ((distance - distance_old) * i / data_n) + cos(this->radian) * urgpos[1] + cos(this->radian) * urgpos[2] + currentCoord_y;
			//this->pointpos[2][i] = x + urgpos[0];

		}
	}
}

void urg_unko::updateCurrentCoord(float coord_x, float coord_y)//使用していない
{
	currentCoord_x = coord_x;
	currentCoord_y = coord_y;
}
void urg_unko::updateCurrentCoord(float coordXY[])
{
	currentCoord_x = coordXY[0];
	currentCoord_y = coordXY[1];
}

void urg_unko::savePCD()
{
	pcd.pcdinit();
	for (int i = 0; i < data_n; i++)
	{
		pcd.pcdWrite(pointpos[0][i] / 1000, pointpos[1][i] / 1000, currentCoord_x / 1000, currentCoord_y / 1000);
		//pcd.pcdWrite(pointpos[0][i] / 1000, pointpos[1][i] / 1000, pointpos[2][i] / 1000);
	}
	pcd.pcdSave();
}
void urg_unko::saveRawPCD(float dist , float rad)
{
	pcd.pcdinit();
	for (int i = 0; i < data_n; i++)
	{
		pcd.pcdWrite(rawpointpos[0][i] / 1000, rawpointpos[1][i] / 1000, currentCoord_x / 1000, currentCoord_y / 1000,dist,rad);
	}
	pcd.pcdSave();
}
void urg_unko::saveRawPCD3D(float dist, float rad)
{
	pcd.pcdinit3D();
	for (int i = 0; i < data_n; i++)
	{
		pcd.pcdWrite3D(rawpointpos[0][i] / 1000, rawpointpos[1][i] / 1000, currentCoord_x / 1000, currentCoord_y / 1000, dist, rad);
	}
	pcd.pcdSave();
}
int urg_unko::getData_n()
{
	return data_n;
}
void urg_unko::getData(float data[] , int data_n, int offset)
{
	for (int i = offset; i < data_n; i++)
	{
		data[i] = this->data[i];
	}
}
writePCD urg_unko::pcd;

int writePCD::pcdnum = 0;

writePCD::writePCD(std::string dirName)
{
	isWritePCD = true;
	this->dirname = dirName;
}
writePCD::~writePCD()
{
}

/*
*	概要:
*		pcdファイルを作成して初期化する
*	引数：
*		なし
*	返り値:
*		なし
*/
void writePCD::pcdinit()
{
	if (!isWritePCD) return;

	//ファイル名を指定してファイルストリームを開く
	if(dirname != "") this->open("./" + dirname + "/pointcloud_" + std::to_string(pcdnum) + ".pcd",std::ios::out);
	else this->open("./pointcloud_" + std::to_string(pcdnum) + ".pcd", std::ios::out);

	//pcdファイル番号を進めてデータ数カウント用変数を初期化
	pcdnum++;
	pcdcount = 0;

	//ヘッダを記入
	*this << "# .PCD v.7 - Point Cloud Data file format\n"
		<< "VERSION .7\n"
		<< "FIELDS x y z\n"
		<< "SIZE 4 4 4\n"
		<< "TYPE F F F\n"
		<< "COUNT 1 1 1\n"
		<< "WIDTH 400\n"
		<< "HEIGHT 1\n"
		<< "VIEWPOINT 0 0 0 1 0 0 0\n"
		<< "POINTS 400\n"
		<< "DATA ascii" << endl;
}
void writePCD::pcdinit3D()
{
	if (!isWritePCD) return;

	//ファイル名を指定してファイルストリームを開く
	if (dirname != "") this->open("./" + dirname + "3D/pointcloud_" + std::to_string(pcdnum) + ".pcd", std::ios::out);
	else this->open("./pointcloud_" + std::to_string(pcdnum) + ".pcd", std::ios::out);

	//pcdファイル番号を進めてデータ数カウント用変数を初期化
	pcdnum++;
	pcdcount = 0;

	//ヘッダを記入
	*this << "# .PCD v.7 - Point Cloud Data file format\n"
		<< "VERSION .7\n"
		<< "FIELDS x y z\n"
		<< "SIZE 4 4 4\n"
		<< "TYPE F F F\n"
		<< "COUNT 1 1 1\n"
		<< "WIDTH 400\n"
		<< "HEIGHT 1\n"
		<< "VIEWPOINT 0 0 0 1 0 0 0\n"
		<< "POINTS 400\n"
		<< "DATA ascii" << endl;
}

/*
*	概要:
*		pcdファイルにデータを書き込む
*	引数：
*		float x	x座標値
*		float y	y座標値
*	返り値:
*		なし
*/
void writePCD::pcdWrite(float x, float y)
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << " " << y << " " << "0.0" << endl;
	pcdcount++;
}

void writePCD::pcdWrite(float x, float y, float pos_x, float pos_y, float droidAngle[], float droidGPS[])
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << ", " << y << ", " << pos_x << ", " << pos_y << ", " << droidAngle[0] << ", " << droidAngle[1] << ", " << droidAngle[2] << ", " << droidGPS[0] << ", " << droidGPS[1] << ", " << droidGPS[2] << ", " << endl;
	pcdcount++;
}
void writePCD::pcdWrite(float x, float y, float pos_x, float pos_y)
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << ", " << y << ", " << pos_x << ", " << pos_y << ", " << endl;
	pcdcount++;
}
void writePCD::pcdWrite(float x, float y, float pos_x, float pos_y,float dist,float rad)
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << ", " << y << ", " << pos_x << ", " << pos_y << ", " << dist << ", " << rad << ", " << endl;
	pcdcount++;
}
void writePCD::pcdWrite3D(float x, float y, float pos_x, float pos_y, float dist, float rad)
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << ", " << y << ", " << pos_x << ", " << pos_y << ", " << dist << ", " << rad << ", " << endl;
	pcdcount++;
}
void writePCD::pcdWrite(float x, float y, float z)
{
	if (!isWritePCD) return;

	//データを書き込んでデータ数をカウント
	*this << x << ", " << y << ", " << z << endl;
	pcdcount++;
}
/*
*	概要:
*		ファイルストリームを閉じて保存する
*	引数：
*		なし
*	返り値:
*		なし
*/
void writePCD::pcdSave()
{
	if (!isWritePCD) return;

	//最終的なデータ数を追記
	this->seekp(0, ios_base::beg);

	*this << "# .PCD v.7 - Point Cloud Data file format\n"
		<< "VERSION .7\n"
		<< "FIELDS x y z\n"
		<< "SIZE 4 4 4\n"
		<< "TYPE F F F\n"
		<< "COUNT 1 1 1\n"
		<< "WIDTH " + std::to_string(pcdcount) + "\n"
		<< "HEIGHT 1\n"
		<< "VIEWPOINT 0 0 0 1 0 0 0\n"
		<< "POINTS " + std::to_string(pcdcount) + "\n"
		<< "DATA ascii" << endl;

	//ファイルストリームを緘
	this->close();
	this->flush();
}
void writePCD::setDirName(std::string dirname)
{
	this->dirname = dirname;
}