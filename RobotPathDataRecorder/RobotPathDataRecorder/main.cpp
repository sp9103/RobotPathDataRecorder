#include <stdio.h>
#include <afx.h>
#include <stdlib.h>

#include "ColorBasedTracker.h"
#include "Robot\RobotArm.h"
#include "ARMSDK\include\ARMSDK.h"
#include "KinectMangerThread.h"

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

#define DEFAULT_PATH "data"

bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, ColorBasedTracker *cbTracker, int* angle, char* path, const int count);
void CreateRGBDdir(const char* className);
void writeDepthData(cv::Mat src, char* path, char* name);
void CreateRGBDdir(const char* className);
void ControllerInit(RobotArm *robot);
bool robotConnectCheck(RobotArm *robot, armsdk::RobotInfo *robotinfo, armsdk::Kinematics *kin);

typedef struct robotMotion_{
	int motion[9];
}robotMotion;

int main(){
	RobotArm arm;
	ColorBasedTracker tracker;
	KinectMangerThread kinectManager;
	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;

	//variable
	char dirName[256];
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);
	std::vector<robotMotion> robotVec;

	//initialize
	kinectManager.Initialize(RobotROI);
	int presentSecond = time(NULL);
	itoa(presentSecond, dirName, 10);
	CreateRGBDdir(dirName);
	ControllerInit(&arm);
	if(robotConnectCheck(&arm, &robot, &kin))
		return -1;

	//배경취득
	printf("press any key if window created.\n");
	getch();
	cv::Mat backRGB = kinectManager.getImg();
	cv::Mat backDepth = kinectManager.getDepth();
	cv::imshow("background", backRGB);
	cv::waitKey(1);
	char buf[256];
	strcpy(buf, DEFAULT_PATH);
	strcat(buf, dirName);
	sprintf(buf, "%s\\%s", DEFAULT_PATH, dirName);
	writeDepthData(backDepth, buf, "backDepth");
	strcat(buf, "\\backRGB.bmp");
	cv::imwrite(buf, backRGB);
	tracker.InsertBackGround(backRGB, backDepth);

	while(1){
		printf("if u want store robot pos, press any key\n");
		int keyinput = getch();
		if(keyinput == 'q'){
			printf("exit.\n");
			break;
		}
		arm.TorqueOff();
		robotVec.clear();
		armsdk::Pose3D prevPos;
		memset(&prevPos, 0, sizeof(armsdk::Pose3D));

		//path 저장부
		while(1){
			cv::Mat kinectImg = kinectManager.getImg();
			cv::imshow("kinectImg", kinectImg);
			char key = cv::waitKey(10);

			if(key == 'q'){
				printf("motion save complete!\n");
				break;
			}

			int presAngle[9];
			arm.GetPresPosition(presAngle);
			veci angi(6);
			vecd angd;
			armsdk::Pose3D endEffector;
			angi.resize(6);
			for(int i = 0; i < 6; i++)		angi[i] = presAngle[i];
			angd = kin.Value2Rad(angi);
			kin.Forward(angd, &endEffector);

			float distance = sqrt(pow(endEffector.x - prevPos.x, 2) + pow(endEffector.y - prevPos.y, 2) + pow(endEffector.z - prevPos.z, 2));
			if(distance > 300){							//3cm 이상 차이가 나면 저장
				robotMotion storeMotion;
				for(int i = 0; i < NUM_XEL; i++)	storeMotion.motion[i] = presAngle[i];
				robotVec.push_back(storeMotion);
			}
		}

		printf("robot move start\n");
		//동작부
		while(1){
			cv::Mat kinectImg = kinectManager.getImg();
			cv::Mat KinectDepth = kinectManager.getDepth();
			cv::Mat kinectPC = kinectManager.getPointCloud();


		}
	}

	kinectManager.Deinitialize();

	return 0;
}

void CreateRGBDdir(const char* className){
	TCHAR szDir[MAX_PATH] = {0,};
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR xyzDir[MAX_PATH] = {0,};
	char dirpath[256];
	sprintf(dirpath, "%s\\%s\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
	bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
	sprintf(dirpath, "%s\\%s\\RGB\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\ANGLE\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//Angle
	sprintf(dirpath, "%s\\%s\\DEPTHMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\XYZMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//포인트 클라우드 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\BACKGROUND\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\PROCESSIMG\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);	
}

bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, ColorBasedTracker *cbTracker, int* angle, char* path, const int count){
	cv::Mat processImg = cbTracker->calcImage(RGBimg, DEPTHimg);
	if(processImg.rows == 0)	return false;

	char pathBuf[256], buf[256], id[256];
	sprintf(pathBuf, "%s\\%s", DEFAULT_PATH, path);
	itoa(count, id, 10);

	//store RGB
	sprintf(buf, "%s\\RGB\\%d.bmp", pathBuf, count);
	cv::imwrite(buf, RGBimg);
	//store Depth
	sprintf(buf, "%s\\DEPTHMAP", pathBuf);
	writeDepthData(DEPTHimg, buf, id);
	//store Angle
	sprintf(buf, "%s\\ANGLE\\%d.txt", pathBuf, count);
	FILE *fp = fopen(buf, "w");
	for(int i = 0; i < NUM_XEL; i++)	fprintf(fp, "%d\n", angle[i]);
	fclose(fp);
	//store Process Img
	sprintf(buf, "%s\\PROCESSIMG\\%d.bmp", pathBuf, count);
	cv::imwrite(buf, processImg);
	cv::imshow("Process Img", processImg);
	cv::waitKey(1);
	//store point cloud
	sprintf(buf, "%s\\XYZMAP\\%d.bin", pathBuf, count);
	fp = fopen(buf, "wb");
	fwrite(&pointCloud.rows, sizeof(int), 1, fp);
	fwrite(&pointCloud.cols, sizeof(int), 1, fp);
	int Type = pointCloud.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < pointCloud.rows * pointCloud.cols; i++)
		for(int c = 0; c < pointCloud.channels(); c++)
			fwrite(&pointCloud.at<cv::Vec3f>(i)[c], sizeof(float), 1, fp);
	fclose(fp);

	return true;
}

void writeDepthData(cv::Mat src, char* path, char* name){
	//Depth Infomation write
	char buf[256];
	sprintf(buf, "%s\\%s.bin", path, name);
	FILE *fp = fopen(buf, "wb");
	fwrite(&src.rows, sizeof(int), 1, fp);
	fwrite(&src.cols, sizeof(int), 1, fp);
	int Type = src.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < src.rows * src.cols; i++)		fwrite(&src.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);
}

void ControllerInit(RobotArm *robot){
	int robotid[] = {1,3,5,7,9,11,13,15,17};
	int vel[] = {1000, 1000, 1000, 1000, 1000, 1000, 50, 50, 50};
	//Upper Left, UpperRight, Thumb

	robot->Init(6,3, robotid);

	robot->SetGoalVelocity(vel);
}

bool robotConnectCheck(RobotArm *robot, armsdk::RobotInfo *robotinfo, armsdk::Kinematics *kin){
	veci angi(6);
	robot->Arm_Get_JointValue(&angi);

#ifdef RIGHT_ARM_USE
	//RightArm
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
	robotinfo->AddJoint( 30.0, -ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 5);
	robotinfo->AddJoint(-30.0,  ML_PI_2,    0.0,  ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 7);
	robotinfo->AddJoint(  0.0, -ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 9);
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 11);
#elif defined LEFT_ARM_USE
	//Leftarm
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	robotinfo->AddJoint( 30.0,  ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
	robotinfo->AddJoint(-30.0, -ML_PI_2,    0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
	robotinfo->AddJoint(  0.0,  ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);
#endif
	kin->InitRobot(robotinfo);

	//맥시멈 앵글 체크 - 쓰레기값 걸러내기
	for(int JointNum = 0; JointNum < 6; JointNum++)
	{
		if(abs(angi[JointNum]) > robotinfo->GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
		{
			cout<<"read fail"<<endl;
			printf("Data Fail %d\n", angi[JointNum]);
			return false;
		}
	}
}