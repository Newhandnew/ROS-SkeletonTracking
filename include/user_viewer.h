/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include "NiTE.h"
#include <ros/ros.h>

#define MAX_DEPTH 10000

class SampleViewer
{
public:
	SampleViewer(const char* strSampleName);
	virtual ~SampleViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return

protected:
	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

	virtual void OnKey(unsigned char key, int x, int y);

	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();

	void Finalize();

private:
	ros::Publisher movePublisher;

	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;

	struct Spherical {
		double r, radianTheta, radianPhi;
	};
	static const int cmd_lengthTwoArm = 12;
	static const int cmd_type1Angle = 2;
	static const int cmd_type2BothArm = 3;

	static const int rightShoulderYawInit = -60;		// horizontal rise it's -60 degree
	static const int rightShoulderPitchInit = 70;		// when hand's down, it's 70 degree
	static const int rightElbowYawInit = 0;			// elbow initial postion is 0 degree
	static const int leftShoulderYawInit = -60;		// horizontal rise it's -60 degree
	static const int leftShoulderPitchInit = 70;		// when hand's down, it's 70 degree
	static const int leftElbowYawInit = 0;			// elbow initial postion is 0 degree

	static const int moveLimitDegree = 10;

	int rightShoulderYaw, rightShoulderPitch, rightElbowYaw;
	int leftShoulderYaw, leftShoulderPitch, leftElbowYaw;
	Spherical Cartesian2Spherical(int x, int y, int z);
	int radian2Degree(double radian, int initialAngle);	// change from radian to degree
	int angleHandler(int inputAngle);	// range limit 0 to 360
	void actionPublish(int rightShoulderYaw, int rightShoulderPitch, int rightElbowYaw, int leftShoulderYaw, int leftShoulderPitch, int leftElbowYaw);
};


#endif // _NITE_USER_VIEWER_H_
