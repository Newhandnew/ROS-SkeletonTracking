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

	int shoulderYaw, shoulderPitch, elbowYaw;
	int radian2Degree(double radian, int initialAngle);	// change from radian to degree
	int angleHandler(int inputAngle);	// range limit 0 to 360
	void actionPublish(int shoulderYaw, int shoulderPitch, int elbowYaw);
};


#endif // _NITE_USER_VIEWER_H_
