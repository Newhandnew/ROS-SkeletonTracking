/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "user_viewer.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif

#include <NiteSampleUtilities.h>
//added
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <math.h>
#define PI 3.1415926

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

SampleViewer* SampleViewer::ms_self = NULL;

bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = false;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = false;

// bool g_golf = true;

int g_nXRes = 0, g_nYRes = 0;


// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;

void SampleViewer::glutIdle()
{
	glutPostRedisplay();
}
void SampleViewer::glutDisplay()
{

	SampleViewer::ms_self->Display();
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->OnKey(key, x, y);
}

SampleViewer::SampleViewer(const char* strSampleName) : m_poseUser(0)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
}
SampleViewer::~SampleViewer()
{
	Finalize();

	delete[] m_pTexMap;

	ms_self = NULL;
}

void SampleViewer::Finalize()
{
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status SampleViewer::Init(int argc, char **argv)
{
	// initial ros node
	ros::init(argc, argv, "user_viewer");
	ros::NodeHandle n;
	movePublisher = n.advertise<std_msgs::String>("cmd_erica", 20);
	rightShoulderYaw = 0;
	rightShoulderPitch = 0;
	rightElbowYaw = 0;
	leftShoulderYaw = 0;
	leftShoulderPitch = 0;
	leftElbowYaw = 0;

	m_pTexMap = NULL;

	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}

	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
	{
		return openni::STATUS_ERROR;
	}


	return InitOpenGL(argc, argv);

}
openni::Status SampleViewer::Run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

void updateUserState(const nite::UserData& user, uint64_t ts)
{
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
	}
	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif
void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)g_nXRes;
	y *= GL_WIN_SIZE_Y/(float)g_nYRes;
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}
void DrawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf(buffer, "%d", frameId);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2i(20, 20);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}
void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = {0};

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	glPointSize(8);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

}
void DrawBoundingBox(const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[] =
	{
		user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
		user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
	};
	coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINE_LOOP, 0, 4);

}



void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	float coordinates[6] = {0};
	pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}
void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}


void SampleViewer::Display()
{
	// namespace bg = boost::geometry;	
	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = userTrackerFrame.getDepthFrame();

	if (m_pTexMap == NULL)
	{
		// Texture map init
		m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
	}

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

	if (depthFrame.isValid() && g_drawDepth)
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}

	memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

	float factor[3] = {1, 1, 1};
	// check if we need to draw depth frame to texture
	if (depthFrame.isValid() && g_drawDepth)
	{
		const nite::UserId* pLabels = userLabels.getPixels();

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
			{
				if (*pDepth != 0)
				{
					if (*pLabels == 0)
					{
						if (!g_drawBackground)
						{
							factor[0] = factor[1] = factor[2] = 0;

						}
						else
						{
							factor[0] = Colors[colorCount][0];
							factor[1] = Colors[colorCount][1];
							factor[2] = Colors[colorCount][2];
						}
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					}
//					// Add debug lines - every 10cm
// 					else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

					int nHistValue = m_pDepthHist[*pDepth];
					pTex->r = nHistValue*factor[0];
					pTex->g = nHistValue*factor[1];
					pTex->b = nHistValue*factor[2];

					factor[0] = factor[1] = factor[2] = 1;
				}
			}

			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	g_nXRes = depthFrame.getVideoMode().getResolutionX();
	g_nYRes = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	// 11.16.15 remove for loop for tracking only one user
	// for (int i = 0; i < users.getSize(); ++i)
	// {
		//const nite::UserData& user = users[i];
	if (users.getSize() > 0) {
		const nite::UserData& user = users[0];

		updateUserState(user, userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			m_pUserTracker->startSkeletonTracking(user.getId());
			m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
		}
		else if (!user.isLost())
		{
			if (g_drawStatusLabel)
			{
				DrawStatusLabel(m_pUserTracker, user);
			}
			if (g_drawCenterOfMass)
			{
				DrawCenterOfMass(m_pUserTracker, user);
			}
			if (g_drawBoundingBox)
			{
				DrawBoundingBox(user);
			}

			if (user.getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton)
			{
				DrawSkeleton(m_pUserTracker, user);
			}
		}

		if (m_poseUser == 0 || m_poseUser == user.getId())
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);

			if (pose.isEntered())
			{
				// Start timer
				sprintf(g_generalMessage, "In exit pose. Keep it for %d second%s to exit\n", g_poseTimeoutToExit/1000, g_poseTimeoutToExit/1000 == 1 ? "" : "s");
				printf("Counting down %d second to exit\n", g_poseTimeoutToExit/1000);
				m_poseUser = user.getId();
				m_poseTime = userTrackerFrame.getTimestamp();
			}
			else if (pose.isExited())
			{
				memset(g_generalMessage, 0, sizeof(g_generalMessage));
				printf("Count-down interrupted\n");
				m_poseTime = 0;
				m_poseUser = 0;
			}
			else if (pose.isHeld())
			{
				// tick
				if (userTrackerFrame.getTimestamp() - m_poseTime > g_poseTimeoutToExit * 1000)
				{
					printf("Count down complete. Exit...\n");
					Finalize();
					exit(2);
				}
			}
		} //user

		char buffer[80] = "";
		// ---------------- calculate right shoulder 2 DOF ----------------------
		int rightShoulderX, rightShoulderY, rightShoulderZ;

		rightShoulderX = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().x - 
				user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().x;
		rightShoulderY = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y - 
				user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().y;
		rightShoulderZ = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().z - 
				user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().z;

		Spherical rightShoulderSpherical;
		rightShoulderSpherical = Cartesian2Spherical(rightShoulderX, rightShoulderY, rightShoulderZ);
		int rightShoulderPhi, rightShoulderTheta;
		rightShoulderTheta = radian2Degree(rightShoulderSpherical.radianTheta, rightShoulderThetaInit);	// horizontal rise it's -60 degree
		rightShoulderPhi = radian2Degree(rightShoulderSpherical.radianPhi, rightShoulderPitchInit);			// when hand's down, it's 70 degree
		int rightShoulderYawNow = rightShoulderPhi * sin(rightShoulderTheta * PI / 180);
		int rightShoulderPitchNow = rightShoulderPhi * cos(rightShoulderTheta * PI / 180);

		sprintf(buffer,"(rightShoulderTheta=%d, rightShoulderYaw=%d, rightShoulderPitch=%d)", rightShoulderTheta, rightShoulderYawNow, rightShoulderPitchNow);
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(20, 20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);

		// ------------------- calculate right elbow 1 DOF -----------------------
		int rightElbowX, rightElbowY, rightElbowZ;

		rightElbowX = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x - 
			 user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().x;
		rightElbowY = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y - 
			 user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y;
		rightElbowZ = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().z - 
			 user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().z;

		Spherical rightElbowSpherical;
		rightElbowSpherical = Cartesian2Spherical(rightElbowX, rightElbowY, rightElbowZ);
		int rightElbowTheta = radian2Degree(rightElbowSpherical.radianTheta, 0);
		int rightElbowYawNow = radian2Degree(rightElbowSpherical.radianPhi, rightElbowYawInit);

		sprintf(buffer,"(rightElbowTheta=%d, rightElbowYawNow=%d)", rightElbowTheta, rightElbowYawNow);
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(20, 60);
		glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);

		// ---------------- calculate left shoulder 2 DOF ----------------------
		int leftShoulderX, leftShoulderY, leftShoulderZ;

		leftShoulderX = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().x - 
				user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().x;
		leftShoulderY = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y - 
				user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().y;
		leftShoulderZ = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().z - 
				user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition().z;

		Spherical leftShoulderSpherical;
		leftShoulderSpherical = Cartesian2Spherical(leftShoulderX, leftShoulderY, leftShoulderZ);
		int leftShoulderPhi, leftShoulderTheta;
		leftShoulderTheta = radian2Degree(leftShoulderSpherical.radianTheta, leftShoulderThetaInit);	// horizontal rise it's -60 degree
		leftShoulderPhi = radian2Degree(leftShoulderSpherical.radianPhi, leftShoulderPitchInit);			// when hand's down, it's 70 degree
		// need to reverse in left side
		int leftShoulderYawNow = - leftShoulderPhi * sin(leftShoulderTheta * PI / 180);
		int leftShoulderPitchNow = - leftShoulderPhi * cos(leftShoulderTheta * PI / 180);

		sprintf(buffer,"(leftShoulderTheta=%d, leftShoulderYaw=%d, leftShoulderPitch=%d)", leftShoulderTheta, leftShoulderYawNow, leftShoulderPitchNow);
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(20, 100);
		glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);

		// ------------------- calculate left elbow 1 DOF -----------------------
		int leftElbowX, leftElbowY, leftElbowZ;

		leftElbowX = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x - 
			 user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().x;
		leftElbowY = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y - 
			 user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y;
		leftElbowZ = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z - 
			 user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().z;

		Spherical leftElbowSpherical;
		leftElbowSpherical = Cartesian2Spherical(leftElbowX, leftElbowY, leftElbowZ);
		int leftElbowTheta = radian2Degree(leftElbowSpherical.radianTheta, 0);
		int leftElbowYawNow = radian2Degree(leftElbowSpherical.radianPhi, leftElbowYawInit);

		sprintf(buffer,"(x=%d, y=%d, z=%d, leftElbowTheta=%d, leftElbowYawNow=%d)", leftElbowX, leftElbowY, leftElbowZ, leftElbowTheta, leftElbowYawNow);
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(20, 140);
		glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);

		// ----------------  constraint movement and publish message  ------------
		int rightShoulderYawDiff = abs(rightShoulderYaw - rightShoulderYawNow);
		int rightShoulderPitchDiff = abs(rightShoulderPitch - rightShoulderPitchNow);
		int rightElbowYawDiff = abs(rightElbowYaw - rightElbowYawNow);
		int leftShoulderYawDiff = abs(leftShoulderYaw - leftShoulderYawNow);
		int leftShoulderPitchDiff = abs(leftShoulderPitch - leftShoulderPitchNow);
		int leftElbowYawDiff = abs(leftElbowYaw - leftElbowYawNow);
		if ((rightShoulderYawDiff < moveLimitDegree) && (rightShoulderPitchDiff < moveLimitDegree) && (rightElbowYawDiff < moveLimitDegree) && (rightShoulderTheta >= 0)) {
			rightShoulderYaw = rightShoulderYawNow;
			rightShoulderPitch = rightShoulderPitchNow;
			rightElbowYaw = rightElbowYawNow;
			// change the angle to 0 to 360 and publish
			int rightShoulderYawPub = angleHandler(rightShoulderYaw);
			int rightShoulderPitchPub = angleHandler(rightShoulderPitch);
			int rightElbowYawPub = angleHandler(rightElbowYaw);
			int leftShoulderYawPub = angleHandler(leftShoulderYaw);
			int leftShoulderPitchPub = angleHandler(leftShoulderPitch);
			int leftElbowYawPub = angleHandler(leftElbowYaw);
			sprintf(buffer,"tracking!");
			glColor3f(0.0f, 0.0f, 1.0f);
			glRasterPos2i(20, 180);
			glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
			actionPublish(rightShoulderYawPub, rightShoulderPitchPub, rightElbowYawPub, leftShoulderYawPub, leftShoulderPitchPub, leftElbowYawPub);
		}
		else {
			sprintf(buffer,"soulder yaw: %d, soulder pitch: %d, elbow yaw: %d, rightShoulderTheta > 0: %d", rightShoulderYaw - rightShoulderYawNow, rightShoulderPitch - rightShoulderPitchNow, rightElbowYaw - rightElbowYawNow, rightShoulderTheta);
			glColor3f(1.0f, 0.0f, 0.5f);
			glRasterPos2i(20, 180);
			glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
		}
		if((leftShoulderYawDiff < moveLimitDegree) && (leftShoulderPitchDiff < moveLimitDegree) && (leftElbowYawDiff < moveLimitDegree) && (leftShoulderTheta >= 0)) {
			leftShoulderYaw = leftShoulderYawNow;
			leftShoulderPitch = leftShoulderPitchNow;
			leftElbowYaw = leftElbowYawNow;
			int rightShoulderYawPub = angleHandler(rightShoulderYaw);
			int rightShoulderPitchPub = angleHandler(rightShoulderPitch);
			int rightElbowYawPub = angleHandler(rightElbowYaw);
			int leftShoulderYawPub = angleHandler(leftShoulderYaw);
			int leftShoulderPitchPub = angleHandler(leftShoulderPitch);
			int leftElbowYawPub = angleHandler(leftElbowYaw);
			sprintf(buffer,"tracking!");
			glColor3f(0.0f, 0.3f, 1.0f);
			glRasterPos2i(20, 220);
			glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
			actionPublish(rightShoulderYawPub, rightShoulderPitchPub, rightElbowYawPub, leftShoulderYawPub, leftShoulderPitchPub, leftElbowYawPub);
		}
		else {
			sprintf(buffer,"soulder yaw: %d, soulder pitch: %d, elbow yaw: %d, leftShoulderTheta > 0: %d", leftShoulderYaw - leftShoulderYawNow, leftShoulderPitch - leftShoulderPitchNow, leftElbowYaw - leftElbowYawNow, leftShoulderTheta);
			glColor3f(0.7f, 0.8f, 2.0f);
			glRasterPos2i(20, 220);
			glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
		}


	}

	if (g_drawFrameId)
	{
		DrawFrameId(userTrackerFrame.getFrameIndex());
	}

	if (g_generalMessage[0] != '\0')
	{
		char *msg = g_generalMessage;
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(100, 20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
	}



	// Swap the OpenGL display buffers
	glutSwapBuffers();

}

void SampleViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
	case 's':
		// Draw skeleton?
		g_drawSkeleton = !g_drawSkeleton;
		break;
	case 'l':
		// Draw user status label?
		g_drawStatusLabel = !g_drawStatusLabel;
		break;
	case 'c':
		// Draw center of mass?
		g_drawCenterOfMass = !g_drawCenterOfMass;
		break;
	case 'x':
		// Draw bounding box?
		g_drawBoundingBox = !g_drawBoundingBox;
		break;
	case 'b':
		// Draw background?
		g_drawBackground = !g_drawBackground;
		break;
	case 'd':
		// Draw depth?
		g_drawDepth = !g_drawDepth;
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	}

}

openni::Status SampleViewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;

}

void SampleViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}

SampleViewer::Spherical SampleViewer::Cartesian2Spherical(int x, int y, int z) {
	Spherical result;
	result.r = sqrt((x * x) + (y * y) + (z * z));
	result.radianTheta = acos(z / result.r);
	result.radianPhi = atan2(y, x);
	return result;
};

int SampleViewer::radian2Degree(double radian, int initialAngle) {
	return int((radian / PI) * 180) + initialAngle;
}

int SampleViewer::angleHandler(int inputAngle) {
	if (inputAngle < 0)
		inputAngle += 360;
	else if (inputAngle > 360)
		inputAngle -= 360;
	return inputAngle;
}

void SampleViewer::actionPublish(int rightShoulderYaw, int rightShoulderPitch, int rightElbowYaw, int leftShoulderYaw, int leftShoulderPitch, int leftElbowYaw) {
	std_msgs::String msg;
	char message[30];
	sprintf(message, "%d.%d.%d.%d.%d.0.%d.0.%d.%d.0.%d.0", cmd_lengthTwoArm, cmd_type1Angle, cmd_type2BothArm, rightShoulderYaw, rightShoulderPitch, rightElbowYaw, leftShoulderYaw, leftShoulderPitch, leftElbowYaw);
  	msg.data = message;//ss.str();
		movePublisher.publish(msg);
}
