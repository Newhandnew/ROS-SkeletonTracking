/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - Simple Skeleton Sample                               *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "NiTE.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <NiteSampleUtilities.h>
#include <boost/geometry.hpp>

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

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

int main(int argc, char** argv)
{
	nite::UserTracker userTracker;
	nite::Status niteRc;

	nite::NiTE::initialize();

	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;
	while (!wasKeyboardHit())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK)
		{
			printf("Get next frame failed\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user,userTrackerFrame.getTimestamp());
			if (user.isNew())
			{
				userTracker.startSkeletonTracking(user.getId());
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
				int vec_x,vec_y,vec_z;
		//		pos_x = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x;
		//		pos_y = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y;
		//		pos_z = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().z;

				vec_x = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().x - 
						user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().x;
				vec_y = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y - 
						user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().y;
				vec_z = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().z - 
						user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition().z;
				// printf("vec_x:%d, vec_y:%d, vec_z:%d\n", vec_x, vec_y, vec_z);

				boost::geometry::model::point<int, 3, boost::geometry::cs::cartesian> v1(vec_x, vec_y, vec_z);
				boost::geometry::model::point<double, 3, boost::geometry::cs::spherical<boost::geometry::degree> > v2;
				boost::geometry::transform(v1, v2);
				printf("(theta=%5f, phi=%5f, r=%5f)\n", v2.get<0>(), v2.get<1>(), v2.get<2>());
				// if (head.getPositionConfidence() > .5)
				//printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
			}
		}

	}

	nite::NiTE::shutdown();

}
