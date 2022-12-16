#pragma once

#include <map>
#include <mutex>
#include <thread>
#include "XR/UnityXRTypes.h"

#include "WaveVR_API.h"
#include "WaveVR_Log.h"
#include "WaveVR_Math.h"

#include "ProviderContext.h"

#define LOG_TAG_Hand "WaveVR_Hand"
#define SETTINGS_GETTER_SETTER_DEFINE

using namespace std;
using namespace wvr::core;
using namespace wvr::log;
using namespace wvr::enumerator;
using namespace wvr::utils;

enum class HandTrackingStatus
{
	// Initial, can call Start API in this state.
	NOT_START = 0,
	START_FAILURE,

	// Processing, should NOT call API in this state.
	STARTING,
	STOPING,

	// Running, can call Stop API in this state.
	AVAILABLE,

	// Do nothing.
	UNSUPPORT
};

class WaveVR_Hand : Loggable
{
protected:
	static WaveVR_Hand * Instance;
public:
	static inline WaveVR_Hand * GetInstance() { return Instance; }

public:
	WaveVR_Hand();
	~WaveVR_Hand();

#pragma region Hand Data Update
public:
	void InitHandData();
	void TickHandData(WVR_PoseOriginModel origin);
	void UpdateNaturalJointPose();
	void UpdateElectronicJointPose();

private:
	WVR_PoseOriginModel poseOrigin = WVR_PoseOriginModel_OriginOnHead;
#pragma endregion

#pragma region Hand Tracking Interface
private:
	HandTrackingStatus m_NaturalTrackerStatus = HandTrackingStatus::NOT_START;
	HandTrackingStatus m_ElectronicTrackerStatus = HandTrackingStatus::NOT_START;
	void SetHandTrackingStatus(WVR_HandTrackerType tracker, HandTrackingStatus status);
	bool CanStartHandTracking(WVR_HandTrackerType tracker);
	bool CanStopHandTracking(WVR_HandTrackerType tracker);
	std::mutex m_HandTrackingMutex;
	void StartHandTrackingLock(WVR_HandTrackerType tracker);
	void StartHandTrackingThread(WVR_HandTrackerType tracker);
	void StopHandTrackingLock(WVR_HandTrackerType tracker);
	void StopHandTrackingThread(WVR_HandTrackerType tracker);

public:
	void StartHandTracking(WVR_HandTrackerType tracker);
	void StopHandTracking(WVR_HandTrackerType tracker);
	bool IsHandTrackingAvailable(WVR_HandTrackerType tracker);
	HandTrackingStatus GetHandTrackingStatus(WVR_HandTrackerType tracker);
	float GetHandConfidence(WVR_HandTrackerType tracker, bool isLeft);
	bool IsHandPoseValid(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetHandJointPosition(WVR_HandTrackerType tracker, bool isLeft, WVR_HandJoint joint);
	UnityXRVector4 GetHandJointRotation(WVR_HandTrackerType tracker, bool isLeft, WVR_HandJoint joint);
	WVR_HandPoseType GetHandMotion(WVR_HandTrackerType tracker, bool isLeft);
	float GetHandPinchThreshold(WVR_HandTrackerType tracker);
	float GetHandPinchStrength(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetHandPinchOrigin(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetHandPinchDirection(WVR_HandTrackerType tracker, bool isLeft);
	WVR_HandHoldRoleType GetHandHoldRole(WVR_HandTrackerType tracker, bool isLeft);
	WVR_HandHoldObjectType GetHandHoldType(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetHandScale(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetWristLinearVelocity(WVR_HandTrackerType tracker, bool isLeft);
	UnityXRVector3 GetWristAngularVelocity(WVR_HandTrackerType tracker, bool isLeft);

private:
	UnityXRVector3 BONE_OFFSET_L = { 0, 0, 0 };
	UnityXRVector3 BONE_OFFSET_R = { 0, 0, 0 };

	// Natural
	bool m_EnableNaturalTracker;
	bool hasNaturalTrackerInfo;
	WVR_HandTrackerInfo_t m_NaturalTrackerInfo;

	bool hasNaturalTrackerData;
	WVR_HandTrackingData_t m_NaturalHandTrackerData;
	UnityXRVector3 s_JointPositionNaturalLeft[kJointCount], s_JointPositionNaturalRight[kJointCount];
	UnityXRVector4 s_JointRotationNaturalLeft[kJointCount], s_JointRotationNaturalRight[kJointCount];
	WVR_HandPoseData_t m_NaturalHandPoseData;

	UnityXRVector3 NaturalHandScaleL, NaturalWristLinearVelocityL, NaturalWristAngularVelocityL;
	UnityXRVector3 NaturalHandScaleR, NaturalWristLinearVelocityR, NaturalWristAngularVelocityR;

	// Electronic
	bool m_EnableElectronicTracker;
	bool hasElectronicTrackerInfo;
	WVR_HandTrackerInfo_t m_ElectronicTrackerInfo;

	bool hasElectronicTrackerData;
	WVR_HandTrackingData_t m_ElectronicHandTrackerData;
	UnityXRVector3 s_JointPositionElectronicLeft[kJointCount], s_JointPositionElectronicRight[kJointCount];
	UnityXRVector4 s_JointRotationElectronicLeft[kJointCount], s_JointRotationElectronicRight[kJointCount];
	WVR_HandPoseData_t m_ElectronicHandPoseData;

	UnityXRVector3 ElectronicHandScaleL, ElectronicWristLinearVelocityL, ElectronicWristAngularVelocityL;
	UnityXRVector3 ElectronicHandScaleR, ElectronicWristLinearVelocityR, ElectronicWristAngularVelocityR;
#pragma endregion

private:
	const UnityXRVector3 kZeroVector = {0, 0, 0};
	const UnityXRVector4 kQuaternionIdentity = { 0, 0, 0, 1 };

	int logCount = 0;
	bool printable = false;
};
