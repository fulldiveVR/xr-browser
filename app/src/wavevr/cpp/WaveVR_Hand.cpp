// ++ LICENSE-HIDDEN SOURCE ++

#include "WaveVR_Hand.h"

WaveVR_Hand * WaveVR_Hand::Instance = nullptr;

WaveVR_Hand::WaveVR_Hand()
	: Loggable(LOG_TAG_Hand)
{
	Instance = this;
}

WaveVR_Hand::~WaveVR_Hand()
{
	Instance = nullptr;
}

#pragma region Hand Data Update
void WaveVR_Hand::InitHandData()
{
	m_EnableNaturalTracker = false;
	hasNaturalTrackerInfo = false;
	hasNaturalTrackerData = false;

	m_EnableElectronicTracker = false;
	hasElectronicTrackerInfo = false;
	hasElectronicTrackerData = false;

	m_NaturalTrackerInfo.jointMappingArray = nullptr;
	m_NaturalTrackerInfo.jointValidFlagArray = nullptr;
	m_NaturalTrackerInfo.pinchTHR = 0;
	m_NaturalHandTrackerData.left.joints = nullptr;
	m_NaturalHandTrackerData.right.joints = nullptr;

	m_ElectronicTrackerInfo.jointMappingArray = nullptr;
	m_ElectronicTrackerInfo.jointValidFlagArray = nullptr;
	m_ElectronicTrackerInfo.pinchTHR = 0;
	m_ElectronicHandTrackerData.left.joints = nullptr;
	m_ElectronicHandTrackerData.right.joints = nullptr;

	for (int i = 0; i < kJointCount; i++)
	{
		s_JointPositionNaturalLeft[i] = { 0, 0, 0 };
		s_JointPositionNaturalRight[i] = { 0, 0, 0 };
		s_JointRotationNaturalLeft[i] = { 0, 0, 0, 1 };
		s_JointRotationNaturalRight[i] = { 0, 0, 0, 1 };

		s_JointPositionElectronicLeft[i] = { 0, 0, 0 };
		s_JointPositionElectronicRight[i] = { 0, 0, 0 };
		s_JointRotationElectronicLeft[i] = { 0, 0, 0, 1 };
		s_JointRotationElectronicRight[i] = { 0, 0, 0, 1 };
	}

	LOGD(DLF_INPUT_1, "InitHandData()");
}
void WaveVR_Hand::TickHandData(WVR_PoseOriginModel origin)
{
	logCount++;
	logCount %= 500;
	printable = (logCount == 0);

	poseOrigin = origin;

	/* ----------------------- Hand Tracking -----------------------*/
	HandTrackingStatus natural_tracker_status = GetHandTrackingStatus(WVR_HandTrackerType_Natural);
	if (natural_tracker_status == HandTrackingStatus::AVAILABLE)
	{
		// Calls GetHandJointCount one time after starting tracker.
		if (!hasNaturalTrackerInfo)
		{
			WVR_Result result = WVR().GetHandJointCount(WVR_HandTrackerType::WVR_HandTrackerType_Natural, &m_NaturalTrackerInfo.jointCount);
			if (result == WVR_Result::WVR_Success)
			{
				LOGD(DLF_INPUT_1, "TickHandData() Natural tracker, get joint count %d", m_NaturalTrackerInfo.jointCount);

				/// Initialize m_NaturalTrackerInfo
				m_NaturalTrackerInfo.handModelTypeBitMask = 0;
				if (m_NaturalTrackerInfo.jointMappingArray != nullptr)
				{
					delete[] m_NaturalTrackerInfo.jointMappingArray;
					m_NaturalTrackerInfo.jointMappingArray = nullptr;

					LOGD(DLF_INPUT_1, "TickHandData() Release m_NaturalTrackerInfo.jointMappingArray.");
				}
				m_NaturalTrackerInfo.jointMappingArray = new WVR_HandJoint[m_NaturalTrackerInfo.jointCount];
				LOGD(DLF_INPUT_1, "TickHandData() m_NaturalTrackerInfo.jointMappingArray %p", m_NaturalTrackerInfo.jointMappingArray);

				if (m_NaturalTrackerInfo.jointValidFlagArray != nullptr)
				{
					delete[] m_NaturalTrackerInfo.jointValidFlagArray;
					m_NaturalTrackerInfo.jointValidFlagArray = nullptr;

					LOGD(DLF_INPUT_1, "TickHandData() Release m_NaturalTrackerInfo.jointValidFlagArray.");
				}
				m_NaturalTrackerInfo.jointValidFlagArray = new uint64_t[m_NaturalTrackerInfo.jointCount];
				LOGD(DLF_INPUT_1, "TickHandData() m_NaturalTrackerInfo.jointValidFlagArray %p", m_NaturalTrackerInfo.jointValidFlagArray);

				m_NaturalTrackerInfo.pinchTHR = 0;

				hasNaturalTrackerInfo =
					(WVR().GetHandTrackerInfo(WVR_HandTrackerType::WVR_HandTrackerType_Natural, &m_NaturalTrackerInfo) == WVR_Result::WVR_Success ? true : false);

				if (hasNaturalTrackerInfo)
				{
					LOGD(DLF_INPUT_1, "TickHandData() Natural tracker, joint %d, pinchTHR %f",
						m_NaturalTrackerInfo.jointCount,
						m_NaturalTrackerInfo.pinchTHR);

					for (int32_t i = 0; i < (int32_t)m_NaturalTrackerInfo.jointCount; i++)
					{
						LOGD(DLF_INPUT_1, "TickHandData() Natural tracker, jointMappingArray[%d] = %d, jointValidFlagArray[%d] = %d",
							i, (uint32_t)m_NaturalTrackerInfo.jointMappingArray[i],
							i, (uint32_t)m_NaturalTrackerInfo.jointValidFlagArray[i]);
					}

					/// Initialize m_NaturalHandTrackerData
					m_NaturalHandTrackerData.timestamp = 0;
					m_NaturalHandTrackerData.left.confidence = 0;
					m_NaturalHandTrackerData.left.isValidPose = false;
					m_NaturalHandTrackerData.left.jointCount = m_NaturalTrackerInfo.jointCount;
					if (m_NaturalHandTrackerData.left.joints != nullptr)
					{
						delete[]m_NaturalHandTrackerData.left.joints;
						m_NaturalHandTrackerData.left.joints = nullptr;

						LOGD(DLF_INPUT_1, "TickHandData() Release m_NaturalHandTrackerData.left.joints.");
					}
					m_NaturalHandTrackerData.left.joints = new WVR_Pose_t[m_NaturalTrackerInfo.jointCount];
					LOGD(DLF_INPUT_1, "TickHandData() m_NaturalHandTrackerData.left.joints %p", m_NaturalHandTrackerData.left.joints);
					m_NaturalHandTrackerData.left.scale.v[0] = 0;
					m_NaturalHandTrackerData.left.scale.v[1] = 0;
					m_NaturalHandTrackerData.left.scale.v[2] = 0;

					m_NaturalHandTrackerData.right.confidence = 0;
					m_NaturalHandTrackerData.right.isValidPose = false;
					m_NaturalHandTrackerData.right.jointCount = m_NaturalTrackerInfo.jointCount;
					if (m_NaturalHandTrackerData.right.joints != nullptr)
					{
						delete[] m_NaturalHandTrackerData.right.joints;
						m_NaturalHandTrackerData.right.joints = nullptr;

						LOGD(DLF_INPUT_1, "TickHandData() Release m_NaturalHandTrackerData.right.joints.");
					}
					m_NaturalHandTrackerData.right.joints = new WVR_Pose_t[m_NaturalTrackerInfo.jointCount];
					LOGD(DLF_INPUT_1, "TickHandData() m_NaturalHandTrackerData.right.joints %p", m_NaturalHandTrackerData.right.joints);
					m_NaturalHandTrackerData.right.scale.v[0] = 0;
					m_NaturalHandTrackerData.right.scale.v[1] = 0;
					m_NaturalHandTrackerData.right.scale.v[2] = 0;


					/// Initialize m_NaturalHandPoseData
					m_NaturalHandPoseData.timestamp = 0;
					m_NaturalHandPoseData.left.base.type = WVR_HandPoseType::WVR_HandPoseType_Invalid;
					m_NaturalHandPoseData.right.base.type = WVR_HandPoseType::WVR_HandPoseType_Invalid;
				}
			}
		} // if (!hasNaturalTrackerInfo)

		// Calls GetHandTrackingData on each frame.
		if (hasNaturalTrackerInfo &&
			((m_NaturalTrackerInfo.handModelTypeBitMask & (uint64_t)WVR_HandModelType::WVR_HandModelType_WithoutController) != 0))
		{
			hasNaturalTrackerData = (
				WVR().GetHandTrackingData(
					WVR_HandTrackerType::WVR_HandTrackerType_Natural,
					WVR_HandModelType::WVR_HandModelType_WithoutController,
					poseOrigin,
					&m_NaturalHandTrackerData,
					&m_NaturalHandPoseData
				) == WVR_Result::WVR_Success ? true : false);

			if (hasNaturalTrackerData)
			{
				UpdateNaturalJointPose();
				auto wvrScaleL = m_NaturalHandTrackerData.left.scale;
				auto wvrScaleR = m_NaturalHandTrackerData.right.scale;
				NaturalHandScaleL.x = wvrScaleL.v[0];
				NaturalHandScaleL.y = wvrScaleL.v[1];
				NaturalHandScaleL.z = wvrScaleL.v[2];
				NaturalHandScaleR.x = wvrScaleR.v[0];
				NaturalHandScaleR.y = wvrScaleR.v[1];
				NaturalHandScaleR.z = wvrScaleR.v[2];

				NaturalWristLinearVelocityL = WVRVector3ToUnityVector3(m_NaturalHandTrackerData.left.wristLinearVelocity);
				NaturalWristLinearVelocityR = WVRVector3ToUnityVector3(m_NaturalHandTrackerData.right.wristLinearVelocity);

				NaturalWristAngularVelocityL = WVRVector3ToUnityVector3(m_NaturalHandTrackerData.left.wristAngularVelocity);
				NaturalWristAngularVelocityR = WVRVector3ToUnityVector3(m_NaturalHandTrackerData.right.wristAngularVelocity);

				if (printable)
				{
				LOGD(DLF_INPUT_1, "TickHandData() Natural left valid %d, right valid %d",
					(uint8_t)m_NaturalHandTrackerData.left.isValidPose,
					(uint8_t)m_NaturalHandTrackerData.right.isValidPose);
				LOGD(DLF_INPUT_1, "TickHandData() Natural left scale (%f, %f, %f), right scale (%f, %f, %f)",
					NaturalHandScaleL.x, NaturalHandScaleL.y, NaturalHandScaleL.z,
					NaturalHandScaleR.x, NaturalHandScaleR.y, NaturalHandScaleR.z);
				LOGD(DLF_INPUT_1, "TickHandData() Natural left linear velocity (%f, %f, %f), right linear velocity (%f, %f, %f)",
					NaturalWristLinearVelocityL.x, NaturalWristLinearVelocityL.y, NaturalWristLinearVelocityL.z,
					NaturalWristLinearVelocityR.x, NaturalWristLinearVelocityR.y, NaturalWristLinearVelocityR.z);
				LOGD(DLF_INPUT_1, "TickHandData() Natural left angular velocity (%f, %f, %f), right angular velocity (%f, %f, %f)",
					NaturalWristAngularVelocityL.x, NaturalWristAngularVelocityL.y, NaturalWristAngularVelocityL.z,
					NaturalWristAngularVelocityR.x, NaturalWristAngularVelocityR.y, NaturalWristAngularVelocityR.z);
				}
			}
		}
		else
		{
			hasNaturalTrackerData = false;
		}
	}

	HandTrackingStatus electronic_tracker_status = GetHandTrackingStatus(WVR_HandTrackerType_Electronic);
	if (electronic_tracker_status == HandTrackingStatus::AVAILABLE)
	{
		// Calls GetHandJointCount one time after starting tracker.
		if (!hasElectronicTrackerInfo)
		{
			WVR_Result result = WVR().GetHandJointCount(WVR_HandTrackerType::WVR_HandTrackerType_Electronic, &m_ElectronicTrackerInfo.jointCount);
			if (result == WVR_Result::WVR_Success)
			{
				LOGD(DLF_INPUT_1, "TickHandData() Electronic tracker, get joint count %d", m_ElectronicTrackerInfo.jointCount);

				// Calls GetHandTrackerInfo one time after starting tracker.
				m_ElectronicTrackerInfo.handModelTypeBitMask = 0;
				if (m_ElectronicTrackerInfo.jointMappingArray != nullptr)
				{
					delete[] m_ElectronicTrackerInfo.jointMappingArray;
					m_ElectronicTrackerInfo.jointMappingArray = nullptr;

					LOGD(DLF_INPUT_1, "TickHandData() Release m_ElectronicTrackerInfo.jointMappingArray.");
				}
				m_ElectronicTrackerInfo.jointMappingArray = new WVR_HandJoint[m_ElectronicTrackerInfo.jointCount];

				if (m_ElectronicTrackerInfo.jointValidFlagArray != nullptr)
				{
					delete[] m_ElectronicTrackerInfo.jointValidFlagArray;
					m_ElectronicTrackerInfo.jointValidFlagArray = nullptr;

					LOGD(DLF_INPUT_1, "TickHandData() Release m_ElectronicTrackerInfo.jointValidFlagArray.");
				}
				m_ElectronicTrackerInfo.jointValidFlagArray = new uint64_t[m_ElectronicTrackerInfo.jointCount];

				m_ElectronicTrackerInfo.pinchTHR = 0;

				hasElectronicTrackerInfo =
					(WVR().GetHandTrackerInfo(WVR_HandTrackerType::WVR_HandTrackerType_Electronic, &m_ElectronicTrackerInfo) == WVR_Result::WVR_Success ? true : false);

				if (hasElectronicTrackerInfo)
				{
					LOGD(DLF_INPUT_1, "TickHandData() Electronic tracker, joint %d, pinchTHR %f",
						m_ElectronicTrackerInfo.jointCount,
						m_ElectronicTrackerInfo.pinchTHR);

					for (int32_t i = 0; i < (int32_t)m_ElectronicTrackerInfo.jointCount; i++)
					{
						LOGD(DLF_INPUT_1, "TickHandData() Electronic tracker, jointMappingArray[%d] = %d, jointValidFlagArray[%d] = %d",
							i, (uint32_t)m_ElectronicTrackerInfo.jointMappingArray[i],
							i, (uint32_t)m_ElectronicTrackerInfo.jointValidFlagArray[i]);
					}

					/// Initialize m_ElectronicHandTrackerData
					m_ElectronicHandTrackerData.timestamp = 0;
					m_ElectronicHandTrackerData.left.confidence = 0;
					m_ElectronicHandTrackerData.left.isValidPose = false;
					m_ElectronicHandTrackerData.left.jointCount = m_ElectronicTrackerInfo.jointCount;
					if (m_ElectronicHandTrackerData.left.joints != nullptr)
					{
						delete[] m_ElectronicHandTrackerData.left.joints;
						m_ElectronicHandTrackerData.left.joints = nullptr;

						LOGD(DLF_INPUT_1, "TickHandData() Release m_ElectronicHandTrackerData.left.joints.");
					}
					m_ElectronicHandTrackerData.left.joints = new WVR_Pose_t[m_ElectronicTrackerInfo.jointCount];
					m_ElectronicHandTrackerData.left.scale.v[0] = 0;
					m_ElectronicHandTrackerData.left.scale.v[1] = 0;
					m_ElectronicHandTrackerData.left.scale.v[2] = 0;

					m_ElectronicHandTrackerData.right.confidence = 0;
					m_ElectronicHandTrackerData.right.isValidPose = false;
					m_ElectronicHandTrackerData.right.jointCount = m_ElectronicTrackerInfo.jointCount;
					if (m_ElectronicHandTrackerData.right.joints != nullptr)
					{
						delete[] m_ElectronicHandTrackerData.right.joints;
						m_ElectronicHandTrackerData.right.joints = nullptr;

						LOGD(DLF_INPUT_1, "TickHandData() Release m_ElectronicHandTrackerData.right.joints.");
					}
					m_ElectronicHandTrackerData.right.joints = new WVR_Pose_t[m_ElectronicTrackerInfo.jointCount];
					m_ElectronicHandTrackerData.right.scale.v[0] = 0;
					m_ElectronicHandTrackerData.right.scale.v[1] = 0;
					m_ElectronicHandTrackerData.right.scale.v[2] = 0;

					/// Initialize m_ElectronicHandPoseData
					m_ElectronicHandPoseData.timestamp = 0;
					m_ElectronicHandPoseData.left.base.type = WVR_HandPoseType::WVR_HandPoseType_Invalid;
					m_ElectronicHandPoseData.right.base.type = WVR_HandPoseType::WVR_HandPoseType_Invalid;
				}
			}
		} // if (!hasElectronicTrackerInfo)

		// Calls GetHandTrackingData on each frame.
		if (hasElectronicTrackerInfo)
		{
			WVR_HandModelType handModel = WVR_HandModelType::WVR_HandModelType_WithoutController;
			if ((m_ElectronicTrackerInfo.handModelTypeBitMask & (uint64_t)handModel) == 0)
				handModel = WVR_HandModelType::WVR_HandModelType_WithController;

			hasElectronicTrackerData = (
				WVR().GetHandTrackingData(
					WVR_HandTrackerType::WVR_HandTrackerType_Electronic,
					handModel,
					poseOrigin,
					&m_ElectronicHandTrackerData,
					&m_ElectronicHandPoseData
				) == WVR_Result::WVR_Success ? true : false);

			if (hasElectronicTrackerData)
			{
				UpdateElectronicJointPose();
				auto wvrScaleL = m_ElectronicHandTrackerData.left.scale;
				auto wvrScaleR = m_ElectronicHandTrackerData.right.scale;
				ElectronicHandScaleL.x = wvrScaleL.v[0];
				ElectronicHandScaleL.y = wvrScaleL.v[1];
				ElectronicHandScaleL.z = wvrScaleL.v[2];
				ElectronicHandScaleR.x = wvrScaleR.v[0];
				ElectronicHandScaleR.y = wvrScaleR.v[1];
				ElectronicHandScaleR.z = wvrScaleR.v[2];

				ElectronicWristLinearVelocityL = WVRVector3ToUnityVector3(m_ElectronicHandTrackerData.left.wristLinearVelocity);
				ElectronicWristLinearVelocityR = WVRVector3ToUnityVector3(m_ElectronicHandTrackerData.right.wristLinearVelocity);

				ElectronicWristAngularVelocityL = WVRVector3ToUnityVector3(m_ElectronicHandTrackerData.left.wristAngularVelocity);
				ElectronicWristAngularVelocityR = WVRVector3ToUnityVector3(m_ElectronicHandTrackerData.right.wristAngularVelocity);

				if (printable)
				{
				LOGD(DLF_INPUT_1, "TickHandData() Electronic left valid %d, right valid %d",
					(uint8_t)m_ElectronicHandTrackerData.left.isValidPose,
					(uint8_t)m_ElectronicHandTrackerData.right.isValidPose);
				LOGD(DLF_INPUT_1, "TickHandData() Electronic left scale (%f, %f, %f), right scale (%f, %f, %f)",
					ElectronicHandScaleL.x, ElectronicHandScaleL.y, ElectronicHandScaleL.z,
					ElectronicHandScaleR.x, ElectronicHandScaleR.y, ElectronicHandScaleR.z);
				LOGD(DLF_INPUT_1, "TickHandData() Electronic left linear velocity (%f, %f, %f), right linear velocity (%f, %f, %f)",
					ElectronicWristLinearVelocityL.x, ElectronicWristLinearVelocityL.y, ElectronicWristLinearVelocityL.z,
					ElectronicWristLinearVelocityR.x, ElectronicWristLinearVelocityR.y, ElectronicWristLinearVelocityR.z);
				LOGD(DLF_INPUT_1, "TickHandData() Electronic left angular velocity (%f, %f, %f), right angular velocity (%f, %f, %f)",
					ElectronicWristAngularVelocityL.x, ElectronicWristAngularVelocityL.y, ElectronicWristAngularVelocityL.z,
					ElectronicWristAngularVelocityR.x, ElectronicWristAngularVelocityR.y, ElectronicWristAngularVelocityR.z);
				}
			}
		}
		else
		{
			hasElectronicTrackerData = false;
		}
	}
}
void WaveVR_Hand::UpdateNaturalJointPose()
{
	if (!hasNaturalTrackerInfo || !hasNaturalTrackerData) { return; }

	for (int i = 0; i < m_NaturalTrackerInfo.jointCount; i++)
	{
		WVR_HandJoint joint = m_NaturalTrackerInfo.jointMappingArray[i];
		if ((m_NaturalTrackerInfo.jointValidFlagArray[i] & WVR_HandJointValidFlag_PositionValid) != 0)
		{
			if (i < m_NaturalHandTrackerData.left.jointCount && m_NaturalHandTrackerData.left.isValidPose)
			{
				s_JointPositionNaturalLeft[(uint32_t)joint] = WVRVector3ToUnityVector3(
					m_NaturalHandTrackerData.left.joints[i].position
				);
			}
			if (i < m_NaturalHandTrackerData.right.jointCount && m_NaturalHandTrackerData.right.isValidPose)
			{
				s_JointPositionNaturalRight[(uint32_t)joint] = WVRVector3ToUnityVector3(
					m_NaturalHandTrackerData.right.joints[i].position
				);
			}
		}
		if ((m_NaturalTrackerInfo.jointValidFlagArray[i] & WVR_HandJointValidFlag_RotationValid) != 0)
		{
			if (i < m_NaturalHandTrackerData.left.jointCount && m_NaturalHandTrackerData.left.isValidPose)
			{
				s_JointRotationNaturalLeft[(uint32_t)joint] = WVRQuatToUnityVector4(
					m_NaturalHandTrackerData.left.joints[i].rotation
				);
			}
			if (i < m_NaturalHandTrackerData.right.jointCount && m_NaturalHandTrackerData.right.isValidPose)
			{
				s_JointRotationNaturalRight[(uint32_t)joint] = WVRQuatToUnityVector4(
					m_NaturalHandTrackerData.right.joints[i].rotation
				);
			}
		}
	}
}
void WaveVR_Hand::UpdateElectronicJointPose()
{
	if (!hasElectronicTrackerInfo || !hasElectronicTrackerData) { return; }

	for (int i = 0; i < m_ElectronicTrackerInfo.jointCount; i++)
	{
		WVR_HandJoint joint = m_ElectronicTrackerInfo.jointMappingArray[i];
		if ((m_ElectronicTrackerInfo.jointValidFlagArray[i] & WVR_HandJointValidFlag_PositionValid) != 0)
		{
			if (i < m_ElectronicHandTrackerData.left.jointCount && m_ElectronicHandTrackerData.left.isValidPose)
			{
				s_JointPositionElectronicLeft[(uint32_t)joint] = WVRVector3ToUnityVector3(
					m_ElectronicHandTrackerData.left.joints[i].position
				);
			}
			if (i < m_ElectronicHandTrackerData.right.jointCount && m_ElectronicHandTrackerData.right.isValidPose)
			{
				s_JointPositionElectronicRight[(uint32_t)joint] = WVRVector3ToUnityVector3(
					m_ElectronicHandTrackerData.right.joints[i].position
				);
			}
		}
		if ((m_ElectronicTrackerInfo.jointValidFlagArray[i] & WVR_HandJointValidFlag_RotationValid) != 0)
		{
			if (i < m_ElectronicHandTrackerData.left.jointCount && m_ElectronicHandTrackerData.left.isValidPose)
			{
				s_JointRotationElectronicLeft[(uint32_t)joint] = WVRQuatToUnityVector4(
					m_ElectronicHandTrackerData.left.joints[i].rotation
				);
			}
			if (i < m_ElectronicHandTrackerData.right.jointCount && m_ElectronicHandTrackerData.right.isValidPose)
			{
				s_JointRotationElectronicRight[(uint32_t)joint] = WVRQuatToUnityVector4(
					m_ElectronicHandTrackerData.right.joints[i].rotation
				);
			}
		}
	}
}
#pragma endregion
#pragma region Hand Tracking Interface
HandTrackingStatus WaveVR_Hand::GetHandTrackingStatus(WVR_HandTrackerType tracker)
{
	if (tracker == WVR_HandTrackerType_Natural)
		return m_NaturalTrackerStatus;

	if (tracker == WVR_HandTrackerType_Electronic)
		return m_ElectronicTrackerStatus;

	return HandTrackingStatus::UNSUPPORT;
}
void WaveVR_Hand::SetHandTrackingStatus(WVR_HandTrackerType tracker, HandTrackingStatus status)
{
	if (tracker == WVR_HandTrackerType_Natural)
		m_NaturalTrackerStatus = status;

	if (tracker == WVR_HandTrackerType_Electronic)
		m_ElectronicTrackerStatus = status;
}
bool WaveVR_Hand::CanStartHandTracking(WVR_HandTrackerType tracker)
{
	HandTrackingStatus status = GetHandTrackingStatus(tracker);
	if (status == HandTrackingStatus::NOT_START ||
		status == HandTrackingStatus::START_FAILURE)
	{
		return true;
	}
	return false;
}
bool WaveVR_Hand::CanStopHandTracking(WVR_HandTrackerType tracker)
{
	return GetHandTrackingStatus(tracker) == HandTrackingStatus::AVAILABLE;
}
void WaveVR_Hand::StartHandTrackingLock(WVR_HandTrackerType tracker)
{
	if (!CanStartHandTracking(tracker)) { return; }

	LOGD(DLF_INPUT_1, "StartHandTrackingLock() %d", (uint32_t)tracker);
	SetHandTrackingStatus(tracker, HandTrackingStatus::STARTING);
	WVR_Result result = WVR().StartHandTracking(tracker);
	switch (result)
	{
		case WVR_Success:
			SetHandTrackingStatus(tracker, HandTrackingStatus::AVAILABLE);
			break;
		case WVR_Error_FeatureNotSupport:
			SetHandTrackingStatus(tracker, HandTrackingStatus::UNSUPPORT);
			break;
		default:
			SetHandTrackingStatus(tracker, HandTrackingStatus::START_FAILURE);
			break;
	}

	HandTrackingStatus status = GetHandTrackingStatus(tracker);
	LOGD(DLF_INPUT_1, "StartHandTrackingLock() tracker %d, result %d, status %d"
	, (uint32_t)tracker
	, (uint32_t)result
	, (uint32_t)status);
}
void WaveVR_Hand::StartHandTrackingThread(WVR_HandTrackerType tracker)
{
	lock_guard<mutex> mLock(m_HandTrackingMutex);
	LOGD(DLF_INPUT_1, "StartHandTrackingThread() %d", (uint32_t)tracker);
	StartHandTrackingLock(tracker);
}
void WaveVR_Hand::StartHandTracking(WVR_HandTrackerType tracker)
{
	if (!CanStartHandTracking(tracker))
	{
		HandTrackingStatus status = GetHandTrackingStatus(tracker);
		LOGD(DLF_INPUT_1, "StartHandTracking() can NOT start tracker %d, status %d"
			, (uint32_t)tracker
			, (uint32_t)status);
		return;
	}
	LOGD(DLF_INPUT_1, "StartHandTracking() %d", (uint32_t)tracker);
	thread start_hand_thread(&WaveVR_Hand::StartHandTrackingThread, this, tracker);
	start_hand_thread.detach();
}
void WaveVR_Hand::StopHandTrackingLock(WVR_HandTrackerType tracker)
{
	if (!CanStopHandTracking(tracker)) { return; }

	LOGD(DLF_INPUT_1, "StopHandTrackingLock() %d", (uint32_t)tracker);
	SetHandTrackingStatus(tracker, HandTrackingStatus::STOPING);
	WVR().StopHandTracking(tracker);
	SetHandTrackingStatus(tracker, HandTrackingStatus::NOT_START);
}
void WaveVR_Hand::StopHandTrackingThread(WVR_HandTrackerType tracker)
{
	lock_guard<mutex> mLock(m_HandTrackingMutex);
	LOGD(DLF_INPUT_1, "StopHandTrackingThread() %d", (uint32_t)tracker);
	StopHandTrackingLock(tracker);
}
void WaveVR_Hand::StopHandTracking(WVR_HandTrackerType tracker)
{
	if (!CanStopHandTracking(tracker))
	{
		HandTrackingStatus status = GetHandTrackingStatus(tracker);
		LOGD(DLF_INPUT_1, "StopHandTracking() can NOT stop tracker %d, status %d"
			, (uint32_t)tracker
			, (uint32_t)status);
		return;
	}
	LOGD(DLF_INPUT_1, "StopHandTracking() %d", (uint32_t)tracker);
	thread stop_hand_thread(&WaveVR_Hand::StopHandTrackingThread, this, tracker);
	stop_hand_thread.detach();
}
bool WaveVR_Hand::IsHandTrackingAvailable(WVR_HandTrackerType tracker)
{
	return GetHandTrackingStatus(tracker) == HandTrackingStatus::AVAILABLE;
}
float WaveVR_Hand::GetHandConfidence(WVR_HandTrackerType tracker, bool isLeft)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		if (hasNaturalTrackerData && hasNaturalTrackerInfo)
		{
			if (isLeft)
				return m_NaturalHandTrackerData.left.confidence;
			else
				return m_NaturalHandTrackerData.right.confidence;
		}
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		if (hasElectronicTrackerInfo && hasElectronicTrackerData)
		{
			if (isLeft)
				return m_ElectronicHandTrackerData.left.confidence;
			else
				return m_ElectronicHandTrackerData.right.confidence;
		}
	}

	return 0;
}
bool WaveVR_Hand::IsHandPoseValid(WVR_HandTrackerType tracker, bool isLeft)
{
	bool valid = false;
	if (tracker == WVR_HandTrackerType_Natural)
	{
		if (hasNaturalTrackerInfo && hasNaturalTrackerData)
		{
			if (isLeft)
				valid = m_NaturalHandTrackerData.left.isValidPose;
			else
				valid = m_NaturalHandTrackerData.right.isValidPose;
		}
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		if (hasElectronicTrackerInfo && hasElectronicTrackerData)
		{
			if (isLeft)
				valid = m_ElectronicHandTrackerData.left.isValidPose;
			else
				valid = m_ElectronicHandTrackerData.right.isValidPose;
		}
	}
	return valid;
}
UnityXRVector3 WaveVR_Hand::GetHandJointPosition(WVR_HandTrackerType tracker, bool isLeft, WVR_HandJoint joint)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		return (isLeft ?
			s_JointPositionNaturalLeft[(uint32_t)joint] :
			s_JointPositionNaturalRight[(uint32_t)joint]
		);
	}
	else
	{
		return (isLeft ?
			s_JointPositionElectronicLeft[(uint32_t)joint] :
			s_JointPositionElectronicRight[(uint32_t)joint]
		);
	}
}
UnityXRVector4 WaveVR_Hand::GetHandJointRotation(WVR_HandTrackerType tracker, bool isLeft, WVR_HandJoint joint)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		return (isLeft ?
			s_JointRotationNaturalLeft[(uint32_t)joint] :
			s_JointRotationNaturalRight[(uint32_t)joint]
		);
	}
	else
	{
		return (isLeft ?
			s_JointRotationElectronicLeft[(uint32_t)joint] :
			s_JointRotationElectronicRight[(uint32_t)joint]
		);
	}
}
WVR_HandPoseType WaveVR_Hand::GetHandMotion(WVR_HandTrackerType tracker, bool isLeft)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		if (hasNaturalTrackerInfo && hasNaturalTrackerData)
		{
			return (isLeft ?
				m_NaturalHandPoseData.left.base.type :
				m_NaturalHandPoseData.right.base.type);
		}
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		if (hasElectronicTrackerInfo && hasElectronicTrackerData)
		{
			return (isLeft ?
				m_ElectronicHandPoseData.left.base.type :
				m_ElectronicHandPoseData.right.base.type);
		}
	}

	return WVR_HandPoseType_Invalid;
}
float WaveVR_Hand::GetHandPinchThreshold(WVR_HandTrackerType tracker)
{
	if (tracker == WVR_HandTrackerType_Natural)
		return m_NaturalTrackerInfo.pinchTHR;
	if (tracker == WVR_HandTrackerType_Electronic)
		return m_ElectronicTrackerInfo.pinchTHR;

	return 0;
}
float WaveVR_Hand::GetHandPinchStrength(WVR_HandTrackerType tracker, bool isLeft)
{
	if (GetHandMotion(tracker, isLeft) == WVR_HandPoseType_Pinch)
	{
		if (tracker == WVR_HandTrackerType_Natural)
		{
			if (isLeft)
				return m_NaturalHandPoseData.left.pinch.strength;
			else
				return m_NaturalHandPoseData.right.pinch.strength;
		}
		if (tracker == WVR_HandTrackerType_Electronic)
		{
			if (isLeft)
				return m_ElectronicHandPoseData.left.pinch.strength;
			else
				return m_ElectronicHandPoseData.right.pinch.strength;
		}
	}
	return 0;
}
UnityXRVector3 WaveVR_Hand::GetHandPinchOrigin(WVR_HandTrackerType tracker, bool isLeft)
{
	UnityXRVector3 origin = kZeroVector;

	if (GetHandMotion(tracker, isLeft) == WVR_HandPoseType_Pinch)
	{
		if (tracker == WVR_HandTrackerType_Natural)
		{
			if (isLeft)
			{
				origin = wvr::utils::WVRVector3ToUnityVector3(m_NaturalHandPoseData.left.pinch.origin);
				origin.x += BONE_OFFSET_L.x;
				origin.y += BONE_OFFSET_L.y;
				origin.z += BONE_OFFSET_L.z;
			}
			else
			{
				origin = wvr::utils::WVRVector3ToUnityVector3(m_NaturalHandPoseData.right.pinch.origin);
				origin.x += BONE_OFFSET_R.x;
				origin.y += BONE_OFFSET_R.y;
				origin.z += BONE_OFFSET_R.z;
			}
		}
		if (tracker == WVR_HandTrackerType_Electronic)
		{
			if (isLeft)
			{
				origin = wvr::utils::WVRVector3ToUnityVector3(m_ElectronicHandPoseData.left.pinch.origin);
				origin.x += BONE_OFFSET_L.x;
				origin.y += BONE_OFFSET_L.y;
				origin.z += BONE_OFFSET_L.z;
			}
			else
			{
				origin = wvr::utils::WVRVector3ToUnityVector3(m_ElectronicHandPoseData.right.pinch.origin);
				origin.x += BONE_OFFSET_R.x;
				origin.y += BONE_OFFSET_R.y;
				origin.z += BONE_OFFSET_R.z;
			}
		}
	}

	return origin;
}
UnityXRVector3 WaveVR_Hand::GetHandPinchDirection(WVR_HandTrackerType tracker, bool isLeft)
{
	UnityXRVector3 direction = kZeroVector;

	if (GetHandMotion(tracker, isLeft) == WVR_HandPoseType_Pinch)
	{
		if (tracker == WVR_HandTrackerType_Natural)
		{
			if (isLeft)
			{
				direction = wvr::utils::WVRVector3ToUnityVector3(m_NaturalHandPoseData.left.pinch.direction);
				direction.x += BONE_OFFSET_L.x;
				direction.y += BONE_OFFSET_L.y;
				direction.z += BONE_OFFSET_L.z;
			}
			else
			{
				direction = wvr::utils::WVRVector3ToUnityVector3(m_NaturalHandPoseData.right.pinch.direction);
				direction.x += BONE_OFFSET_R.x;
				direction.y += BONE_OFFSET_R.y;
				direction.z += BONE_OFFSET_R.z;
			}
		}
		if (tracker == WVR_HandTrackerType_Electronic)
		{
			if (isLeft)
			{
				direction = wvr::utils::WVRVector3ToUnityVector3(m_ElectronicHandPoseData.left.pinch.direction);
				direction.x += BONE_OFFSET_L.x;
				direction.y += BONE_OFFSET_L.y;
				direction.z += BONE_OFFSET_L.z;
			}
			else
			{
				direction = wvr::utils::WVRVector3ToUnityVector3(m_ElectronicHandPoseData.right.pinch.direction);
				direction.x += BONE_OFFSET_R.x;
				direction.y += BONE_OFFSET_R.y;
				direction.z += BONE_OFFSET_R.z;
			}
		}
	}

	return direction;
}
WVR_HandHoldRoleType WaveVR_Hand::GetHandHoldRole(WVR_HandTrackerType tracker, bool isLeft)
{
	if (GetHandMotion(tracker, isLeft) == WVR_HandPoseType_Hold)
	{
		if (tracker == WVR_HandTrackerType_Natural)
		{
			return (isLeft ?
				m_NaturalHandPoseData.left.hold.role :
				m_NaturalHandPoseData.right.hold.role);
		}
		if (tracker == WVR_HandTrackerType_Electronic)
		{
			return (isLeft ?
				m_ElectronicHandPoseData.left.hold.role :
				m_ElectronicHandPoseData.right.hold.role);
		}
	}
	return WVR_HandHoldRoleType_None;
}
WVR_HandHoldObjectType WaveVR_Hand::GetHandHoldType(WVR_HandTrackerType tracker, bool isLeft)
{
	if (GetHandMotion(tracker, isLeft) == WVR_HandPoseType_Hold)
	{
		if (tracker == WVR_HandTrackerType_Natural)
		{
			return (isLeft ?
				m_NaturalHandPoseData.left.hold.object :
				m_NaturalHandPoseData.right.hold.object);
		}
		if (tracker == WVR_HandTrackerType_Electronic)
		{
			return (isLeft ?
				m_ElectronicHandPoseData.left.hold.object :
				m_ElectronicHandPoseData.right.hold.object);
		}
	}
	return WVR_HandHoldObjectType_None;
}
UnityXRVector3 WaveVR_Hand::GetHandScale(WVR_HandTrackerType tracker, bool isLeft)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		return isLeft ? NaturalHandScaleL : NaturalHandScaleR;
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		return isLeft ? ElectronicHandScaleL : ElectronicHandScaleR;
	}

	return kZeroVector;
}
UnityXRVector3 WaveVR_Hand::GetWristLinearVelocity(WVR_HandTrackerType tracker, bool isLeft)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		return isLeft ? NaturalWristLinearVelocityL : NaturalWristLinearVelocityR;
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		return isLeft ? ElectronicWristLinearVelocityL : ElectronicWristLinearVelocityR;
	}

	return kZeroVector;
}
UnityXRVector3 WaveVR_Hand::GetWristAngularVelocity(WVR_HandTrackerType tracker, bool isLeft)
{
	if (tracker == WVR_HandTrackerType_Natural)
	{
		return isLeft ? NaturalWristAngularVelocityL : NaturalWristAngularVelocityR;
	}
	if (tracker == WVR_HandTrackerType_Electronic)
	{
		return isLeft ? ElectronicWristAngularVelocityL : ElectronicWristAngularVelocityR;
	}

	return kZeroVector;
}
#pragma endregion
