#define LOG_TAG "APHandModule"

#include <memory>
#include "log.h"

#include "shared/quat.h"
#include "HandManager.h"
#include "vrb/Matrix.h"

void WVR_PoseState_tToMatrix4(const WVR_PoseState_t &iPoseState, Matrix4 &ioPoseMat) {
  for (uint32_t r = 0; r < 4; ++r) {
    for (uint32_t c = 0; c < 4; ++c) {
      ioPoseMat[c * 4 + r] = iPoseState.poseMatrix.m[r][c];
    }
  }
}

void
WVR_HandJointData_tToMatrix4(const WVR_Pose_t &iPose, uint64_t iValidBitMask, Matrix4 &ioPoseMat) {

  q_xyz_quat_struct q_trans = {};

  if ((iValidBitMask & WVR_HandJointValidFlag_RotationValid) ==
      WVR_HandJointValidFlag_RotationValid) {
    q_trans.quat[Q_X] = iPose.rotation.x;
    q_trans.quat[Q_Y] = iPose.rotation.y;
    q_trans.quat[Q_Z] = iPose.rotation.z;
    q_trans.quat[Q_W] = iPose.rotation.w;
  } else {
    q_trans.quat[Q_X] = 0.0;
    q_trans.quat[Q_Y] = 0.0;
    q_trans.quat[Q_Z] = 0.0;
    q_trans.quat[Q_W] = 1.0;
  }

  if ((iValidBitMask & WVR_HandJointValidFlag_PositionValid) ==
      WVR_HandJointValidFlag_PositionValid) {
    q_trans.xyz[0] = iPose.position.v[0];
    q_trans.xyz[1] = iPose.position.v[1];
    q_trans.xyz[2] = iPose.position.v[2];
  } else {
    q_trans.xyz[0] = 0.0;
    q_trans.xyz[1] = 0.0;
    q_trans.xyz[2] = 0.0;
  }

  ioPoseMat = Matrix4(q_trans);
}

Matrix4 WVR_Matrix4fToMatrix4(const WVR_Matrix4f &matPose) {
  return Matrix4(
      matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], matPose.m[3][0],
      matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], matPose.m[3][1],
      matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], matPose.m[3][2],
      matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], matPose.m[3][3]
  );
}

void ClearWVR_HandTrackerInfo(WVR_HandTrackerInfo_t &ioInfo) {
  if (ioInfo.jointMappingArray != nullptr) {
    delete[] ioInfo.jointMappingArray;
    ioInfo.jointMappingArray = nullptr;
  }
  if (ioInfo.jointValidFlagArray != nullptr) {
    delete[] ioInfo.jointValidFlagArray;
    ioInfo.jointValidFlagArray = nullptr;
  }

  ioInfo = {};
}


void InitializeWVR_HandTrackerInfo(WVR_HandTrackerInfo_t &ioInfo, uint32_t iJointCount) {
  //Clear old.
  ClearWVR_HandTrackerInfo(ioInfo);

  if (iJointCount > 0) {
    ioInfo.jointCount = iJointCount;
    ioInfo.jointMappingArray = new WVR_HandJoint[ioInfo.jointCount];
    ioInfo.jointValidFlagArray = new uint64_t[ioInfo.jointCount];
  }
}

void InitializeWVR_HandJointsData(WVR_HandTrackingData_t &ioData) {
  ioData = {};
  ioData.left.joints = nullptr;
  ioData.right.joints = nullptr;
}

void ClearWVR_HandJointsData(WVR_HandTrackingData_t &ioData) {
  if (ioData.left.joints != nullptr) {
    delete[] ioData.left.joints;
    ioData.left.joints = nullptr;
  }

  if (ioData.right.joints != nullptr) {
    delete[] ioData.right.joints;
    ioData.right.joints = nullptr;
  }

  ioData = {};
}

void
InitializeWVR_HandTrackingData(WVR_HandTrackingData_t &ioData, WVR_HandModelType iHandModelType,
                               const WVR_HandTrackerInfo_t &iTrackerInfo) {
  InitializeWVR_HandJointsData(ioData);

  if (iTrackerInfo.jointCount > 0) {
    ioData.left.jointCount = iTrackerInfo.jointCount;
    ioData.left.joints = new WVR_Pose_t[iTrackerInfo.jointCount];
    ioData.right.jointCount = iTrackerInfo.jointCount;
    ioData.right.joints = new WVR_Pose_t[iTrackerInfo.jointCount];
  }
}


HandManager::HandManager(WVR_HandTrackerType iType)
    : mHandObjs{nullptr, nullptr}, mTrackingType(iType), mHandTrackerInfo({}),
      mHandTrackingData({}), mHandPoseData({}), mStartFlag(false), mHandInitialized(false) {
  mShift.translate(1, 1.5, 2);
}

HandManager::~HandManager() {
}

void HandManager::onCreate() {
  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
    mHandObjs[hID] = new HandObj(this, static_cast<HandTypeEnum>(hID));
  }
}

void HandManager::onDestroy() {
  stopHandTracking();

  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
    mHandObjs[hID]->releaseGraphicsSystem();
    delete mHandObjs[hID];
    mHandObjs[hID] = nullptr;
  }

  delete mHandAlphaTex;
  mHandAlphaTex = nullptr;
  mHandInitialized = false;
}

void HandManager::handleHandTrackingMechanism() {
  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
    mHandObjs[hID] = new HandObj(this, static_cast<HandTypeEnum>(hID));
//      mHandObjs[hID]->initializeGraphicsSystem()
  }
  WVR_InteractionMode currentMode = WVR_GetInteractionMode();
  if (WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Left) == true ||
      WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Right) == true) {
    if (mStartFlag == false) {
      startHandTracking();
    }

    if (!mHandInitialized) {
      loadHandModelAsync();
    }
  }
}

void HandManager::startHandTracking() {
  WVR_Result result;
  if (mStartFlag == true) {
    LOGE("HandManager started!!!");
    return;
  }

  LOGI("AP:startHandTracking()++");
  uint32_t jointCount = 0u;
  result = WVR_GetHandJointCount(mTrackingType, &jointCount);
  LOGI("AP:WVR_GetHandJointCount()");
  if (result != WVR_Success) {
    LOGE("WVR_GetHandJointCount failed(%d).", result);
    return;
  }
  InitializeWVR_HandTrackerInfo(mHandTrackerInfo, jointCount);
  InitializeWVR_HandTrackingData(mHandTrackingData, WVR_HandModelType_WithoutController,
                                 mHandTrackerInfo);

  LOGI("AP:WVR_GetHandTrackerInfo()");
  if (WVR_GetHandTrackerInfo(mTrackingType, &mHandTrackerInfo) != WVR_Success) {
    LOGE("WVR_GetHandTrackerInfo failed(%d).", result);
    return;
  }

  LOGI("AP:WVR_StartHandTracking()");
  result = WVR_StartHandTracking(mTrackingType);
  if (result == WVR_Success) {
    mStartFlag.exchange(true);
    LOGI("WVR_StartHandTracking successful.");
  } else {
    LOGE("WVR_StartHandTracking error(%d).", result);
  }
  LOGI("AP:startHandTracking()--");
}

void HandManager::stopHandTracking() {
  if (!WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Right) ||
      !WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Left)) {
    return;
  }

  mStartFlag.exchange(false);
  WVR_StopHandTracking(WVR_HandTrackerType_Natural);
}


void HandManager::updateAndRender(HandTypeEnum handMode, vrb::CameraEyePtr cameraEyePtr) {
  if (!mHandInitialized)return;

  mProjectionMatrix.set(cameraEyePtr->GetPerspective().Data());
  mEyeMatrix.set(cameraEyePtr->GetEyeTransform().Data());
  mHeadMatrix.set(cameraEyePtr->GetHeadTransform().Data());

  mProjectionMatrix.identity();
  mEyeMatrix.identity();
  mHeadMatrix.identity();

  WVR_Vector3f_t trackingScale = (handMode == Hand_Left) ? mHandTrackingData.left.scale
                                                         : mHandTrackingData.right.scale;

//    if (handMode == Hand_Right) {
//        trackingScale.v[0] = 1.0f;
//        trackingScale.v[1] = 1.0f;
//        trackingScale.v[2] = 1.0f;
//    }

  if (mHandObjs[handMode] != nullptr && mIsHandPoseValids[handMode]) {
    mHandObjs[handMode]->updateSkeleton(
        mJointMats[handMode],
        Vector3(trackingScale.v[0], trackingScale.v[1], trackingScale.v[2])
    );

//    mHandObjs[handMode]->render(
//        mProjectionMatrix,
//        mEyeMatrix,
//        mHeadMatrix,
//        mHandPoseMats[handMode]
//    );
  }
}

void HandManager::calculateHandMatrices() {
  if (!mHandInitialized)return;
  WVR_Result result = WVR_GetHandTrackingData(
      WVR_HandTrackerType_Natural,
      WVR_HandModelType_WithoutController,
      WVR_PoseOriginModel_OriginOnHead,
      &mHandTrackingData, &mHandPoseData);

  for (uint32_t handType = 0; handType < Hand_MaxNumber; ++handType) {
    WVR_HandJointData_t *handJoints = nullptr;
    WVR_HandPoseState_t *handPose = nullptr;
    if (static_cast<HandTypeEnum>(handType) == Hand_Left) {
      handJoints = &mHandTrackingData.left;
      handPose = &mHandPoseData.left;
    } else {
      handJoints = &mHandTrackingData.right;
      handPose = &mHandPoseData.right;
    }

    //1. Pose data.
    if (handJoints->isValidPose == true) {
      for (uint32_t jCount = 0; jCount < handJoints->jointCount; ++jCount) {
        uint32_t jID = mHandTrackerInfo.jointMappingArray[jCount];
        uint64_t validBits = mHandTrackerInfo.jointValidFlagArray[jCount];
        WVR_HandJointData_tToMatrix4(handJoints->joints[jCount],
                                     validBits, mJointMats[handType][jID]);
      }

      if (mIsPrintedSkeErrLog[handType] == true) {
        LOGI("T(%d):Hand[%d] : pose recovered.", mTrackingType, handType);
      }
      mIsPrintedSkeErrLog[handType] = false;
      mIsHandPoseValids[handType] = true;
      mHandPoseMats[handType] = mJointMats[handType][WVR_HandJoint_Wrist];
    } else {
      if (mIsPrintedSkeErrLog[handType] == false) {
        LOGI("T(%d):Hand[%d] : pose invalid.", mTrackingType, handType);
        mIsPrintedSkeErrLog[handType] = true;
      }
      mIsHandPoseValids[handType] = false;
      mHandPoseMats[handType] = Matrix4();
    }

    //2. Ray data.
    if (handPose->base.type == WVR_HandPoseType_Pinch &&
        mTrackingType == WVR_HandTrackerType_Natural) {
//      Vector3 rayUp(mHandPoseMats[handType][4], mHandPoseMats[handType][5],
//                    mHandPoseMats[handType][6]);
      Vector3 rayUp(0.0, 1.0, 0.0);
      rayUp.normalize();
      Vector3 rayOri(handPose->pinch.origin.v);
      Vector3 negRayF = Vector3(handPose->pinch.direction.v);
      mHandRayMats[handType] = Matrix_LookAtFrom(rayOri, negRayF, rayUp);
    }
  }
}

float HandManager::getButtonPressure(const HandTypeEnum handType) {
  WVR_HandPoseState_t *handPose = nullptr;
  if (handType == Hand_Left) {
    handPose = &mHandPoseData.left;
  } else {
    handPose = &mHandPoseData.right;
  }
  return handPose->pinch.strength;
}

Matrix4 HandManager::getHandMatrix(const HandTypeEnum handType) {
  WVR_HandPoseState_t *handPose = nullptr;
  if (handType == Hand_Left) {
    handPose = &mHandPoseData.left;
  } else {
    handPose = &mHandPoseData.right;
  }

  Vector3 rayUp(mHandPoseMats[handType][4], mHandPoseMats[handType][5],
                mHandPoseMats[handType][6]);
  rayUp.normalize();
  Vector3 rayOri = Vector3(handPose->pinch.origin.v);
  Vector3 negRayF = Vector3(handPose->pinch.direction.v) * -1.0f;
  return Matrix_LookAtFrom(rayOri, negRayF, rayUp);
}

bool HandManager::isDataAvailable(const HandTypeEnum handType) {
  return mIsHandPoseValids[handType];
}

void HandManager::loadHandModelAsync() {
  std::lock_guard<std::mutex> lck(mGraphicsChangedMutex);
  LOGI("T(%d): loadHandModelAsync Begin(CSBegin)", mTrackingType);
  WVR_HandRenderModel_t *renderModel = nullptr;
  WVR_GetCurrentNaturalHandModel(&renderModel);

  if (renderModel != nullptr) {
//    mHandObjs[Hand_Left]->loadModel(&(*renderModel).left);
//    mHandObjs[Hand_Right]->loadModel(&(*renderModel).right);
//    mHandAlphaTex = Texture::loadTextureFromBitmapWithoutCached((*renderModel).handAlphaTex);
//    mHandObjs[Hand_Left]->setTexture(mHandAlphaTex);
//    mHandObjs[Hand_Right]->setTexture(mHandAlphaTex);

    WVR_ReleaseNatureHandModel(&renderModel);
  }
  LOGI("T(%d): loadHandModelAsync End(CSEnd)", mTrackingType);
  mHandInitialized = true;
}

bool HandManager::isControllerAvailable(WVR_DeviceType param) {
  return (param == WVR_DeviceType_Controller_Left && isDataAvailable(Hand_Left)) ||
         (param == WVR_DeviceType_Controller_Right && isDataAvailable(Hand_Right));
}
