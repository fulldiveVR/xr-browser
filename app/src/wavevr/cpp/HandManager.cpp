#define LOG_TAG "APHandModule"

#include <memory>
#include "log.h"

#include "shared/quat.h"
#include "HandManager.h"
#include "vrb/Matrix.h"
#include "vrb/Geometry.h"
#include "vrb/Group.h"

namespace crow {
  static const int32_t kControllerCount = 2;  // Left, Right

  void WVR_HandJointData_tToMatrix4(const WVR_Pose_t &iPose, uint64_t iValidBitMask,
                                    Matrix4 &ioPoseMat) {

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


  HandManager::HandManager(WVR_HandTrackerType iType, ControllerDelegatePtr &controllerDelegatePtr)
      : mTrackingType(iType),
        delegate(controllerDelegatePtr), renderMode(device::RenderMode::StandAlone),
        mHandTrackerInfo({}), mHandTrackingData({}), mHandPoseData({}), mStartFlag(false),
        modelCachedData(nullptr) {
    mShift.translate(1, 1.5, 2);
//    memset((void *) modelCachedData, 0, sizeof(WVR_HandRenderModel_t));
    memset((void *) isModelDataReady, 0, sizeof(bool) * kControllerCount);
  }

  HandManager::~HandManager() {
  }

  void HandManager::onCreate() {
//  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
//    mHandObjs[hID] = new HandObj(this, static_cast<HandTypeEnum>(hID));
//  }
  }

  void HandManager::onDestroy() {
    stopHandTracking();

//  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
//    mHandObjs[hID]->releaseGraphicsSystem();
//    delete mHandObjs[hID];
//    mHandObjs[hID] = nullptr;
//  }

    if (mHandAlphaTex != nullptr) {
      delete mHandAlphaTex;
      mHandAlphaTex = nullptr;
    }

    if (modelCachedData != nullptr) {
      WVR_ReleaseNatureHandModel(&modelCachedData); //we will clear cached data ptr to nullptr.
      modelCachedData = nullptr;
    }
  }

  void HandManager::update() {
//  for (uint32_t hID = 0; hID < Hand_MaxNumber; ++hID) {
//    mHandObjs[hID] = new HandObj(this, static_cast<HandTypeEnum>(hID));
////      mHandObjs[hID]->initializeGraphicsSystem()
//  }
    if (WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Left) ||
        WVR_IsDeviceConnected(WVR_DeviceType_NaturalHand_Right)) {
      if (!mStartFlag) {
        startHandTracking();
      }
    }

    if (mStartFlag) {
      calculateHandMatrices();
    }
  }

  void HandManager::startHandTracking() {
    WVR_Result result;
    if (mStartFlag) {
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
    WVR_Vector3f_t trackingScale = (handMode == Hand_Left) ? mHandTrackingData.left.scale
                                                           : mHandTrackingData.right.scale;

//    if (handMode == Hand_Right) {
//        trackingScale.v[0] = 1.0f;
//        trackingScale.v[1] = 1.0f;
//        trackingScale.v[2] = 1.0f;
//    }

//  if (mHandObjs[handMode] != nullptr && mIsHandPoseValids[handMode]) {
//    mHandObjs[handMode]->updateSkeleton(
//        mJointMats[handMode],
//        Vector3(trackingScale.v[0], trackingScale.v[1], trackingScale.v[2])
//    );

//    mHandObjs[handMode]->render(
//        mProjectionMatrix,
//        mEyeMatrix,
//        mHeadMatrix,
//        mHandPoseMats[handMode]
//    );
//}
  }

  void HandManager::calculateHandMatrices() {
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
      if (handJoints->isValidPose) {
        for (uint32_t jCount = 0; jCount < handJoints->jointCount; ++jCount) {
          uint32_t jID = mHandTrackerInfo.jointMappingArray[jCount];
          uint64_t validBits = mHandTrackerInfo.jointValidFlagArray[jCount];
          WVR_HandJointData_tToMatrix4(handJoints->joints[jCount],
                                       validBits, mJointMats[handType][jID]);
        }

        if (mIsPrintedSkeErrLog[handType]) {
          LOGI("T(%d):Hand[%d] : pose recovered.", mTrackingType, handType);
        }
        mIsPrintedSkeErrLog[handType] = false;
        mIsHandPoseValids[handType] = true;
        mHandPoseMats[handType] = mJointMats[handType][WVR_HandJoint_Wrist];
      } else {
        if (!mIsPrintedSkeErrLog[handType]) {
          LOGI("T(%d):Hand[%d] : pose invalid.", mTrackingType, handType);
          mIsPrintedSkeErrLog[handType] = true;
        }
        mIsHandPoseValids[handType] = false;
        mHandPoseMats[handType] = Matrix4();
      }

      //2. Ray data.
      if (handPose->base.type == WVR_HandPoseType_Pinch &&
          mTrackingType == WVR_HandTrackerType_Natural) {
        Vector3 rayUp(mHandPoseMats[handType][4], mHandPoseMats[handType][5],
                      mHandPoseMats[handType][6]);
//      Vector3 rayUp(0.0, 1.0, 0.0);
        rayUp.normalize();
        Vector3 rayOri(handPose->pinch.origin.v);
        Vector3 negRayF = Vector3(handPose->pinch.direction.v);
//      Vector3 negRayF = (Vector3(handPose->pinch.direction.v) * -1.0f);
        mHandRayMats[handType] = Matrix_LookAtFrom(rayOri, negRayF, rayUp);
      }
    }
  }

  float HandManager::getPinchStrength(const ElbowModel::HandEnum hand) {
    if (hand == ElbowModel::HandEnum::Left) {
      return mHandPoseData.left.pinch.strength;
    }
    return mHandPoseData.right.pinch.strength;
  }

  vrb::Matrix HandManager::getRayMatrix(const ElbowModel::HandEnum hand) {
    WVR_HandPoseState_t *handPose;
    Matrix4 handPoseMat;
    if (hand == ElbowModel::HandEnum::Left) {
      handPose = &mHandPoseData.left;
      handPoseMat = mHandPoseMats[Hand_Left];
    } else {
      handPose = &mHandPoseData.right;
      handPoseMat = mHandPoseMats[Hand_Right];
    }

    Vector3 rayUp(handPoseMat[4], handPoseMat[5], handPoseMat[6]);
    rayUp.normalize();
    Vector3 rayOri = Vector3(handPose->pinch.origin.v);
    Vector3 negRayF = Vector3(handPose->pinch.direction.v) * -1.0f;

    Matrix4 rayMatrix = Matrix_LookAtFrom(rayOri, negRayF, rayUp);
    vrb::Matrix matrix = vrb::Matrix::FromColumnMajor(rayMatrix.get());
    return matrix;
  }

  bool HandManager::isHandAvailable(const ElbowModel::HandEnum hand) {
    if (hand == ElbowModel::HandEnum::Left) {
      return mIsHandPoseValids[Hand_Left];
    }
    return mIsHandPoseValids[Hand_Right];
  }

  void HandManager::loadHandModelAsync() {
//  std::lock_guard<std::mutex> lck(mGraphicsChangedMutex);
//  LOGI("T(%d): loadHandModelAsync Begin(CSBegin)", mTrackingType);
//
//  if (renderModel != nullptr) {
////    mHandObjs[Hand_Left]->loadModel(&(*renderModel).left);
////    mHandObjs[Hand_Right]->loadModel(&(*renderModel).right);
////    mHandAlphaTex = Texture::loadTextureFromBitmapWithoutCached((*renderModel).handAlphaTex);
////    mHandObjs[Hand_Left]->setTexture(mHandAlphaTex);
////    mHandObjs[Hand_Right]->setTexture(mHandAlphaTex);
//
//  }
    LOGI("T(%d): loadHandModelAsync End(CSEnd)", mTrackingType);
  }

  void HandManager::setRenderMode(const device::RenderMode mode) {
    renderMode = mode;
  }

  void HandManager::updateHand(Controller &controller, const WVR_DevicePosePair_t &devicePair,
                               const vrb::Matrix &hmd) {
    delegate->SetBatteryLevel(controller.index, 30);

    if (!isHandAvailable(controller.hand)) {
      return;
    }

    controller.transform = getRayMatrix(controller.hand);

    if (renderMode == device::RenderMode::StandAlone) {
      controller.transform.TranslateInPlace(kAverageHeight);
    }
    delegate->SetTransform(controller.index, controller.transform);
  }

  void HandManager::updateHandState(Controller &controller) {
    float bumperPressure = getPinchStrength(controller.hand);

    if (bumperPressure < 0.1) {
      bumperPressure = -1.0;
    } else if (bumperPressure > 0.9) {
      bumperPressure = 1.0;
    }
    bool bumperPressed = bumperPressure > 0.4;

    delegate->SetSelectActionStop(controller.index);
    delegate->SetFocused(controller.index);
    delegate->SetButtonCount(controller.index, 1);
    delegate->SetButtonState(controller.index, ControllerDelegate::BUTTON_TRIGGER,
                             device::kImmersiveButtonTrigger, bumperPressed,
                             bumperPressed, bumperPressure);
  }

  vrb::LoadTask HandManager::getHandModelTask(ControllerMetaInfo controllerMetaInfo) {
    return [this, controllerMetaInfo](vrb::CreationContextPtr &aContext) -> vrb::GroupPtr {
      vrb::GroupPtr root = vrb::Group::Create(aContext);
      // Load controller model from SDK
      VRB_LOG("[WaveVR] (%p) Loading internal hand model: %d", this, controllerMetaInfo.hand);

      if (modelCachedData == nullptr) {
        WVR_GetCurrentNaturalHandModel(&modelCachedData);
      }


      WVR_HandModel_t &handModel = modelCachedData->left;
      if (controllerMetaInfo.hand == ElbowModel::HandEnum::Right) {
        handModel = modelCachedData->right;
      }

      // TODO: load hand.

      return nullptr;
    };
  }
}