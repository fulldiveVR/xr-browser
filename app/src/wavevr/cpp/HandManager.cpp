#define LOG_TAG "APHandModule"

#include <memory>
#include "log.h"

#include "shared/quat.h"
#include "HandManager.h"
#include "HandsShaders.h"
#include "vrb/Matrix.h"
#include "vrb/Geometry.h"
#include "vrb/Group.h"
#include "vrb/TextureGL.h"
#include "vrb/VertexArray.h"
#include "vrb/Program.h"
#include "vrb/ProgramFactory.h"
#include "vrb/CreationContext.h"
#include "vrb/RenderState.h"
#include "vrb/Color.h"
#include "vrb/Vector4.h"

namespace crow {
  static const std::string kHandGeometryName = "HandGeometry";
  static const int32_t kControllerCount = 2;  // Left, Right
  static const vrb::Color kWhiteColor = vrb::Color(1.0f, 1.0f, 1.0f);
  static const vrb::Color kBlackColor = vrb::Color(0.0f, 0.0f, 0.0f);
  static const vrb::Color kBlueColor = vrb::Color(29.0f / 255.0f, 189.0f / 255.0f, 247.0f / 255.0f);
  static const vrb::Color kBlue2Color = vrb::Color(191.0f / 255.0f, 182.0f / 255.0f,
                                                   182.0f / 255.0f);

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
        modelCachedData(nullptr), mTexture(nullptr) {
    mShift.translate(1, 1.5, 2);
//    memset((void *) modelCachedData, 0, sizeof(WVR_HandRenderModel_t));
    memset((void *) isModelDataReady, 0, sizeof(bool) * kControllerCount);
  }

  HandManager::~HandManager() {
  }

  void HandManager::onCreate() {
  }

  void HandManager::onDestroy() {
    stopHandTracking();

    mTexture = nullptr;

    if (modelCachedData != nullptr) {
      WVR_ReleaseNatureHandModel(&modelCachedData); //we will clear cached data ptr to nullptr.
      modelCachedData = nullptr;
    }
  }

  void HandManager::update() {
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
        rayUp.normalize();
        Vector3 rayOri(handPose->pinch.origin.v);
        Vector3 negRayF = Vector3(handPose->pinch.direction.v) * -1.0f;
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

  void HandManager::setRenderMode(const device::RenderMode mode) {
    renderMode = mode;
  }

  void HandManager::updateHand(Controller &controller, const WVR_DevicePosePair_t &devicePair,
                               const vrb::Matrix &hmd) {
    if (!isHandAvailable(controller.hand)) {
      return;
    }

    controller.transform = getRayMatrix(controller.hand);

    const int handIndex = int(controller.hand);
    auto jointMat = mJointMats[handIndex];
    auto skeletonMatrices = mSkeletonMatrices[handIndex];
    auto jointUsageTable = mJointUsageTable[handIndex];
    auto jointInvTransMats = mJointInvTransMats[handIndex];
    auto finalSkeletonPoses = mFinalSkeletonPoses[handIndex];
    auto skeletonPoses = mSkeletonPoses[handIndex];
    auto modelSkeletonPoses = mModelSkeletonPoses[handIndex];

    Matrix4 wristPose = jointMat[HandBone_Wrist];
    auto wristPoseInv = Matrix4(wristPose);
    wristPoseInv.invert();

    const auto blockSize = 16 * sizeof(float);

    for (uint32_t jointID = 0; jointID < sMaxSupportJointNumbers; ++jointID) {
      skeletonPoses[jointID] = (wristPoseInv * jointMat[jointID]); // convert to model space.

      if (jointUsageTable[jointID] == 1) {
        Vector4 wp = calculateJointWorldPosition(handIndex, jointID);
        finalSkeletonPoses[jointID] = skeletonPoses[jointID];
        finalSkeletonPoses[jointID][12] = wp.x;
        finalSkeletonPoses[jointID][13] = wp.y;
        finalSkeletonPoses[jointID][14] = wp.z;
        modelSkeletonPoses[jointID] = finalSkeletonPoses[jointID] * jointInvTransMats[jointID];
      }

      memcpy(skeletonMatrices + jointID * 16, modelSkeletonPoses[jointID].get(), blockSize);
    }

    if (renderMode == device::RenderMode::StandAlone) {
      controller.transform.TranslateInPlace(kAverageHeight);
    }

    delegate->SetTransform(controller.index, controller.transform);
    delegate->SetJointsMatrices(controller.index, kHandGeometryName, skeletonMatrices);
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

  Vector4 HandManager::calculateJointWorldPosition(const int handIndex, uint32_t jID) const {
    Vector4 wp;
    Vector3 lp = getModelJointLocalPosition(handIndex, jID);

    if (mJointParentTable[handIndex][jID] == sIdentityJoint) {
      wp = Vector4(lp.x, lp.y, lp.z, 1.0f);
    } else {
      uint32_t parentJointID = mJointParentTable[handIndex][jID];
      Matrix4 parentJointTrans = mFinalSkeletonPoses[handIndex][parentJointID];
      wp = parentJointTrans * Vector4(lp.x, lp.y, lp.z, 1.0f);
    }

    return wp;
  }

  Vector3 HandManager::getModelJointLocalPosition(const int handIndex, uint32_t iJointID) const {
    Vector3 result;
    if (iJointID < sMaxSupportJointNumbers) {
      result = Vector3(
          mJointLocalTransMats[handIndex][iJointID][12],
          mJointLocalTransMats[handIndex][iJointID][13],
          mJointLocalTransMats[handIndex][iJointID][14]);
    }
    return result;
  }


  vrb::LoadTask HandManager::getHandModelTask(ControllerMetaInfo controllerMetaInfo) {
    return [this, controllerMetaInfo](vrb::CreationContextPtr &aContext) -> vrb::GroupPtr {
      vrb::GroupPtr root = vrb::Group::Create(aContext);
      VRB_LOG("[WaveVR] (%p) Loading internal hand model: %d", this, controllerMetaInfo.hand);

      if (modelCachedData == nullptr) {
        WVR_GetCurrentNaturalHandModel(&modelCachedData);
      }

      uint32_t handType = Hand_Left;
      WVR_HandModel_t &handModel = modelCachedData->left;
      if (controllerMetaInfo.hand == ElbowModel::HandEnum::Right) {
        handModel = modelCachedData->right;
        handType = Hand_Right;
      }

      for (uint32_t jointID = 0; jointID < sMaxSupportJointNumbers; ++jointID) {
        float *mat;

        mat = ((float *) handModel.jointInvTransMats + jointID * 16);
        mJointInvTransMats[handType][jointID].set(mat);
        mat = ((float *) handModel.jointTransMats + jointID * 16);
        mJointTransMats[handType][jointID].set(mat);
        mat = ((float *) handModel.jointLocalTransMats + jointID * 16);
        mJointLocalTransMats[handType][jointID].set(mat);

        mJointParentTable[handType][jointID] = handModel.jointParentTable[jointID];
        mJointUsageTable[handType][jointID] = handModel.jointUsageTable[jointID];
      }

      // Initialize textures
      if (mTexture == nullptr) {
        VRB_LOG("[WaveVR] (%p) [%d]: Initialize texture", this, controllerMetaInfo.type);

        WVR_CtrlerTexBitmap_t &srcTexture = modelCachedData->handAlphaTex;
        size_t texture_size = srcTexture.stride * srcTexture.height;
        std::unique_ptr<uint8_t[]> data = std::make_unique<uint8_t[]>(texture_size);
        memcpy(data.get(), (void *) srcTexture.bitmap, texture_size);

        mTexture = vrb::TextureGL::Create(aContext);
        mTexture->SetImageData(
            data,
            texture_size,
            (int) srcTexture.width,
            (int) srcTexture.height,
            GL_RGBA
        );
      } else {
        VRB_LOG("[WaveVR] (%p) [%d]: Texture initialized already", this, controllerMetaInfo.type);
      }

      uint32_t verticesCount = handModel.vertices.size;

      VRB_LOG("[WaveVR] (%p) [%d]: Initialize mesh. Vertices: %d", this, controllerMetaInfo.type,
              verticesCount)

      vrb::VertexArrayPtr array = vrb::VertexArray::Create(aContext);

      // Vertices

      WVR_VertexBuffer_t &vertices = handModel.vertices;
      if (vertices.buffer == nullptr || vertices.size == 0 ||
          vertices.dimension == 0) {
        VRB_LOG("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", vertices.buffer,
                vertices.size, vertices.dimension);
        return nullptr;
      }

      uint32_t verticesComponents = vertices.dimension;
      if (verticesComponents == 3) {
        for (uint32_t i = 0; i < vertices.size; i += verticesComponents) {
          auto vertex = vrb::Vector(vertices.buffer[i], vertices.buffer[i + 1],
                                    vertices.buffer[i + 2]);
          array->AppendVertex(vertex);
        }
      } else {
        VRB_ERROR("[WaveVR] (%p) [%d]: vertex with wrong dimension: %d", this,
                  controllerMetaInfo.type,
                  verticesComponents)
      }

      // Normals
      WVR_VertexBuffer_t &normals = handModel.normals;
      if (normals.buffer == nullptr || normals.size == 0 ||
          normals.dimension == 0) {
        VRB_LOG("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", normals.buffer,
                normals.size, normals.dimension);
        return nullptr;
      }

      uint32_t normalsComponents = normals.dimension;
      if (normalsComponents == 3) {
        for (uint32_t i = 0; i < normals.size; i += normalsComponents) {
          auto normal = vrb::Vector(normals.buffer[i], normals.buffer[i + 1],
                                    normals.buffer[i + 2]).Normalize();
          array->AppendNormal(normal);
        }
      } else {
        VRB_ERROR("[WaveVR] (%p) [%d]: normal with wrong dimension: %d", this,
                  controllerMetaInfo.type,
                  normalsComponents)
      }

      // UV
      WVR_VertexBuffer_t &texCoords = handModel.texCoords;
      if (texCoords.buffer == nullptr || texCoords.size == 0 ||
          texCoords.dimension == 0) {
        VRB_LOG("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", texCoords.buffer,
                texCoords.size, texCoords.dimension);
        return nullptr;
      }

      uint32_t texCoordsComponents = texCoords.dimension;
      if (texCoordsComponents == 2) {
        for (uint32_t i = 0; i < texCoords.size; i += texCoordsComponents) {
          auto textCoord = vrb::Vector(texCoords.buffer[i], texCoords.buffer[i + 1],
                                       0);
          array->AppendUV(textCoord);
        }
      } else {
        VRB_ERROR("[WaveVR] (%p) [%d]: UVs with wrong dimension: %d", this,
                  controllerMetaInfo.type,
                  texCoordsComponents)
      }

      // Bones
      WVR_BoneIDBuffer_t &jointsIds = handModel.boneIDs;
      if (jointsIds.buffer == nullptr || jointsIds.size == 0 ||
          jointsIds.dimension == 0) {
        VRB_LOG("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", jointsIds.buffer,
                jointsIds.size, jointsIds.dimension);
        return nullptr;
      }

      uint32_t boneIdsComponents = jointsIds.dimension;
      if (boneIdsComponents == 4) {
        for (uint32_t i = 0; i < jointsIds.size; i += boneIdsComponents) {
          auto joints = vrb::Vector4((float) jointsIds.buffer[i], (float) jointsIds.buffer[i + 1],
                                     (float) jointsIds.buffer[i + 2],
                                     (float) jointsIds.buffer[i + 3]);
          array->AppendJoints(joints);
        }
      } else {
        VRB_ERROR("[WaveVR] (%p) [%d]: BIs with wrong dimension: %d", this,
                  controllerMetaInfo.type,
                  boneIdsComponents)
      }

      WVR_VertexBuffer_t &jointsWeights = handModel.boneWeights;
      if (jointsWeights.buffer == nullptr || jointsWeights.size == 0 ||
          jointsWeights.dimension == 0) {
        VRB_LOG("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", jointsWeights.buffer,
                jointsWeights.size, jointsWeights.dimension);
        return nullptr;
      }

      uint32_t jointWeightsComponents = jointsWeights.dimension;
      if (jointWeightsComponents == 4) {
        for (uint32_t i = 0; i < jointsWeights.size; i += jointWeightsComponents) {
          auto jointWeights = vrb::Vector4(jointsWeights.buffer[i], jointsWeights.buffer[i + 1],
                                           jointsWeights.buffer[i + 2],
                                           jointsWeights.buffer[i + 3]);
          array->AppendJointWeights(jointWeights);
        }
      } else {
        VRB_ERROR("[WaveVR] (%p) [%d]: BWs with wrong dimension: %d", this,
                  controllerMetaInfo.type,
                  jointWeightsComponents)
      }

      // Shaders
      vrb::ProgramPtr program = aContext->GetProgramFactory()->CreateProgram(aContext,
                                                                             vrb::FeatureTexture,
                                                                             GetHandDepthFragment(),
                                                                             sMaxSupportJointNumbers);
      vrb::RenderStatePtr state = vrb::RenderState::Create(aContext);
      state->SetProgram(program);
      state->SetLightsEnabled(false);
      state->SetTintColor(kBlueColor);
      state->SetTexture(mTexture);
      state->SetJointsCount(sMaxSupportJointNumbers);

      // Geometry
      vrb::GeometryPtr geometry = vrb::Geometry::Create(aContext);
      geometry->SetName(kHandGeometryName);
      geometry->SetVertexArray(array);
      geometry->SetRenderState(state);

      // Indices
      WVR_IndexBuffer_t &indices = handModel.indices;
      if (indices.buffer == nullptr || indices.size == 0 || indices.type == 0) {
        VRB_ERROR("Parameter invalid!!! iData(%p), iSize(%u), iType(%u)", indices.buffer,
                  indices.size, indices.type);
        return nullptr;
      }

      uint32_t type = indices.type;
      for (uint32_t i = 0; i < indices.size; i += type) {
        std::vector<int> indicesVector;
        for (uint32_t j = 0; j < type; j++) {
          indicesVector.push_back((int) (indices.buffer[i + j] + 1));
        }
        geometry->AddFace(indicesVector, indicesVector, indicesVector);
      }
      root->AddNode(geometry);

      return root;
    };
  }
}