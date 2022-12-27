#pragma once

#include <mutex>
#include <thread>
#include <atomic>
#include <string>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <wvr/wvr_types.h>
#include <wvr/wvr_hand.h>
#include <wvr/wvr_device.h>

#include "DrawEnum.h"

#include "shared/Matrices.h"
#include "object/Mesh.h"
#include "object/Texture.h"
#include "object/Shader.h"

#include "HandConstant.h"
#include "HandObj.h"
#include "vrb/CameraEye.h"

class HandManager {
public:
  HandManager(WVR_HandTrackerType iTrackingType);

  ~HandManager();

  bool isControllerAvailable(WVR_DeviceType param);

public:
  void onCreate();

  void onDestroy();

public:
  void updateAndRender(HandTypeEnum handMode, vrb::CameraEyePtr cameraEyePtr);

public:
  void handleHandTrackingMechanism();

protected:
  void startHandTracking();

  void stopHandTracking();

protected:
  void createSharedContext();

  void destroySharedContext();

  void loadHandModelAsync();

public:
  void calculateHandMatrices();

  float getButtonPressure(const HandTypeEnum handType);
  Matrix4 getHandMatrix(const HandTypeEnum handType);
  bool isDataAvailable(const HandTypeEnum handType);

protected:
  WVR_HandTrackerType mTrackingType;
  WVR_HandTrackerInfo_t mHandTrackerInfo;
  WVR_HandTrackingData_t mHandTrackingData;
  WVR_HandPoseData_t mHandPoseData;
  std::atomic<bool> mStartFlag;
protected:
  bool mIsPrintedSkeErrLog[Hand_MaxNumber];
protected:
  Matrix4 mJointMats[Hand_MaxNumber][sMaxSupportJointNumbers]; //Store mapping-convert poses.
  Matrix4 mHandPoseMats[Hand_MaxNumber];
  bool mIsHandPoseValids[Hand_MaxNumber];
protected:
  Matrix4 mHandRayMats[Hand_MaxNumber];
protected:
  HandObj *mHandObjs[Hand_MaxNumber];
  Texture *mHandAlphaTex;
  Matrix4 mShift;
  bool mHandInitialized;
  Matrix4 mProjectionMatrix;
  Matrix4 mEyeMatrix;
  Matrix4 mHeadMatrix;
protected:
  std::thread mLoadModelThread;
  std::mutex mGraphicsChangedMutex;
  EGLContext mEGLInitContext;
  EGLDisplay mEGLInitDisplay;
};
