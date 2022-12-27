#ifndef DEVICE_DELEGATE_WAVE_VR_DOT_H
#define DEVICE_DELEGATE_WAVE_VR_DOT_H

#include "vrb/Forward.h"
#include "vrb/MacroUtils.h"
#include "DeviceDelegate.h"
#include "vrb/Vector.h"
#include "shared/Matrices.h"
#include <memory>
#include <wvr/wvr_hand.h>

const uint32_t kHandCount = 2;
const uint32_t kJointCount = 26;

namespace crow {
class DeviceDelegateWaveVR;
typedef std::shared_ptr<DeviceDelegateWaveVR> DeviceDelegateWaveVRPtr;

class DeviceDelegateWaveVR : public DeviceDelegate {
public:
  static DeviceDelegateWaveVRPtr Create(vrb::RenderContextPtr& aContext);
  void InitializeRender();
  // DeviceDelegate interface
  device::DeviceType GetDeviceType() override;
  void SetRenderMode(const device::RenderMode aMode) override;
  device::RenderMode GetRenderMode() override;
  void RegisterImmersiveDisplay(ImmersiveDisplayPtr aDisplay) override;
  void SetImmersiveSize(const uint32_t aEyeWidth, const uint32_t aEyeHeight) override;
  GestureDelegateConstPtr GetGestureDelegate() override;
  vrb::CameraPtr GetCamera(const device::Eye aWhich) override;
  const vrb::Matrix& GetHeadTransform() const override;
  const vrb::Matrix& GetReorientTransform() const override;
  void SetReorientTransform(const vrb::Matrix& aMatrix) override;
  void SetClearColor(const vrb::Color& aColor) override;
  void SetClipPlanes(const float aNear, const float aFar) override;
  void SetControllerDelegate(ControllerDelegatePtr& aController) override;
  void ReleaseControllerDelegate() override;
  int32_t GetControllerModelCount() const override;
  void ProcessEvents() override;
  void StartFrame(const FramePrediction aPrediction) override;
  void BindEye(const device::Eye aWhich) override;
  void EndFrame(const FrameEndMode aMode) override;
  vrb::LoadTask GetControllerModelTask(int32_t index) override;
  // DeviceDelegateWaveVR interface
  bool IsRunning();
protected:
  struct State;
  DeviceDelegateWaveVR(State& aState);
  virtual ~DeviceDelegateWaveVR();
private:
  State& m;
  VRB_NO_DEFAULTS(DeviceDelegateWaveVR)

private:
  void UpdateController(const int id, const uint32_t deviceId, const vrb::Matrix hmd);
  void HandsCalculate(const vrb::Matrix hmd);

protected:
    inline Matrix4 wvrmatrixConverter(const WVR_Matrix4f_t& mat) const {
        return Matrix4(
            mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
            mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
            mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
            mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
        );
    }
};

} // namespace crow
#endif // DEVICE_DELEGATE_WAVE_VR_DOT_H
