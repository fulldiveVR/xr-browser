# How to build ViveBrowser

## setup SDK

Make folder ./third_party/wavesdk/build
```bash
mkdir -p third_party/wavesdk/build
```

Download 5.0.2 VIVE SDK from [@HTC VIVE Native SDK download archive](https://developer.vive.com/resources/vive-wave/download/archive/)
```bash
curl -o ./third_party/wavesdk/wave_5.0.3_native.zip  https://dl.vive.com/SDK/wave/wave_5.0.3_native.zip
```

Unzip SDK
```bash
unzip ./third_party/wavesdk/wave_5.0.3_native.zip -d ./third_party/wavesdk
```

Unpack ./third_party/wavesdk/repo/com/htc/vr/wvr_client/5.0.3-u05/wvr_client-5.0.3-u05.aar into ./third_party/wavesdk/build (change .arr to .zip and use unzip)

```bash
cp ./third_party/wavesdk/repo/com/htc/vr/wvr_client/5.0.3-u05/wvr_client-5.0.3-u05.aar ./third_party/wavesdk/build/wvr_client-5.0.3-u05.zip
unzip ./third_party/wavesdk/build/wvr_client-5.0.3-u05.zip -d ./third_party/wavesdk/build/wvr_client-5.0.3-u05/
```

Check file `./third_party/wavesdk/build/wvr_client-5.0.3-u05/jni/arm64-v8a/libwvr_api.so` exist
```bash
[ ! -f ./third_party/wavesdk/build/wvr_client-5.0.3-u05/jni/arm64-v8a/libwvr_api.so ] && echo ">>> Not found"
```

Update `./version.gradle` set `versions.wavevr = "5.0.3-u05"`

Download VBR repo into ./app/src/main/cpp/vrb

```bash
git submodule update --init --recursive
```

## If nothing works

step 1
download repo from [@github.com/MozillaReality/vrb commit 4d13824588bc7127ab9d5ef015b2a1a4cfbb7430](https://github.com/MozillaReality/vrb/tree/4d13824588bc7127ab9d5ef015b2a1a4cfbb7430)
and place into ./app/src/main/cpp/vrb

step 2
and also you should the same download [@github.com/floooh/gliml commit aea9653b2e3a98de647087c04d9c8337c8683f92](https://github.com/floooh/gliml/tree/aea9653b2e3a98de647087c04d9c8337c8683f92)
and place into ./app/src/main/cpp/vrb/third_party/gliml

## Build

```bash
./gradlew
./gradlew assembleWavevrArm64WorldDebug
```

## result APK

Result APK located in `./app/build/outputs/apk/wavevrArm64/debug/FirefoxReality-wavevr-arm64-debug.apk`

# Build for Oculus

## Download and unpack Oculus SDKs into folders:
/third_party/OpenXR-SDK
/third_party/ovr_mobile
/third_party/ovr_openxr_mobile_sdk
/third_party/OVRPlatformSDK

Attention!: if you use OVRPlatformSDK with version for example 38, then application will work only on devices with updated firmware to 38 version ! I think we do not use latest version of SDK.

## Openxr build
Add
openxr=true
into user.properties file (in root folder) for openxr build

# Already prepared package of third_party folder

You can just unzip third_party.zip from root - it contains all unpacked and configured SDKs for Wave and Oculus build

# Add .dev to package for test on device already has preinstalled browser with the same package id

add
simultaneousDevProduction=true
into user.properties file (in root folder)
