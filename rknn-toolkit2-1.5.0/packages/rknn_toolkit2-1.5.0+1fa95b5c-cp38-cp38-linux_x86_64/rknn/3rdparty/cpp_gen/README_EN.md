# RKNN deploy demo

## 1.Introduction

This demo is mainly used to quickly verify the CAPI inference of RKNN.



Currently, the following functions are supported:

- Obtain inference performance

- Obtain inference results and save them as npy files
  - Input needs to be specified, which can be an image or npy file. When the input image does not match the model, automatic resizing will be performed.
  
- If the dataset is specified during "**rknn.build**" and the dataset is valid, the first row of the dataset will be used as the output when generating the rknn deploy demo. The golden data will be generated through the "**rknn.inference**" interface of the Python API. When the demo is executed on the board, the cosine simularity of the inference results will be compared.

  **Notice:** Due to Python API read/resize image via “opencv”, while rknn deploy demo use stbi, this may cause cosine simularity could lower than 0.9999



Supported platforms:

- [x] RK3566/RK3568/RK3588/RK3562
- [ ] RV1103/RV1103



Feature List:

- [x]  Support RKNN API general interface
- [x]  Support specifying image input. When the input image size does not match, use the stbi library for letterbox resizing.
- [x]  Support specifying npy input, currently only supporting npy files of fp32.
- [x]  Support verification of inference result cosine distance.
- [ ]  RGA letterbox resize function
- [ ]  Support testing zero copy
- [ ]  Support pushing to board-side compilation



## 2.Compilation

Specify the local rknpu2 repository path (available at https://github.com/rockchip-linux/rknpu2)

```
export RKNPU2={rknpu2_path}
```

If compiling the demo for Android system, please specify the ANDROID_NDK_PATH path

```
export ANDROID_NDK_PATH={android_ndk_path}
```



Compile

```
./build_android.sh {RKNPU_platform} or ./build_linux.sh {RKNPU_platform} 

e.g.
./build_android.sh rk3562
```



## 3.Push to device and execute

```
adb push ./install/rknn_deploy_demo /data/rknn_deploy_demo
adb push ./model.rknn /data/rknn_deploy_demo
adb shell
cd /data/rknn_deploy_demo
export LD_LIBRARY_PATH=./lib/

#There are four ways to execute the command:
./run.sh

#Only obtain performance
./RKNN_deploy_demo ./model.rknn

#Specify input (if "./data/outputs_golden" exists, it will calculate the cosine similarity of the inference results after inference)
./RKNN_deploy_demo ./model.rknn ./data/inputs/cat.jpg

#Specify input, and run it for 100 times (if "./data/outputs_golden" exists, it will calculate the cosine similarity of the inference results after inference)
./RKNN_deploy_demo ./model.rknn ./data/inputs/cat.jpg 100
```

- **Here we assume the model name is "model.rknn"**.  Please adjust the name to your rknn model name.

- If the model has multiple inputs, let's suppose the inputs are named as in0.npy and in1.npy. The command to be followed is: **"./RKNN_deploy_demo ./model.rknn ./data/inputs/in0.npy#./data/inputs/in1.npy"**.

