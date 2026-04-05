# RKNN deploy demo

## 1.简介

该 demo 主要用于快速验证 RKNN 的 CAPI 推理。



目前支持以下功能：

- 获取推理性能

- 获取推理结果，并保存为npy文件
  - 需要指定输入，可以是图片或npy文件，输入图片与模型不匹配时，会自动做 resize 处理
  
- 若 rknn.build 时指定了 dataset、且dataset有效。则生成 rknn deploy demo时，会采用dataset的首行数据作为输出，通过 python api 的 rknn.inference 接口生成 golden 数据。板端执行demo时，会比对推理结果的余弦相似度

  **注意:** 由于 Python API 读取、缩放图片使用 opencv 库，而 rknn_deploy_demo 使用 stbi 读取、缩放图片，余弦相似度可能因此达不到0.9999



支持平台：

- [x] RK3566/RK3568/RK3588/RK3562
- [ ] RV1103/RV1103



支持特性：

- [x] 支持 RKNN API常规接口
- [x] 支持指定图片输入，输入图片尺寸不匹配时，使用 stbi 库进行 letterbox resize
- [x] 支持指定npy输入，目前只支持fp32的npy文件
- [x] 支持校验推理结果余弦距离

- [ ] RGA letterbox resize 功能
- [ ] 支持测试零拷贝
- [ ] 支持推送至板端编译



## 2.编译

指定本地 rknpu2 仓库路径（可从https://github.com/rockchip-linux/rknpu2 获取）

```
export RKNPU2={rknpu2_path}
```

如果是编译安卓系统的demo，请指定  ANDROID_NDK_PATH 路径

```
export ANDROID_NDK_PATH={android_ndk_path}
```



编译

```
./build_android.sh {RKNPU_platform} 或 ./build_linux.sh {RKNPU_platform}
如
./build_android.sh rk3562
```



## 3.推送至板端并执行

```
adb push ./install/rknn_deploy_demo /data/rknn_deploy_demo
adb push ./model.rknn /data/rknn_deploy_demo
adb shell
cd /data/rknn_deploy_demo
export LD_LIBRARY_PATH=./lib/

执行指令有以下4种形式
执行shell脚本推理
./run.sh

只获取性能
./RKNN_deploy_demo ./model.rknn

指定输入(若 ./data/outputs_golden 存在，则推理后会计算推理结果的余弦相似度)
./RKNN_deploy_demo ./model.rknn ./data/inputs/input0.jpg

指定输入、循环跑100次(若 ./data/outputs_golden 存在，则推理后会计算推理结果的余弦相似度)
./RKNN_deploy_demo ./model.rknn ./data/inputs/input0.jpg 100
```

- **这里假设模型名字为 model.rknn，实际使用时请替换为自己 rknn 模型的名字**

- **假设存在多输入 in0.npy, in1.npy，则推理指令为 ./RKNN_deploy_demo ./model.rknn ./data/inputs/in0.npy#./data/inputs/in1.npy**

