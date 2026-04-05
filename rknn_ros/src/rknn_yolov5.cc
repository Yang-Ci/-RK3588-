// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>


#define _BASETSD_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "yolov5_postprocess.h"
#include "rknn_api.h"

#define PERF_WITH_POST 1
/*-------------------------------------------
                  Functions
-------------------------------------------*/

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz)
{
  unsigned char* data;
  int            ret;

  data = NULL;

  if (NULL == fp) {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0) {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char*)malloc(sz);
  if (data == NULL) {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  FILE*          fp;
  unsigned char* data;

  fp = fopen(filename, "rb");
  if (NULL == fp) {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

static int saveFloat(const char* file_name, float* output, int element_size)
{
  FILE* fp;
  fp = fopen(file_name, "w");
  for (int i = 0; i < element_size; i++) {
    fprintf(fp, "%.6f\n", output[i]);
  }
  fclose(fp);
  return 0;
}

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "object_information_msgs/Object.h"
#include <std_msgs/Int32.h>

ros::Time last_time;
object_information_msgs::Object objMsg;
ros::Publisher obj_pub;
ros::Publisher kuan_pub;
ros::Publisher chang_pub;
image_transport::Publisher image_pub;
detect_result_group_t detect_result_group;
int channel = 3;
int width   = 0;
int height  = 0;
bool display_output = true;
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::ImagePtr image_msg;
rknn_context   ctx;
rknn_input_output_num io_num;
rknn_input inputs[1];
rknn_output outputs[3];
rknn_tensor_attr output_attrs[3];
const float    nms_threshold      = NMS_THRESH;
float    box_conf_threshold = BOX_THRESH;

static const unsigned char colors[19][3] = {
    {54, 67, 244},
    {99, 30, 233},
    {176, 39, 156},
    {183, 58, 103},
    {181, 81, 63},
    {243, 150, 33},
    {244, 169, 3},
    {212, 188, 0},
    {136, 150, 0},
    {80, 175, 76},
    {74, 195, 139},
    {57, 220, 205},
    {59, 235, 255},
    {7, 193, 255},
    {0, 152, 255},
    {34, 87, 255},
    {72, 85, 121},
    {158, 158, 158},
    {139, 125, 96}
};

static int rknn_detect_yolov5(const cv::Mat& bgr, detect_result_group_t* detect_result)
{
  int            ret;
  int            img_width          = bgr.cols;
  int            img_height         = bgr.rows;

  // You may not need resize when src resulotion equals to dst resulotion
  memset(inputs, 0, sizeof(inputs));
  inputs[0].index        = 0;
  inputs[0].type         = RKNN_TENSOR_UINT8;
  inputs[0].size         = width * height * channel;
  inputs[0].fmt          = RKNN_TENSOR_NHWC;
  inputs[0].pass_through = 0;

  cv::Mat dst_image;
  if (img_width != width || img_height != height) 
  {
    ros::Time current_time = ros::Time::now();
    cv::resize( bgr,dst_image, cv::Size(width,height));   
    inputs[0].buf = (void*)dst_image.data;
  } 
  else 
  {
    inputs[0].buf = (void*)bgr.data;
  }

  rknn_inputs_set(ctx, io_num.n_input, inputs);
  
  memset(outputs, 0, sizeof(outputs));
  for (int i = 0; i < io_num.n_output; i++) {
    outputs[i].want_float = 0;
  }
  ret = rknn_run(ctx, NULL);
  ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

  // post process
  float scale_w = (float)width / img_width;
  float scale_h = (float)height / img_height;

  std::vector<float>    out_scales;
  std::vector<int32_t>  out_zps;
  for (int i = 0; i < io_num.n_output; ++i) {
    out_scales.push_back(output_attrs[i].scale);
    out_zps.push_back(output_attrs[i].zp);
  }
  post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf, height, width,
               box_conf_threshold, nms_threshold, scale_w, scale_h, out_zps, out_scales, detect_result);
  ret = rknn_outputs_release(ctx, io_num.n_output, outputs);
  return ret;
}

static cv::Mat draw_objects(const cv::Mat& bgr, detect_result_group_t detect_result_group)
{
  int color_index = 0;

  cv::Mat image = bgr.clone();

  // Draw Objects
  char text[256];
  for (int i = 0; i < detect_result_group.count; i++) {
    const unsigned char* color = colors[color_index % 19];
    color_index++;
    cv::Scalar cc(color[0], color[1], color[2]);

    detect_result_t* det_result = &(detect_result_group.results[i]);
    sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
    int x1 = det_result->box.left;
    int y1 = det_result->box.top;
    int x2 = det_result->box.right;
    int y2 = det_result->box.bottom;
    rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cc, 3);

    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    int x = x1;
    int y = y1 - label_size.height - baseLine;
    if (y < 0)
        y = 0;
    if (x + label_size.width > image.cols)
        x = image.cols - label_size.width;    

    cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                    cc, -1);
    putText(image, text, cv::Point(x, y+label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
  }
  return image;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    float fps = 0.0;
    ros::Time current_time = ros::Time::now();

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    rknn_detect_yolov5(cv_ptr->image,&detect_result_group);
    fps = 1.0/(ros::Time::now() - current_time).toSec();
    last_time = current_time;

    for (size_t i = 0; i < detect_result_group.count; i++)
    {
        detect_result_t* det_result = &(detect_result_group.results[i]);
        int x1 = det_result->box.left;
        int y1 = det_result->box.top;
        int x2 = det_result->box.right;
        int y2 = det_result->box.bottom;
        ROS_INFO("%s = %.5f at %d %d %d x %d", det_result->name, det_result->prop,
        x1, y1, (x2-x1), (y2-y1));
        objMsg.header.seq++;
        objMsg.header.stamp = current_time;
        objMsg.probability = det_result->prop;
        objMsg.label = det_result->name;
        objMsg.position.position.x = x1;
        objMsg.position.position.y = y1;
        objMsg.size.x = y2-y1;
        objMsg.size.y = x2-x1;
        obj_pub.publish(objMsg);
        std_msgs::Int32 kuan_msg;
        std_msgs::Int32 chang_msg;
        chang_msg.data = y2-y1;
        chang_pub.publish(chang_msg);
        kuan_msg.data = x2-x1;
        kuan_pub.publish(kuan_msg);
    }
    if (display_output) {
      cv::Mat out_image ;
      out_image = draw_objects(cv_ptr->image, detect_result_group);
      char text[32];
      sprintf(text, "RKNN YOLO V5 FPS %.03f", fps);
      cv::putText(out_image, text, cv::Point(20, 20),cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0));      
      image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out_image).toImageMsg();
      image_pub.publish(image_msg);
    }
    
    last_time = current_time;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("CV bridge exception: %s", e.what());
    return;
  }
}


/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char** argv)
{
  int            ret;

  //ROS node relative 
  ros::init(argc, argv, "rknn_yolov5_node"); /**/
  ros::NodeHandle nhLocal("~");
  ros::NodeHandle n;
  std::string node_name = ros::this_node::getName();
  nhLocal.param("prob_threshold", box_conf_threshold, 0.35f);
  nhLocal.param("display_output", display_output, true);
  const std::string package_name = "rknn_ros";
  std::string chip_type;
  nhLocal.param("chip_type", chip_type, std::string("RK3588"));
  std::string path = ros::package::getPath(package_name)+("/models/")+chip_type+("/");
  ROS_INFO("Assets path: %s", path.c_str());
  std::string model_file;
  nhLocal.param("model_file", model_file, std::string("best.rknn"));
  /* Create the neural network */
  ROS_INFO("Loading mode...\n");
  ROS_INFO("Loading  model path: %s", (path+model_file).c_str());
  int            model_data_size = 0;
  unsigned char* model_data      = load_model((path+model_file).c_str(), &model_data_size);
  ret                            = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
  if (ret < 0) {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }

  rknn_sdk_version version;
  ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
  if (ret < 0) {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }
  ROS_INFO("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);


  ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if (ret < 0) {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }
  ROS_INFO("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  rknn_tensor_attr input_attrs[io_num.n_input];
  memset(input_attrs, 0, sizeof(input_attrs));
  for (int i = 0; i < io_num.n_input; i++) {
    input_attrs[i].index = i;
    ret                  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret < 0) {
      printf("rknn_init error ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
  }

  memset(output_attrs, 0, sizeof(output_attrs));
  for (int i = 0; i < io_num.n_output; i++) {
    output_attrs[i].index = i;
    ret                   = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    dump_tensor_attr(&(output_attrs[i]));
  }

  if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
    ROS_INFO("model is NCHW input fmt\n");
    channel = input_attrs[0].dims[1];
    height  = input_attrs[0].dims[2];
    width   = input_attrs[0].dims[3];
  } else {
    ROS_INFO("model is NHWC input fmt\n");
    height  = input_attrs[0].dims[1];
    width   = input_attrs[0].dims[2];
    channel = input_attrs[0].dims[3];
  }

  ROS_INFO("model input height=%d, width=%d, channel=%d\n", height, width, channel);

  //ROS node relative 
  image_transport::ImageTransport it(n);
  image_pub = it.advertise("/rknn_image", 1);
  obj_pub = n.advertise<object_information_msgs::Object>("/objects", 50);
  kuan_pub = n.advertise<std_msgs::Int32>("/objects_kuan", 50);
  chang_pub = n.advertise<std_msgs::Int32>("/objects_chang", 50);
  image_transport::Subscriber video = it.subscribe("/usb_cam/image_raw", 1, imageCallback);


  while (ros::ok()) {
    ros::spinOnce();
  }

  // release
  ret = rknn_destroy(ctx);
  if (model_data) {
    free(model_data);
  }
  return 0;
}
