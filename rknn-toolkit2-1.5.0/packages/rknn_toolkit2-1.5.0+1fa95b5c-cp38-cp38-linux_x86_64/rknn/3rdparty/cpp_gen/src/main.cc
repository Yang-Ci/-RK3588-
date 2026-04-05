
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>


#include <fstream>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "rknn_api.h"

#include "rknn_demo_utils.h"
#include "resize_function.h"
#include "timer.h"

#define NPY_SUPPORT 1
#if NPY_SUPPORT
#include "cnpy.h"
using namespace cnpy;
#endif



void check_ret(int ret, const char* msg){
    if (ret < 0){
        printf("ERROR: %s failed! ret=%d\n", msg, ret);
        exit(-1);
    }
}


static unsigned char* load_npy(const char* input_path, rknn_tensor_attr* input_attr){
    NpyArray npy_data = npy_load(input_path);
    int type_bytes = npy_data.word_size;
    std::string typeName = npy_data.typeName;
    // printf("npy data type:%s\n", typeName.c_str());

    // TODO now only support float32
    if (typeName != "float32") {
        printf("  meet unsupported type:%s\n", typeName.c_str());
        return NULL;
    }

    // check shape match
    bool nchw2nhwc = false;
    if (input_attr != NULL)
    {
        int req_height  = 0;
        int req_width   = 0;
        int req_channel = 0;

        switch (input_attr->fmt) {
            case RKNN_TENSOR_NHWC:
                req_height  = input_attr->dims[1];
                req_width   = input_attr->dims[2];
                req_channel = input_attr->dims[3];
                break;
            case RKNN_TENSOR_NCHW:
                req_height  = input_attr->dims[2];
                req_width   = input_attr->dims[3];
                req_channel = input_attr->dims[1];
                break;
            case RKNN_TENSOR_UNDEFINED:
                break;
            default:
                printf("  meet unsupported layout\n");
            return NULL;
        }

        int npy_shape[4] = {1, 1, 1, 1};
        int start = npy_data.shape.size() == 4 ? 0 : 1;
        for (size_t i = 0; i < npy_data.shape.size() && i < 4; ++i) {
            npy_shape[start + i] = npy_data.shape[i];
        }

        int height  = npy_shape[1];
        int width   = npy_shape[2];
        int channel = npy_shape[3];

        if ((input_attr->fmt != RKNN_TENSOR_UNDEFINED) &&
            (width != req_width || height != req_height || channel != req_channel)) {
            printf("  npy shape match failed!, (%d, %d, %d) != (%d, %d, %d)\n", height, width, channel, req_height, req_width,
                req_channel);

            nchw2nhwc = true;
            if (input_attr->fmt != RKNN_TENSOR_NHWC){
                nchw2nhwc = false;
            }
            if ((req_channel!=height) || (req_height!=width) || (req_width!=channel)){
                nchw2nhwc = false;
            }
            if (npy_data.shape.size()!=4){
                nchw2nhwc = false;
            }

            if (nchw2nhwc == false){
                return NULL;
            }
        }
    }

    unsigned char* data = (unsigned char*)malloc(npy_data.num_bytes());
    if (!data){
        return NULL;
    }
    memcpy(data, npy_data.data<unsigned char>(), npy_data.num_bytes());

    if (nchw2nhwc){
        printf("  try to convert nchw2nhwc\n");
        int batch   = npy_data.shape[0];
        int channel = npy_data.shape[1];
        int height  = npy_data.shape[2];
        int width   = npy_data.shape[3];

        unsigned char* data2 = (unsigned char*)malloc(npy_data.num_bytes());
        if (!data2){
            return NULL;
        }

        for (int b = 0; b < batch; ++b) {
            for (int c = 0; c < channel; ++c) {
                for (int h = 0; h < height; ++h) {
                    for (int w = 0; w < width; ++w) {
                        int src_idx = b * channel * height * width + c * height * width + h * width + w;
                        int dst_idx = b * channel * height * width + h * width * channel + w * channel + c;
                        memcpy(data2 + dst_idx * type_bytes, data + src_idx * type_bytes, type_bytes);
                    }
                }
            }
        }
        free(data);
        data = data2;
    }
    return data;
}


static void save_npy(const char* output_path, float* output_data, rknn_tensor_attr* output_attr)
{
    std::vector<size_t> output_shape;

    for (uint32_t i = 0; i < output_attr->n_dims; ++i) {
        output_shape.push_back(output_attr->dims[i]);
    }

    npy_save<float>(output_path, output_data, output_shape);
}


int folder_mkdirs(const char *folder_path)
{	
	if(!access(folder_path, F_OK)){
		return 0;
	}

	char path[256];
	char *path_buf = path;
	char temp_path[256];
	char *temp;
	int temp_len;
	
	memset(path, 0, sizeof(path));
	memset(temp_path, 0, sizeof(temp_path));
	strcat(path, folder_path);
	path_buf = path;

	while((temp = strsep(&path_buf, "/")) != NULL){
		temp_len = strlen(temp);	
		if(0 == temp_len){
			continue;
		}
		strcat(temp_path, temp);
		if(-1 == access(temp_path, F_OK)){
			if(-1 == mkdir(temp_path, 0777)){
				return 2;
			}
		}
        strcat(temp_path, "/");
	}
	return 0;
}

int replacechar(char *str, char orig, char rep) {
    char *ix = str;
    int n = 0;
    while((ix = strchr(ix, orig)) != NULL) {
        *ix++ = rep;
        n++;
    }
    return n;
}

int save_rknn_result(const char* output_folder_path, MODEL_INFO* m_info){
    int ret = 0;
    ret = folder_mkdirs(output_folder_path);
    if (ret != 0){return ret;}

    printf("SAVE OUTPUTS AS NPY\n");
    for (int out_index=0; out_index < m_info->n_output; out_index++){
        char* output_name = m_info->out_attr[out_index].name;
        char* output_name_replace_no_allow_char = (char*)malloc(strlen(output_name) + 1);
        memcpy(output_name_replace_no_allow_char, output_name, strlen(output_name) + 1);
        ret = replacechar(output_name_replace_no_allow_char, '/', '_');

        char* output_path = (char*)malloc(strlen(output_folder_path) + strlen(output_name_replace_no_allow_char) + 20);
        ret = sprintf(output_path, "%s/%s.npy", output_folder_path, output_name_replace_no_allow_char);
        printf("  output[%d]: %s\n", out_index, output_path);
        save_npy(output_path, (float*)m_info->outputs[out_index].buf, &m_info->out_attr[out_index]);

        free(output_name_replace_no_allow_char);
        free(output_path);
    }
    return ret;
}


static unsigned char *load_image_and_autoresize(const char *image_path, LETTER_BOX* letter_box, rknn_tensor_attr *input_attr)
{
    // printf("loading %s\n", image_path);
    int req_height = 0;
    int req_width = 0;
    int req_channel = 0;

    switch (input_attr->fmt)
    {
    case RKNN_TENSOR_NHWC:
        req_height = input_attr->dims[2];
        req_width = input_attr->dims[1];
        req_channel = input_attr->dims[3];
        break;
    case RKNN_TENSOR_NCHW:
        //Need to double check dims!!!!!
        req_height = input_attr->dims[2];
        req_width = input_attr->dims[1];
        req_channel = input_attr->dims[3];
        break;
    default:
        printf("meet unsupported layout\n");
        return NULL;
    }

    int height = 0;
    int width = 0;
    int channel = 0;

    unsigned char *image_data = stbi_load(image_path, &width, &height, &channel, req_channel);
    if (image_data == NULL)
    {
        printf("load image-%s failed!\n", image_path);
        return NULL;
    }

    if (channel == 1){
        printf("image is grey, convert to RGB");
        void* rgb_data = malloc(width* height* 3);
        for(int i=0; i<height; i++){
            for(int j=0; j<width; j++){
                    int offset = (i*width + j)*3;
                    ((unsigned char*)rgb_data)[offset] = ((unsigned char*)image_data)[offset];
                    ((unsigned char*)rgb_data)[offset + 1] = ((unsigned char*)image_data)[offset];
                    ((unsigned char*)rgb_data)[offset + 2] = ((unsigned char*)image_data)[offset];
            }
        }
        free(image_data);
        image_data = (unsigned char*)rgb_data;
        channel = 3;
    }

    letter_box->in_width = width;
    letter_box->in_height = height;
    letter_box->channel = channel;

    if (width != req_width || height != req_height || channel != req_channel)
    {
        printf("image shape match failed!, (%d, %d, %d) != (%d, %d, %d)\n", height, width, channel, req_height, req_width, req_channel);
        printf("Trying resize image\n");        

        letter_box->target_height = req_height;
        letter_box->target_width = req_width;

        unsigned char *resize_buf = (unsigned char *)malloc(letter_box->target_height* letter_box->target_width* letter_box->channel);
        if (resize_buf == NULL){
            printf("malloc resize_buf failed!\n");
            return NULL;
        }

        compute_letter_box(letter_box);
        stb_letter_box_resize(image_data, resize_buf, *letter_box);
        letter_box->reverse_available = true;
        free(image_data);
        image_data = resize_buf;
    }

    return image_data;
}


static std::vector<std::string> split(const std::string& str, const std::string& pattern)
{
  std::vector<std::string> res;
  if (str == "")
    return res;
  std::string strs = str + pattern;
  size_t      pos  = strs.find(pattern);
  while (pos != strs.npos) {
    std::string temp = strs.substr(0, pos);
    res.push_back(temp);
    strs = strs.substr(pos + 1, strs.size());
    pos  = strs.find(pattern);
  }
  return res;
}


bool random_generate_input(void** input_data, MODEL_INFO* m_info){
    bool all_int8 = true;
    for (int i=0; i < m_info->n_input; i++){
        switch (m_info->in_attr[i].type)
        {
        case RKNN_TENSOR_INT8:
            break;
        case RKNN_TENSOR_UINT8:
            break;        
        default:
            all_int8 = false;
            break;
        }
    }

    if (all_int8){
        for (int i=0; i < m_info->n_input; i++){
            int n_elems = m_info->in_attr[i].n_elems;
            input_data[i] = (void*)malloc(n_elems * sizeof(char));
            for (int j=0; j<n_elems; j++){
                ((unsigned char*)input_data[i])[j] = rand() % 256;
            }
        }
    }
    else{
        for (int i=0; i < m_info->n_input; i++){
            int n_elems = m_info->in_attr[i].n_elems;
            input_data[i] = (void*)malloc(n_elems * sizeof(float));
            for (int j=0; j<n_elems; j++){
                ((float*)input_data[i])[j] = (float)(rand() % 256) / 256;
                // printf("%f ", ((float*)input_data[i])[j]);
            }
        }
    }
    return all_int8;
}


int input_data_load(void** input_data, const char* input_path, bool* all_int8, MODEL_INFO* m_info){
    int ret = 0;

    std::vector<std::string> input_paths_split;
    input_paths_split = split(input_path, "#");
    
    if (m_info->n_input != input_paths_split.size()) {
        printf("ERROR: input number is not match, input number is %d, but input path is %s\n", m_info->n_input, input_path);
        return -1;
    }

    for (int i=0; i<m_info->n_input; i++){
        if (strstr(input_paths_split[i].c_str(), ".npy")) {
            input_data[i] = load_npy(input_paths_split[i].c_str(), &(m_info->in_attr[i]));
            *all_int8 = false;
        } 
        else {
            LETTER_BOX letter_box;
            input_data[i] = load_image_and_autoresize(input_paths_split[i].c_str(), &letter_box, &(m_info->in_attr[i]));
            *all_int8 = true;
        }
        if (!input_data[i]) {
            return -1;
        }
    }

    return ret;
}


float compute_cosine_similarity(float* a, float* b, int len){
    // printf("len = %d\n", len);
    float dot = 0.0, denom_a = 0.0, denom_b = 0.0 ;
    for (int i = 0; i < len; i++){
        dot += a[i] * b[i];
        denom_a += a[i] * a[i];
        denom_b += b[i] * b[i];
        // printf("%f %f %f\n", dot, denom_a, denom_b);
    }
    return dot / (sqrt(denom_a) * sqrt(denom_b));
}


float compare_npy_cos_similarity(const char* fpath_1, const char* fpath_2, int len){
    void* data_1 = load_npy(fpath_1, NULL);
    void* data_2 = load_npy(fpath_2, NULL);

    float cos = compute_cosine_similarity((float*)data_1, (float*)data_2, len);
    free(data_1);
    free(data_2);
    return cos;
}


int check_output_accuracy(const char* golden_folder, const char* runtime_folder, MODEL_INFO* m_info){
    if (access(golden_folder, F_OK) != 0){
        printf("WARNING: [%s] not exist, skip output check.\n", golden_folder);
        return -1;
    }
    
    int ret = 0;
    printf("CHECK OUTPUT ACCURACY\n");
    for (int i=0; i<m_info->n_output; i++){
        char fpath_1[512];
        char fpath_2[512];
        sprintf(fpath_1, "%s/%s.npy", golden_folder, m_info->out_attr[i].name);
        sprintf(fpath_2, "%s/%s.npy", runtime_folder, m_info->out_attr[i].name);
        float cos = compare_npy_cos_similarity(fpath_1, fpath_2, m_info->out_attr[i].n_elems);
        printf("  output[%d]-[%s] cosine similarity: %f\n", i, m_info->out_attr[i].name, cos);
    }
    return 0;
}

int run_rkdemo(MODEL_INFO* m_info, void** input_data, int loop_time){
    // rknn - input_set, run, output_get. Calculate the average time of the repeat times. Print the result.
    int ret = 0;
    TIMER timer;
    timer.start();
    printf("RUNING rknn model.\nRun time for first loop\n");
    for (int _loop=0; _loop<loop_time; _loop++){
        TIMER step_timer;
        step_timer.indent_set("  ");

        step_timer.start();
        for (int input_index=0; input_index<m_info->n_input; input_index++){
            m_info->inputs[input_index].buf = input_data[input_index];
        }
        ret = rknn_inputs_set(m_info->ctx, m_info->n_input, m_info->inputs);
        check_ret(ret, "rknn_inputs_set");
        step_timer.stop();
        if (_loop==0){ step_timer.print_time("rknn_inputs_set");}

        step_timer.start();
        ret = rknn_run(m_info->ctx, NULL);
        check_ret(ret, "rknn_run");
        step_timer.stop();
        if (_loop==0){ step_timer.print_time("rknn_run");}

        step_timer.start();
        ret = rknn_outputs_get(m_info->ctx, m_info->n_output, m_info->outputs, NULL);
        check_ret(ret, "rknn_outputs_get");
        step_timer.stop();
        if (_loop==0){ step_timer.print_time("rknn_outputs_get");}

        if (_loop != loop_time-1){
            ret = rknn_outputs_release(m_info->ctx, m_info->n_output, m_info->outputs);
            check_ret(ret, "rknn_outputs_release");
        }
    }
    timer.stop();
    if (loop_time!= 1){ printf("For %d loop. Average total time %.2f ms\n", loop_time, timer.get_time()/loop_time);}

    return 0;
}


int main(int argc, char* argv[]){
    int ret = 0;
    
    // argv[1] is the path to the model
    // argv[2] is the path to the image
    // argv[3] is repeat times(given as int)
    // TODO argv[4] support normal_api / zerocopy_api 
    // TODO check if int8/int32/float16 input is available for this demo
    if (argc < 2 ){
        printf("Usage: ./rknn_demo model_path image_path repeat_times\n");
        printf("  Example: ./rknn_demo ./yolo.rknn ./bus.jpg 100\n");
        printf("  Example(Multi-input): ./rknn_demo ./transformer.rknn ./token.npy#./mask.npy 100\n");
        return -1;
    }


    // model init
    MODEL_INFO m_info;
    memset(&m_info, 0, sizeof(MODEL_INFO));
    m_info.m_path = argv[1];
    rkdemo_init(&m_info);


    // input load
    printf("LOADING INPUT DATA\n");
    void** input_data;
    bool all_int8 = true;
    input_data = (void**)malloc(sizeof(void*) * m_info.n_input);
    if (argc > 2){
        ret = input_data_load(input_data, argv[2], &all_int8, &m_info);
        check_ret(ret, "input_data_load");
    }
    else{
        // random gen data
        printf("input was not given, random generate input\n");
        all_int8 = random_generate_input(input_data, &m_info);
    }


    // init input/output
    printf("INIT RKNN INPUT OUTPUT\n");
    if (all_int8){
        rkdemo_init_input_buffer_all(&m_info, NORMAL_API, RKNN_TENSOR_UINT8);
    }
    else{
        rkdemo_init_input_buffer_all(&m_info, NORMAL_API, RKNN_TENSOR_FLOAT32);
    }
    uint8_t want_float = 1;
    rkdemo_init_output_buffer_all(&m_info, NORMAL_API, want_float);


    // rknn rum
    int loop_time = 1;
    if (argc == 4){
        loop_time = atoi(argv[3]);
    }
    ret = run_rkdemo(&m_info, input_data, loop_time);
    check_ret(ret, "run_rkdemo");


    // save output result as npy
    if (argc > 2){
        const char* save_path = "./data/outputs/runtime";
        ret = save_rknn_result(save_path, &m_info);

        // check output if available.
        const char* golden_folder = "./data/outputs/golden";
        ret = check_output_accuracy(golden_folder, save_path, &m_info);
    }
    else{
        printf("Inputs was not given, skip save output\n");
    }
    

    // release input data
    for (int input_index=0; input_index< m_info.n_input; input_index++){
        free(input_data[input_index]);
    }
    free(input_data);


    // release rknn
    rknn_outputs_release(m_info.ctx, m_info.n_output, m_info.outputs);
    rkdemo_release(&m_info);

    return 0;
}