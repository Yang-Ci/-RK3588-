#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <assert.h>

#include <rknn_api.h>
#include <rknn_demo_utils.h>


#ifdef RKDMO_NPU_1 
const char* get_format_string(rknn_tensor_format fmt)
{
    switch(fmt) {
    case RKNN_TENSOR_NCHW: return "NCHW";
    case RKNN_TENSOR_NHWC: return "NHWC";
    default: return "UNKNOW";
    }
}

const char* get_type_string(rknn_tensor_type type)
{
    switch(type) {
    case RKNN_TENSOR_FLOAT32: return "FLOAT32";
    case RKNN_TENSOR_FLOAT16: return "FLOAT16";
    case RKNN_TENSOR_INT8: return "INT8";
    case RKNN_TENSOR_UINT8: return "UINT8";
    case RKNN_TENSOR_INT16: return "INT16";
    default: return "UNKNOW";
    }
}

const char* get_qnt_type_string(rknn_tensor_qnt_type qnt_type)
{
    switch(qnt_type) {
    case RKNN_TENSOR_QNT_NONE: return "NONE";
    case RKNN_TENSOR_QNT_DFP: return "QNT_DFP";
    case RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC: return "AFFINE_ASYMMETRIC";
    default: return "UNKNOW";
    }
}

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{

    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
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

#endif

int offset_nchw_2_nc1hwc2(rknn_tensor_attr *src_attr, rknn_tensor_attr *native_attr, int offset, bool batch){
    int h = src_attr->n_dims > 2 ? src_attr->dims[2] : 1;
    int w = src_attr->n_dims > 3 ? src_attr->dims[3] : 1;

    int C = src_attr->dims[1];
    int C1 = native_attr->dims[1];
    int C2 = native_attr->dims[2];

    int hw = h*w;
    int Chw = hw*C;
    int hwC2 = hw*C2;
    int C1hwC2 = C1*hwC2;

    int native_offset = 0;
    if (batch==true){
        int n_index = offset / Chw;
        native_offset = n_index * C1hwC2;
        offset = offset - n_index * Chw;
    }

    int c_offset = offset / (hw);
    int hw_offset = offset % (hw);

    int c1_offset = c_offset / C2;
    int c2_offset = c_offset % C2;

    native_offset = native_offset + 
                        c1_offset * hwC2 +
                        hw_offset * C2 +
                        c2_offset;

    return native_offset;
}


int offset_nc1hwc2_2_nchw(rknn_tensor_attr *src_attr, rknn_tensor_attr *native_attr, int offset, bool batch){
    printf("offset_nc1hwc2_2_nchw not support");
    return 0;
}


// static void dump_tensor_attr(rknn_tensor_attr *attr)
void dump_tensor_attr(rknn_tensor_attr *attr)
{
    char dims_str[100];
    memset(dims_str, 0, sizeof(dims_str));
    for (int i = 0; i < attr->n_dims; i++) {
        sprintf(dims_str, "%s%d, ", dims_str, attr->dims[i]);
    }

    printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, size_with_stride=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, dims_str,
           attr->n_elems, attr->size, attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}


int rkdemo_init(MODEL_INFO* model_info){
    if (model_info->m_path == nullptr){
        printf("ERROR model path is null");
        return -1;
    }

    int ret = 0;
#ifdef RKDMO_NPU_1
    int model_data_size = 0;
    unsigned char *model_data = load_model(model_info->m_path, &model_data_size);
    ret = rknn_init(&model_info->ctx, model_data, model_data_size, 0);
#else
    ret = rknn_init(&model_info->ctx, (void *)model_info->m_path, 0, 0, NULL);
#endif

    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }
    ret = rkdemo_query_model_info(model_info);
    return ret;
}

int rkdemo_init_share_weight(MODEL_INFO* model_info, MODEL_INFO* src_model_info){
    if (model_info->m_path == nullptr){
        printf("ERROR model path is null");
        return -1;
    }

    int ret = 0;
#ifndef RKDMO_NPU_1
    rknn_init_extend extend;
    memset(&extend,0,sizeof(rknn_init_extend));
    extend.ctx = src_model_info->ctx;

    ret = rknn_init(&model_info->ctx, (void *)model_info->m_path, 0, RKNN_FLAG_SHARE_WEIGHT_MEM, &extend);
    if (ret < 0) {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }
#endif

    ret = rkdemo_query_model_info(model_info);
    return ret;
}


int rkdemo_query_model_info(MODEL_INFO* model_info){
    printf("rkdemo_query_model_info");
    int ret=0;

    rknn_input_output_num io_num;
    ret = rknn_query(model_info->ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);
    model_info->n_input = io_num.n_input;
    model_info->n_output = io_num.n_output;


    model_info->inputs = (rknn_input*)malloc(sizeof(rknn_input) * model_info->n_input);
    model_info->in_attr = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * model_info->n_input);
    model_info->in_attr_native = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * model_info->n_input);
    model_info->input_mem = (rknn_tensor_mem**)malloc(sizeof(rknn_tensor_mem*) * model_info->n_input);
    model_info->rkdmo_input_param = (RKDEMO_INPUT_PARAM*)malloc(sizeof(RKDEMO_INPUT_PARAM) * model_info->n_input);
    for (int i = 0; i < model_info->n_input; i++)
    {
        memset(&(model_info->inputs[i]), 0, sizeof(rknn_input));
        memset(&(model_info->rkdmo_input_param[i]), 0, sizeof(RKDEMO_INPUT_PARAM));
    }
    
    model_info->outputs = (rknn_output*)malloc(sizeof(rknn_output) * model_info->n_output);
    model_info->out_attr = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * model_info->n_output);
    model_info->out_attr_native = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr) * model_info->n_output);
    model_info->output_mem = (rknn_tensor_mem**)malloc(sizeof(rknn_tensor_mem*) * model_info->n_output);
    model_info->rkdmo_output_param = (RKDEMO_OUTPUT_PARAM*)malloc(sizeof(RKDEMO_OUTPUT_PARAM) * model_info->n_output);
    for (int i = 0; i < model_info->n_output; i++)
    {
        memset(&(model_info->outputs[i]), 0, sizeof(rknn_output));
        memset(&(model_info->rkdmo_output_param[i]), 0, sizeof(RKDEMO_OUTPUT_PARAM));
    }

    printf("INPUTS ATTR:\n");
    for (int i = 0; i < model_info->n_input; i++)
    {
        model_info->in_attr[i].index = i;
        ret = rknn_query(model_info->ctx, RKNN_QUERY_INPUT_ATTR, &(model_info->in_attr[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&model_info->in_attr[i]);

#ifdef RKDMO_NPU_2_NATIVE_ZP
        model_info->in_attr_native[i].index = i;
        ret = rknn_query(model_info->ctx, RKNN_QUERY_INPUT_ATTR, &(model_info->in_attr_native[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        printf("native input attr: size_with_stride %d;\n", model_info->in_attr_native[i].size_with_stride);
        printf("native input attr: w_stride %d;\n", model_info->in_attr_native[i].w_stride);
        printf("native input attr: width %d;\n", model_info->in_attr_native[i].dims[2]);
        // dump_tensor_attr(&model_info->in_attr[i]);
#endif

    }

    printf("OUTPUTS ATTR:\n");
    for (int i = 0; i < model_info->n_output; i++)
    {
        model_info->out_attr[i].index = i;
        ret = rknn_query(model_info->ctx, RKNN_QUERY_OUTPUT_ATTR, &(model_info->out_attr[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&model_info->out_attr[i]);

#ifdef RKDMO_NPU_2_NATIVE_ZP
        model_info->out_attr_native[i].index = i;
        // ret = rknn_query(model_info->ctx, RKNN_QUERY_NATIVE_NC1HWC2_OUTPUT_ATTR, &(model_info->out_attr_native[i]), sizeof(rknn_tensor_attr));
        ret = rknn_query(model_info->ctx, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR, &(model_info->out_attr_native[i]), sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&model_info->out_attr_native[i]);
#endif
    }

    return 0;
}


int rkdemo_get_type_size(rknn_tensor_type type){
    switch (type)
    {
    case RKNN_TENSOR_FLOAT32: return 4;
    case RKNN_TENSOR_FLOAT16: return 2;
    case RKNN_TENSOR_UINT8: return 1;
    case RKNN_TENSOR_INT8: return 1;
    default: printf("ERROR: not support tensor type %s", get_type_string(type)); return -1;
    }
}


int rkdemo_init_input_buffer(MODEL_INFO* model_info, int node_index, API_TYPE api_type, uint8_t pass_through, rknn_tensor_type dtype, rknn_tensor_format layout_fmt){
    if (model_info->rkdmo_input_param[node_index]._already_init){
        printf("ERROR model input buffer already init");
        return -1;
    }
    model_info->rkdmo_input_param[node_index]._already_init = true;
    model_info->rkdmo_input_param[node_index].api_type = api_type;
    int elem_size = rkdemo_get_type_size(dtype);

    if (api_type==NORMAL_API){
        model_info->inputs[node_index].index = node_index;
        model_info->inputs[node_index].pass_through = pass_through;
        model_info->inputs[node_index].type = dtype;
        model_info->inputs[node_index].fmt = layout_fmt;
        model_info->inputs[node_index].size = model_info->in_attr[node_index].n_elems* elem_size;
        printf("rkdemo_init_input_buffer(n_api): node_index=%d, size=%d, n_elems=%d, fmt=%s, type=%s\n", 
                node_index, model_info->inputs[node_index].size, model_info->in_attr[node_index].n_elems, 
                get_format_string(layout_fmt), 
                get_type_string(dtype));
    }
    else if (api_type==ZERO_COPY_API){
        // assert(dtype==RKNN_TENSOR_UINT8);
        // assert((model_info->in_attr[node_index].type==RKNN_TENSOR_UINT8) || (model_info->in_attr[node_index].type==RKNN_TENSOR_INT8));
        model_info->in_attr[node_index].fmt = layout_fmt;
        model_info->in_attr[node_index].type = dtype;
#ifdef RKDMO_NPU_1
        model_info->input_mem[node_index] = rknn_create_mem(model_info->ctx, model_info->in_attr[node_index].size);
#else
        model_info->input_mem[node_index] = rknn_create_mem(model_info->ctx, model_info->in_attr[node_index].size_with_stride);
#endif 
        // printf("not support zero-copy\n");
        printf("rkdemo_init_input_buffer(zero copy): node_index=%d, size %d, size with stride %d, fmt=%s, type=%s\n", 
                node_index, 
                model_info->in_attr[node_index].size,
                model_info->in_attr[node_index].size_with_stride,
                get_format_string(layout_fmt),
                get_type_string(dtype));
    }
    return 0;
}


int rkdemo_init_output_buffer(MODEL_INFO* model_info, int node_index, API_TYPE api_type, uint8_t want_float){
    if (model_info->rkdmo_output_param[node_index]._already_init){
        printf("ERROR model output buffer already init");
        return -1;
    }
    model_info->rkdmo_output_param[node_index]._already_init = true;
    model_info->rkdmo_output_param[node_index].api_type = api_type;

    if (api_type==NORMAL_API){
        model_info->outputs[node_index].index = node_index;
        model_info->outputs[node_index].want_float = want_float;
        printf("rkdemo_init_output_buffer(n_api): node_index=%d, want_float=%d\n", node_index, want_float);
    }
    else if(api_type==ZERO_COPY_API){
        // assert(want_float==0); // toolkit1 not support, toolkit2 support zero_copy as float output

#ifdef RKDMO_NPU_2_NATIVE_ZP
        if (want_float){
            model_info->out_attr_native[node_index].type = RKNN_TENSOR_FLOAT32;
        }
        // model_info->out_attr_native[node_index].fmt = RKNN_TENSOR_NHWC;
        // model_info->out_attr_native[node_index].fmt = RKNN_TENSOR_NC1HWC2;
        int elem_size = rkdemo_get_type_size(model_info->out_attr_native[node_index].type);
        model_info->output_mem[node_index] = rknn_create_mem(model_info->ctx, model_info->out_attr_native[node_index].n_elems*elem_size);
        printf("size with stride %d\n", model_info->out_attr_native[node_index].size_with_stride);
        printf("size %d\n", model_info->out_attr_native[node_index].size);
#else
        if (want_float){
            model_info->out_attr[node_index].type = RKNN_TENSOR_FLOAT32;
        }
        int elem_size = rkdemo_get_type_size(model_info->out_attr[node_index].type);
        model_info->output_mem[node_index] = rknn_create_mem(model_info->ctx, model_info->out_attr[node_index].n_elems*elem_size);
        printf("rkdemo_init_output_buffer(zero copy): node_index=%d, size with stride %d\n", node_index, model_info->out_attr[node_index].size_with_stride);
#endif

        // printf("not support zero-copy\n");
    }
    return 0;
}


int rkdemo_init_input_buffer_all(MODEL_INFO* model_info, API_TYPE default_api_type, rknn_tensor_type default_t_type){
    rknn_tensor_format default_layout_fmt = RKNN_TENSOR_NHWC;

    uint8_t default_pass_through = 0;

    for (int i = 0; i < model_info->n_input; i++){
        if (model_info->rkdmo_input_param[i]._already_init){
            printf("model input buffer already init, ignore");
            continue;
        }

        if (model_info->rkdmo_input_param[i].enable){
            rkdemo_init_input_buffer(model_info, 
                                     i, 
                                     model_info->rkdmo_input_param[i].api_type, 
                                     model_info->rkdmo_input_param[i].pass_through, 
                                     model_info->rkdmo_input_param[i].dtype, 
                                     model_info->rkdmo_input_param[i].layout_fmt);
        }
        else{
            if (model_info->in_attr[i].n_dims==4){
                default_layout_fmt = RKNN_TENSOR_NHWC;
            }
            else{
#ifdef RKDMO_NPU_1
                default_layout_fmt = RKNN_TENSOR_NHWC;
#else
                default_layout_fmt = RKNN_TENSOR_UNDEFINED;
#endif
            }
            
            rkdemo_init_input_buffer(model_info, i, default_api_type, default_pass_through, default_t_type, default_layout_fmt);
        }
    }
    return 0;
}


int rkdemo_init_output_buffer_all(MODEL_INFO* model_info, API_TYPE default_api_type, uint8_t default_want_float){
    for (int i = 0; i < model_info->n_output; i++){
        if (model_info->rkdmo_output_param[i]._already_init){
            printf("model output buffer already init, ignore");
            continue;
        }

        if (model_info->rkdmo_output_param[i].enable){
            rkdemo_init_output_buffer(model_info, i, model_info->rkdmo_output_param[i].api_type, model_info->rkdmo_output_param[i].want_float);
        }
        else{
            rkdemo_init_output_buffer(model_info, i, default_api_type, default_want_float);
        }
    }
    return 0;
}



// order is: x -> model_top -> model_bottom -> result
int rkdemo_connect_models_node(MODEL_INFO* model_top, int top_out_index, MODEL_INFO* model_bottom, int bottom_in_index){
    assert (model_top->rkdmo_output_param[top_out_index]._already_init);    //  "model_top output not init ready"
    assert (model_bottom->rkdmo_input_param[bottom_in_index]._already_init);  //  "model_bottom input not init ready"
    assert (model_top->out_attr[top_out_index].type == model_bottom->in_attr[bottom_in_index].type); // "model_top output type not match model_bottom input type"
    assert (model_top->out_attr[top_out_index].fmt == model_bottom->in_attr[bottom_in_index].fmt); // "model_top output n_elems not match model_bottom input n_elems"

    model_top->outputs[top_out_index].want_float = 0;

    int byte = 0;
    if (model_bottom->in_attr[bottom_in_index].type == RKNN_TENSOR_FLOAT16){
        byte = 4;
    }
    else if (model_bottom->in_attr[bottom_in_index].type == RKNN_TENSOR_UINT8){
        byte = 1;
    }
    model_bottom->inputs[bottom_in_index].type = model_bottom->in_attr[bottom_in_index].type;
    model_bottom->inputs[bottom_in_index].size = model_bottom->in_attr[bottom_in_index].n_elems*byte;
    model_bottom->inputs[bottom_in_index].fmt  = model_top->out_attr[top_out_index].fmt;
    // model_bottom->inputs[bottom_in_index].pass_through = 1;

    printf("connect top_model output-%d to botten_model input-%d", top_out_index, bottom_in_index);
    return 0;
}


int rkdemo_release(MODEL_INFO* model_info){
    // TODO support zero cpoy
    for (int i = 0; i < model_info->n_input; i++){
        if (model_info->rkdmo_input_param[i].api_type == ZERO_COPY_API){
            rknn_destroy_mem(model_info->ctx, model_info->input_mem[i]);
        }
    }

    for (int i = 0; i < model_info->n_output; i++){
        if (model_info->rkdmo_output_param[i].api_type == ZERO_COPY_API){
            rknn_destroy_mem(model_info->ctx, model_info->output_mem[i]);
        }
    }

    if (model_info->ctx>0){
        rknn_destroy(model_info->ctx);
    }

    free(model_info->inputs);
    free(model_info->in_attr);
    free(model_info->in_attr_native);
    free(model_info->input_mem);
    free(model_info->rkdmo_input_param);

    free(model_info->outputs);
    free(model_info->out_attr);
    free(model_info->out_attr_native);
    free(model_info->output_mem);
    free(model_info->rkdmo_output_param);

    return 0;
}
