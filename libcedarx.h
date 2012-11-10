
#ifndef LIBCEDARX_H
#define LIBCEDARX_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef u8
    typedef unsigned char u8;    
#endif
#ifndef u16
    typedef unsigned short u16;    
#endif
#ifndef u32
    typedef unsigned int u32;    
#endif
#ifndef u64
    #ifdef COMPILER_ARMCC
    	typedef unsigned __int64 u64;
    #else
    	typedef unsigned long long u64;
    #endif
#endif
#ifndef s8
    typedef signed char s8;    
#endif
#ifndef s16
    typedef signed short s16;    
#endif
#ifndef s32
    typedef signed int s32;    
#endif
#ifndef s64
    #ifdef COMPILER_ARMCC
    	typedef signed __int64 s64;
    #else
    	typedef signed long long s64;
    #endif
#endif
#ifndef Handle
    typedef void* Handle;
#endif
#ifndef NULL
    #define NULL ((void*)0)
#endif

typedef enum CEDARX_RESULT
{
    CEDARX_RESULT_OK                      = 0, 
    CEDARX_RESULT_NO_INIT                 = -1, 
    CEDARX_RESULT_NO_ENOUGH_MEMORY        = -2,
    CEDARX_RESULT_INVALID_ARGS            = -3, 
    CEDARX_RESULT_VBV_BUFFER_FULL         = -4, 
    CEDARX_RESULT_VBV_BUFFER_NO_ENOUGH    = -5, 
    CEDARX_RESULT_VBV_BUFFER_EMPTY        = -6, 
    CEDARX_RESULT_NO_FRAME_BUFFER         = -7, 
    CEDARX_RESULT_NO_DISPLAY_BUFFER       = -8, 
    CEDARX_RESULT_VE_FAILED               = -9,
    CEDARX_RESULT_DE_FAILED               = -10,
    CEDARX_RESULT_NO_RESOURCE             = -11, 
}cedarx_result_e;

typedef enum CEDARV_STREAM_FORMAT
{
    CEDARX_STREAM_FORMAT_UNKNOWN,
    CEDARX_STREAM_FORMAT_MPEG1,
    CEDARX_STREAM_FORMAT_MPEG2,
    CEDARX_STREAM_FORMAT_XVID,
    CEDARX_STREAM_FORMAT_REALVIDEO,
    CEDARX_STREAM_FORMAT_H263,
    CEDARX_STREAM_FORMAT_SORENSSON_H263,
    CEDARX_STREAM_FORMAT_H264,
    CEDARX_STREAM_FORMAT_DIVX3,
    CEDARX_STREAM_FORMAT_DIVX4,
    CEDARX_STREAM_FORMAT_DIVX5,
    CEDARX_STREAM_FORMAT_VC1,
    CEDARX_STREAM_FORMAT_AVS,
    CEDARX_STREAM_FORMAT_MJPEG,  
    CEDARX_STREAM_FORMAT_WMV1,
    CEDARX_STREAM_FORMAT_WMV2,
    CEDARX_STREAM_FORMAT_VP6,
    CEDARX_STREAM_FORMAT_VP8,
    CEDARX_STREAM_FORMAT_DIVX2,		//MSMPEGV2
    CEDARX_STREAM_FORMAT_DIVX1		//MSMPEGV1
}cedarx_stream_format_e;

typedef enum CEDARV_CONTAINER_FORMAT
{
    CEDARX_CONTAINER_FORMAT_UNKNOW,
    CEDARX_CONTAINER_FORMAT_AVI,
    CEDARX_CONTAINER_FORMAT_ASF,
    CEDARX_CONTAINER_FORMAT_DAT,
    CEDARX_CONTAINER_FORMAT_FLV,
    CEDARX_CONTAINER_FORMAT_MKV,
    CEDARX_CONTAINER_FORMAT_MOV,
    CEDARX_CONTAINER_FORMAT_MPG,
    CEDARX_CONTAINER_FORMAT_PMP,
    CEDARX_CONTAINER_FORMAT_RM,
    CEDARX_CONTAINER_FORMAT_TS,
    CEDARX_CONTAINER_FORMAT_VOB,
    CEDARX_CONTAINER_FORMAT_WEBM,
    CEDARX_CONTAINER_FORMAT_OGM
}cedarx_container_format_e;

typedef enum CEDARV_PIXEL_FORMAT
{
    CEDARX_PIXEL_FORMAT_1BPP       = 0x0,
    CEDARX_PIXEL_FORMAT_2BPP       = 0x1,
    CEDARX_PIXEL_FORMAT_4BPP       = 0x2,
    CEDARX_PIXEL_FORMAT_8BPP       = 0x3,
    CEDARX_PIXEL_FORMAT_RGB655     = 0x4,
    CEDARX_PIXEL_FORMAT_RGB565     = 0x5,
    CEDARX_PIXEL_FORMAT_RGB556     = 0x6,
    CEDARX_PIXEL_FORMAT_ARGB1555   = 0x7,
    CEDARX_PIXEL_FORMAT_RGBA5551   = 0x8,
    CEDARX_PIXEL_FORMAT_RGB888     = 0x9,
    CEDARX_PIXEL_FORMAT_ARGB8888   = 0xa,
    CEDARX_PIXEL_FORMAT_YUV444     = 0xb,
    CEDARX_PIXEL_FORMAT_YUV422     = 0xc,
    CEDARX_PIXEL_FORMAT_YUV420     = 0xd,
    CEDARX_PIXEL_FORMAT_YUV411     = 0xe,
    CEDARX_PIXEL_FORMAT_CSIRGB     = 0xf,
    CEDARX_PIXEL_FORMAT_AW_YUV420  = 0x10,
    CEDARX_PIXEL_FORMAT_AW_YUV422	= 0x11,
    CEDARX_PIXEL_FORMAT_AW_YUV411  = 0x12
}cedarx_pixel_format_e;
   
typedef struct
{
    cedarx_stream_format_e stream;
    cedarx_container_format_e container;
    u32 frame_rate;
    u32 frame_duration;
    u32 width;
    u32 height; 
    u8* data;
    u32 data_size;
}cedarx_info_t;

cedarx_result_e libcedarx_decoder_open(cedarx_info_t* info);
void libcedarx_decoder_close(void);
cedarx_result_e libcedarx_decoder_add_stream(u8* buf, u32 size, u64 pts, u64 pcr);
cedarx_result_e libcedarx_decoder_decode_stream(void);
cedarx_result_e libcedarx_decoder_flush(void);
cedarx_result_e libcedarx_display_open(void);
void libcedarx_display_close(void);
cedarx_result_e libcedarx_display_request_layer(void);
void libcedarx_display_release_layer(void);
cedarx_result_e libcedarx_display_video_frame(int idx);
cedarx_result_e libcedarx_display_request_frame(u32 *id, 
            u64 *pts, u32 *frame_rate, u32 *width, u32 *height);
cedarx_result_e libcedarx_display_return_frame(int id);
char* libcedarx_version(void);

#ifdef __cplusplus
}
#endif

#endif


