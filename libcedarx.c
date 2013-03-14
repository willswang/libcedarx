#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "avheap.h"
#include "libve_typedef.h"
#include "libve.h"
#include "libve_adapter.h"
#include "cedardev_api.h"
#include "drv_display.h"
#include "libcedarx.h"

#define LIBCEDARX_VERSION               "version 1.0.0"

#define VBV_FRAME_SIZE                      (8*1024*1024)
#define VBV_MAX_FRAME_NUM                   64
#define VBV_MIN_FRAME_NUM                   2
#define FBM_MAX_FRAME_NUM                   64
#define MAX_SUPPORTED_VIDEO_WIDTH           1920
#define MAX_SUPPORTED_VIDEO_HEIGHT          1088
#define MAX_SUPPORTED_OUTPUT_WIDTH          1920
#define MAX_SUPPORTED_OUTPUT_HEIGHT         1080


typedef struct
{
    Handle  ve;
    Handle  fbm;
    Handle  vbv;
    Handle  sys;
    u8* init_data;
    u32 address;
    u32 greedy;
    int fd;
    pthread_mutex_t mutex;
    void (*request_buffer)(cedarx_picture_t * pic, void *sys);
    void (*update_buffer)(cedarx_picture_t * pic, void *sys);
    void (*release_buffer)(cedarx_picture_t * pic, void *sys);
    void (*lock_buffer)(cedarx_picture_t * pic, void *sys);
    void (*unlock_buffer)(cedarx_picture_t * pic, void *sys);    
}cedarx_decoder_t;

typedef struct
{
    int fd;
    struct {
        int enable;
        int layer;
        int start;
        int init;
        int top;
        int last_id;
    } info;
    u32 idx;
    pthread_mutex_t mutex;
}cedarx_display_t;

typedef struct STREAM_FRAME_T stream_frame_t;
struct STREAM_FRAME_T
{
    vstream_data_t  vstream;
    stream_frame_t* next;
};

typedef struct
{
    stream_frame_t* in_frames;
    u32 write_index;
    u32 frame_num;
    u32 max_frame_num;
}stream_fifo_t;

typedef struct
{
    stream_frame_t* head;
    u32 frame_num;
}stream_queue_t;

typedef struct
{
    u8* vbv_buf;
    u8* vbv_buf_end;
    u8* write_addr;
    u32 valid_size;
    u32 max_size;
    stream_fifo_t frame_fifo;
    stream_queue_t frame_queue;    
    pthread_mutex_t mutex;
}vbv_t;

typedef enum
{
    FS_INVALID                       = 0,
    FS_EMPTY                         = 1,
    FS_BINDING                       = 2,
    FS_DECODER_USING                 = 3,
    FS_DECODER_DISCARD               = 4,
    FS_DECODER_SHARED                = 5,
    FS_DECODER_SHARE_DISCARD         = 6
}frame_status_e;

typedef struct DISPLAY_FRAME_T display_frame_t;
struct DISPLAY_FRAME_T
{
    vpicture_t vpicture;
    frame_status_e status;
    cedarx_picture_t picture;
    display_frame_t* next;
};

typedef struct
{
    display_frame_t* head;
    u32 frame_num;
}display_queue_t;

typedef struct
{
    u32 init_frame_num;
    u32 exhausted;
    u32 size_y[2];
    u32 size_u[2];
    u32 size_v[2];
    u32 size_alpha[2];
    _3d_mode_e mode;
    display_queue_t empty_queue;
    display_queue_t bind_queue;
    display_queue_t decoded_queue;
    display_frame_t frames[FBM_MAX_FRAME_NUM];
    pthread_mutex_t mutex;
    cedarx_decoder_t *decoder;
}fbm_t;

static cedarx_decoder_t *cedarx_decoder = NULL;
static cedarx_display_t *cedarx_display = NULL;

static void mem_init(int fd)
{
    av_heap_init(fd);
}

static void mem_exit(void)
{
    av_heap_release();
}

static void* mem_alloc(u32 size)
{
    return malloc(size);
}

static void  mem_free(void* p)
{
    free(p);
}

void* mem_palloc(u32 size, u32 align)
{
    void *p;
    p = (void*)av_heap_alloc(size);
    return p; 
}

void mem_pfree(void* p)
{
    av_heap_free(p); 
}

static void  mem_set(void* mem, u32 value, u32 size)
{
    memset(mem, value, size);
}

static void  mem_cpy(void* dst, void* src, u32 size)
{
    memcpy(dst, src, size);
}

static void  mem_flush_cache(u8* mem, u32 size)
{
}

static u32 mem_phy_addr(u32 virtual_addr)
{
    if (virtual_addr) 
        return (u32)av_heap_physic_addr((void*)virtual_addr);
    
    return 0;
}

static s32 sys_print(u8* func, u32 line, ...)
{
    return 0;
}

static void sys_sleep(u32 ms)
{
    usleep(ms * 1000);
}

IOS_t IOS = 
{
    //* Heap operation.
    mem_alloc,
    mem_free,
    mem_palloc,
    mem_pfree,
    
    //* Memory operation.
    mem_set,
    mem_cpy,
    mem_flush_cache,
    mem_phy_addr,

    //* MISC functions.
    sys_print,
    sys_sleep,
};


void ve_request(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    if (decoder && decoder->fd != -1) {
        ioctl(decoder->fd, IOCTL_ENGINE_REQ, 0);
    }
}

void ve_release(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    if (decoder && decoder->fd != -1) {
        ioctl(decoder->fd, IOCTL_ENGINE_REL, 0);
    }
}

static cedarx_result_e ve_init(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder) {
        decoder->fd = open("/dev/cedar_dev", O_RDWR);
        if(decoder->fd != -1) {
            cedarv_env_info_t cedarx_env_info;
            ioctl(decoder->fd, IOCTL_GET_ENV_INFO, (unsigned long)&cedarx_env_info);
            
            decoder->address = (unsigned int)mmap(NULL, 2048, PROT_READ | PROT_WRITE, 
                  MAP_SHARED, decoder->fd, (unsigned long)cedarx_env_info.address_macc);
            return CEDARX_RESULT_OK;
        }
    }  

    return CEDARX_RESULT_VE_FAILED;
}

static void ve_exit(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    if (decoder && decoder->fd != -1) {
        munmap((void *)decoder->address, 2048);
        close(decoder->fd);
    }
}

static void ve_reset_hardware(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder && decoder->fd != -1) {
        ioctl(decoder->fd, IOCTL_RESET_VE, 0);
    }
}

static void ve_enable_clock(u8 enable, u32 speed)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder && decoder->fd != -1) {
        if(enable) {
            ioctl(decoder->fd, IOCTL_ENABLE_VE, 0);
            ioctl(decoder->fd, IOCTL_SET_VE_FREQ, speed/1000000);
        } else {
          ioctl(decoder->fd, IOCTL_DISABLE_VE, 0);
        }
    }
}

static void ve_enable_intr(u8 enable)
{
    /*
     * cedar_dev.ko dose not support this operation, leave this method to be empty here.
     */
}

static s32 ve_wait_intr(void)
{
    s32 status = 0;
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder && decoder->fd != -1) {
        status = ioctl(decoder->fd, IOCTL_WAIT_VE, 2);
    }
    
    return (status - 1);
}


static u32 ve_get_reg_base_addr(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder)
        return decoder->address;
    return 0;
}


static memtype_e ve_get_memtype(void)
{
    return MEMTYPE_DDR3_32BITS; //* return the DRAM type.
}

IVEControl_t IVE = 
{
    ve_reset_hardware,
    ve_enable_clock,
    ve_enable_intr,
    ve_wait_intr,
    ve_get_reg_base_addr,
    ve_get_memtype
};

static void vbv_enqueue_head(stream_queue_t* q, stream_frame_t* stream)
{
    if (q && stream) {
        stream->next = q->head;
        q->head = stream;
        q->frame_num++;
    }
}

static void vbv_enqueue_tail(stream_queue_t* q, stream_frame_t* stream)
{
    stream_frame_t* node;
    
    if (q && stream) {
        node = q->head;
        if(node) {
            while(node->next != NULL)
                node = node->next;
            
            node->next = stream;
            stream->next = NULL;
        } else {
            q->head = stream;
            stream->next = NULL;
        }
        q->frame_num ++;
    }
}

static stream_frame_t* vbv_dequeue(stream_queue_t* q)
{
    stream_frame_t* node = NULL;
    
    if (q) {
        node = q->head;
        if(node) {
            q->head = q->head->next;
            node->next = NULL;
            q->frame_num--;
        }
    }

    return node;
}

static Handle vbv_init(void)
{
    int i;
    u8* vbv_buf;
    vbv_t* vbv;

    vbv = (vbv_t*)mem_alloc(sizeof(vbv_t));
    if (vbv) {
        mem_set(vbv, 0, sizeof(vbv_t));
        
        vbv->frame_fifo.in_frames = (stream_frame_t*)mem_alloc(VBV_MAX_FRAME_NUM * sizeof(stream_frame_t));
        if (!vbv->frame_fifo.in_frames) {
            mem_free(vbv);
            return NULL;
        }
        
        mem_set(vbv->frame_fifo.in_frames, 0, VBV_MAX_FRAME_NUM * sizeof(stream_frame_t));
        for(i = 0; i < VBV_MAX_FRAME_NUM; i++)
        {
            vbv->frame_fifo.in_frames[i].vstream.id = i;
        }
                
        vbv_buf = (u8*)mem_palloc(VBV_FRAME_SIZE, 1024);
        if (!vbv_buf) {
            mem_free(vbv); 
            mem_free(vbv->frame_fifo.in_frames); 
            return NULL;
        }
        
        vbv->vbv_buf     = vbv_buf;
        vbv->vbv_buf_end = vbv_buf + VBV_FRAME_SIZE;
        vbv->max_size    = VBV_FRAME_SIZE;
        vbv->write_addr  = vbv_buf;
        vbv->valid_size  = 0;
        
        vbv->frame_fifo.frame_num     = 0;
        vbv->frame_fifo.write_index   = 0;
        vbv->frame_fifo.max_frame_num = VBV_MAX_FRAME_NUM;
        
        vbv->frame_queue.frame_num = 0;
        vbv->frame_queue.head      = NULL;
        
        pthread_mutex_init(&vbv->mutex, NULL);
        return (Handle)vbv;
    }

    return NULL;
}

static void vbv_exit(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;

    if (vbv) {
        pthread_mutex_destroy(&vbv->mutex);

        if (vbv->frame_fifo.in_frames) {
            mem_free(vbv->frame_fifo.in_frames);
        }
        
        if (vbv->vbv_buf) {
            mem_pfree(vbv->vbv_buf);
        }

        mem_free(vbv);
    }
}

static cedarx_result_e vbv_check_stream_frame(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    
    if (vbv->frame_fifo.frame_num > VBV_MIN_FRAME_NUM) {
        return CEDARX_RESULT_OK;
    }

    return CEDARX_RESULT_VBV_BUFFER_EMPTY;    
}

static cedarx_result_e vbv_add_stream_frame(vstream_data_t* stream, Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    stream_frame_t* frame;
    u32 write_index;
    u8* new_write_addr;

    if (!stream || !vbv)
        return CEDARX_RESULT_NO_INIT;
    
    if (vbv->frame_fifo.frame_num >= vbv->frame_fifo.max_frame_num) {
        printf("Warning: The fifo is full!\n");
        return CEDARX_RESULT_VBV_BUFFER_FULL;
    }
    
    if (vbv->valid_size >= vbv->max_size) {
        printf("Warning: The buffer is full!\n");
        return CEDARX_RESULT_VBV_BUFFER_FULL;
    }

    if (stream->length + vbv->valid_size > vbv->max_size) {
        printf("Warning: There are not enough memory!\n");
        return CEDARX_RESULT_VBV_BUFFER_NO_ENOUGH;
    } 
    
    pthread_mutex_lock(&vbv->mutex);

    new_write_addr = vbv->write_addr + stream->length;
    if (new_write_addr >= vbv->vbv_buf_end) { 
      u32 size = vbv->vbv_buf_end - vbv->write_addr;
      mem_cpy(vbv->write_addr, stream->data, size);
      mem_cpy(vbv->vbv_buf, stream->data + size, stream->length - size);
      new_write_addr -= vbv->max_size;
    } else {
      mem_cpy(vbv->write_addr, stream->data, stream->length);
    }

    write_index = vbv->frame_fifo.write_index;
    frame = &vbv->frame_fifo.in_frames[write_index];
    frame->vstream.data = stream->data;
    frame->vstream.length = stream->length;
    frame->vstream.pts = stream->pts;
    frame->vstream.pcr = stream->pcr;
    frame->vstream.valid = stream->valid;
    frame->vstream.stream_type = stream->stream_type;
    frame->vstream.pict_prop = stream->pict_prop;
    frame->vstream.data = vbv->write_addr;
    write_index ++;
    if (write_index >= vbv->frame_fifo.max_frame_num) {
        write_index = 0;
    }
    vbv->frame_fifo.write_index = write_index;
    vbv->frame_fifo.frame_num++;
    vbv->valid_size += stream->length;
    vbv->write_addr = new_write_addr;
    vbv_enqueue_tail(&vbv->frame_queue, frame);     //* add this frame to the queue tail.
    pthread_mutex_unlock(&vbv->mutex);
    return CEDARX_RESULT_OK;
}

static vstream_data_t* vbv_request_stream_frame(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    stream_frame_t* frame;    
    vstream_data_t* stream = NULL;

    if (vbv) {
        pthread_mutex_lock(&vbv->mutex);
        frame = vbv_dequeue(&vbv->frame_queue);
        if (frame) {
            stream = &frame->vstream;
        }
        pthread_mutex_unlock(&vbv->mutex);
    }
    
    return stream;
}


static void vbv_return_stream_frame(vstream_data_t* stream, Handle h)
{
    int i;
    vbv_t* vbv = (vbv_t*)h;
    stream_frame_t* frame = NULL;    
    
    if (vbv && stream) {
        if (vbv->frame_fifo.frame_num > 0) {
            pthread_mutex_lock(&vbv->mutex);
            for(i = 0; i < vbv->frame_fifo.max_frame_num; i++) {
                frame = &vbv->frame_fifo.in_frames[i];
                if (stream == &frame->vstream) {
                    vbv_enqueue_head(&vbv->frame_queue, frame);
                }
            }
            pthread_mutex_unlock(&vbv->mutex);
        }
    }
}

static void vbv_flush_stream_frame(vstream_data_t* stream, Handle h)
{
    int i;
    vbv_t* vbv = (vbv_t*)h;
    stream_frame_t* frame = NULL;    

    if (vbv && stream) {
        if (vbv->frame_fifo.frame_num > 0) {
            pthread_mutex_lock(&vbv->mutex);
            for(i = 0; i < vbv->frame_fifo.max_frame_num; i++) {
                frame = &vbv->frame_fifo.in_frames[i];
                if (stream == &frame->vstream) {
                    vbv->frame_fifo.frame_num--;
                    vbv->valid_size -= stream->length;
                }
            }
            pthread_mutex_unlock(&vbv->mutex);
        }
    }
}

static u8* vbv_get_base_addr(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    
    if (vbv) {
        return vbv->vbv_buf;
    }

    return NULL;
}
    

static u32 vbv_get_buffer_size(Handle h)
{    
    vbv_t* vbv = (vbv_t*)h;
    
    if(vbv) {
        return vbv->max_size;
    }
    
    return 0;
}

IVBV_t IVBV = 
{
    vbv_request_stream_frame,
    vbv_return_stream_frame,
    vbv_flush_stream_frame,
    vbv_get_base_addr,
    vbv_get_buffer_size
};

static void fbm_enqueue_head(display_queue_t* q, display_frame_t* frame)
{
    if (q && frame) {
        frame->next = q->head;
        q->head = frame;
        q->frame_num++;
    }
}

static void fbm_enqueue_tail(display_queue_t* q, display_frame_t* frame)
{
    display_frame_t* node;
    
    if (q && frame) {
        node = q->head;
        if(node) {
            while(node->next != NULL)
                node = node->next;
            
            node->next = frame;
        } else {
            q->head = frame;
        }
        frame->next = NULL;
        q->frame_num++;
    }
}

static display_frame_t* fbm_dequeue(display_queue_t* q)
{
    display_frame_t* node = NULL;
    
    if (q) {
        node = q->head;
        if(node) {
            q->head = q->head->next;
            node->next = NULL;
            q->frame_num--;
        }
    }
    return node;
}

static display_frame_t* fbm_pick(display_queue_t* q, Handle sys)
{
    display_frame_t *node, *frame;
    
    if (q && sys) {
        node = q->head;
        if(node) {
            if (node->picture.sys == sys) {
                q->head = node->next;
                node->next = NULL;
                q->frame_num--;
                return node;
            } else {
                while(node->next != NULL) {
                    frame = node->next;
                    if (frame->picture.sys == sys) {
                        node->next = frame->next;
                        frame->next = NULL;
                        q->frame_num--;
                        return frame;    
                    }
                    node = frame;
                }
            }
        }
    }
    return NULL;
}

static u8 fbm_pointer_to_index(vpicture_t* frame, Handle h)
{
    u8 i;
    fbm_t* fbm = (fbm_t*)h;
    
    if(fbm) {
      for(i = 0; i < FBM_MAX_FRAME_NUM; i++) {
          if(&fbm->frames[i].vpicture == frame)
              return i;
      }
    }

    return 0xff;
}

static void fbm_free_frame_buffer(cedarx_picture_t* picture)
{
    if(picture->y[0] && picture->size_y[0]) {
        mem_pfree(picture->y[0]);
        picture->y[0] = NULL;
        picture->size_y[0] = 0;
    }
    
    if(picture->u[0] && picture->size_u[0]) {
        mem_pfree(picture->u[0]);
        picture->u[0] = NULL;
        picture->size_u[0] = 0;
    }
    
    if(picture->v[0] && picture->size_v[0]) {
        mem_pfree(picture->v[0]);
        picture->v[0] = NULL;
        picture->size_v[0] = 0;
    }
    
    if(picture->alpha[0] && picture->size_alpha[0]) {
        mem_pfree(picture->alpha[0]);
        picture->alpha[0] = NULL;
        picture->size_alpha[0] = 0;
    }
    
    if(picture->y[1] && picture->size_y[1]) {
        mem_pfree(picture->y[1]);
        picture->y[1] = NULL;
        picture->size_y[1] = 0;
    }
    
    if(picture->u[1] && picture->size_u[1]) {
        mem_pfree(picture->u[1]);
        picture->u[1] = NULL;
        picture->size_u[1] = 0;
    }
    
    if(picture->v[1] && picture->size_v[1]) {
        mem_pfree(picture->v[1]);
        picture->v[1] = NULL;
        picture->size_v[1] = 0;
    }
    
    if(picture->alpha[1] && picture->size_alpha[1]) {
        mem_pfree(picture->alpha[1]);
        picture->alpha[1] = NULL;
        picture->size_alpha[1] = 0;
    }
}

static s32 fbm_alloc_frame_buffer(cedarx_picture_t* picture)
{
    cedarx_decoder_t* decoder = cedarx_decoder;

    if (decoder && decoder->fbm) {
        fbm_t* fbm = decoder->fbm;
        if (picture->size_y[0] < fbm->size_y[0])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_y[0], picture->size_y[0], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_u[0] < fbm->size_u[0])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_u[0], picture->size_u[0], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_v[0] < fbm->size_v[0])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_v[0], picture->size_v[0], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_alpha[0] < fbm->size_alpha[0])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_alpha[0], picture->size_alpha[0], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_y[1] < fbm->size_y[1])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_y[1], picture->size_y[1], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_u[1] < fbm->size_u[1])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_u[1], picture->size_u[1], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_v[1] < fbm->size_v[1])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_v[1], picture->size_v[1], __FILE__, __FUNCTION__, __LINE__);

        if (picture->size_alpha[1] < fbm->size_alpha[1])
            printf("Warning: need 0x%x bytes but just request 0x%x bytes at %s:%s:%d!!!!\n", \
                fbm->size_alpha[1], picture->size_alpha[1], __FILE__, __FUNCTION__, __LINE__);
    }

    if(picture->size_y[0]) {
        picture->y[0] = (u8*)mem_palloc(picture->size_y[0], 0);
        if(!picture->y[0]) 
          goto err;
    }
      
    if(picture->size_u[0]) {
        picture->u[0] = (u8*)mem_palloc(picture->size_u[0], 0);
        if(!picture->u[0])
          goto err;
    }
      
    if(picture->size_v[0]) {
        picture->v[0] = (u8*)mem_palloc(picture->size_v[0], 0);
        if(!picture->v[0])
          goto err;
    }
      
    if(picture->size_alpha[0]) {
        picture->alpha[0] = (u8*)mem_palloc(picture->size_alpha[0], 0);
        if(!picture->alpha[0])
          goto err;
    }
    
    if(picture->size_y[1]) {
        picture->y[1] = (u8*)mem_palloc(picture->size_y[1], 0);
        if(!picture->y[1]) 
          goto err;
    }
      
    if(picture->size_u[1]) {
        picture->u[1] = (u8*)mem_palloc(picture->size_u[1], 0);
        if(!picture->u[1])
          goto err;
    }
      
    if(picture->size_v[1]) {
        picture->v[1] = (u8*)mem_palloc(picture->size_v[1], 0);
        if(!picture->v[1])
          goto err;
    }
      
    if(picture->size_alpha[1]) {
        picture->alpha[1] = (u8*)mem_palloc(picture->size_alpha[1], 0);
        if(!picture->alpha[1])
          goto err;
    }
    return CEDARX_RESULT_OK;
err:
    fbm_free_frame_buffer(picture);
    return CEDARX_RESULT_NO_ENOUGH_MEMORY;
}

static Handle fbm_init(u32 max_frame_num,
            u32 min_frame_num, u32 size_y[2],
            u32 size_u[2], u32 size_v[2],
            u32 size_alpha[2], _3d_mode_e out_3d_mode,
            pixel_format_e format, void* parent)
{
    s32 i;
    fbm_t*  fbm;
    cedarx_decoder_t* decoder = cedarx_decoder;
    
//    libve_io_ctrl(LIBVE_COMMAND_GET_PARENT, (u32)&decoder, parent);
//    if (!decoder)
//        return NULL;

    fbm = (fbm_t*) mem_alloc(sizeof(fbm_t));
    if(!fbm)
        return NULL;
    
    mem_set(fbm, 0, sizeof(fbm_t));
    fbm->size_y[0] = size_y[0];
    fbm->size_u[0] = size_u[0];
    fbm->mode = out_3d_mode;
    if (fbm->mode == _3D_MODE_DOUBLE_STREAM) {
        fbm->size_y[1] = size_y[1];
        fbm->size_u[1] = size_u[1];
    }
    
    for(i = 0; i < FBM_MAX_FRAME_NUM; i++) {
        fbm->frames[i].status                   = FS_EMPTY;
    	fbm->frames[i].vpicture.id              = i;
        fbm->frames[i].vpicture._3d_mode        = fbm->mode;
        fbm->frames[i].vpicture.size_y          = fbm->size_y[0];
        fbm->frames[i].vpicture.size_u          = fbm->size_u[0];
        fbm->frames[i].vpicture.size_v          = fbm->size_v[0];
        fbm->frames[i].vpicture.size_alpha      = fbm->size_alpha[0];
        fbm->frames[i].vpicture.size_y2         = fbm->size_y[1];
        fbm->frames[i].vpicture.size_u2         = fbm->size_u[1];
        fbm->frames[i].vpicture.size_v2         = fbm->size_v[1];
        fbm->frames[i].vpicture.size_alpha2     = fbm->size_alpha[1];
        if(i < max_frame_num) {
            fbm_enqueue_tail(&fbm->empty_queue, &fbm->frames[i]);
            fbm->init_frame_num ++;
        }
    }
    
    fbm->decoder = decoder;
    decoder->fbm = fbm;
    pthread_mutex_init(&fbm->mutex, NULL);
    
    return (Handle)fbm;
}

static void fbm_release(Handle h, void* parent)
{
    fbm_t* fbm = (fbm_t*)h;
    cedarx_decoder_t* decoder;
  
    if(fbm) {
        pthread_mutex_destroy(&fbm->mutex);
        decoder = fbm->decoder;
        decoder->fbm = NULL;
        mem_free(fbm);
    }
}

static vpicture_t* fbm_request_decoder_frame(Handle h)
{
    fbm_t* fbm;
    display_frame_t* frame_info;
    vpicture_t* vpicture = NULL;
    cedarx_picture_t *picture;
    
    fbm = (fbm_t*)h;
    if(fbm && fbm->decoder && fbm->decoder->request_buffer) {
        pthread_mutex_lock(&fbm->mutex);
        cedarx_picture_t pic;
        
        memset(&pic, 0, sizeof(cedarx_picture_t));
        if (fbm->decoder->greedy && !fbm->exhausted) {
            frame_info = fbm_dequeue(&fbm->empty_queue);
            if(frame_info) {
                fbm->decoder->request_buffer(&pic, fbm->decoder->sys);
                if (pic.sys) {
                    picture = &frame_info->picture;
                    vpicture = &frame_info->vpicture;
                    vpicture->y             = pic.y[0];
                    vpicture->u             = pic.u[0];
                    vpicture->v             = pic.v[0];
                    vpicture->alpha         = pic.alpha[0];
                    vpicture->y2            = pic.y[1];
                    vpicture->u2            = pic.u[1];
                    vpicture->v2            = pic.v[1];
                    vpicture->alpha2        = pic.alpha[1];
                    picture->sys            = pic.sys;
                    frame_info->status = FS_DECODER_USING;
                } else {
                    fbm_enqueue_head(&fbm->empty_queue, frame_info);
                }
            } else {
                fbm->exhausted = 1;  
            }
        } else {
            fbm->decoder->request_buffer(&pic, fbm->decoder->sys);
            if (pic.sys) {
                frame_info = fbm_pick(&fbm->bind_queue, pic.sys);
                if(frame_info) {
                    vpicture = &frame_info->vpicture;
                    frame_info->status = FS_DECODER_USING;
                } else {
                    frame_info = fbm_dequeue(&fbm->empty_queue);
                    if(!frame_info) {
                        if (fbm->init_frame_num < FBM_MAX_FRAME_NUM) {
                            frame_info = &fbm->frames[fbm->init_frame_num];
                            fbm->init_frame_num ++;
                        }
                    }
                    
                    if(frame_info) {
                        picture = &frame_info->picture;
                        vpicture = &frame_info->vpicture;
                        vpicture->y             = pic.y[0];
                        vpicture->u             = pic.u[0];
                        vpicture->v             = pic.v[0];
                        vpicture->alpha         = pic.alpha[0];
                        vpicture->y2            = pic.y[1];
                        vpicture->u2            = pic.u[1];
                        vpicture->v2            = pic.v[1];
                        vpicture->alpha2        = pic.alpha[1];
                        picture->sys            = pic.sys;
                        frame_info->status = FS_DECODER_USING;
                    } else {
                        if (fbm->decoder->release_buffer)
                            fbm->decoder->release_buffer(&pic, fbm->decoder->sys);
                    }
                }
            }
        }
        pthread_mutex_unlock(&fbm->mutex);
    }
    
    return vpicture;
}

static void fbm_return_decoder_frame(vpicture_t* frame, u8 valid, Handle h)
{
    u8 idx;
    fbm_t* fbm = (fbm_t*)h;
    display_frame_t* frame_info = NULL;
    cedarx_picture_t *picture;
    
    if(fbm && frame && fbm->decoder) {
        pthread_mutex_lock(&fbm->mutex);
        idx = fbm_pointer_to_index(frame, h);
        if(idx < FBM_MAX_FRAME_NUM) {
            frame_info = &fbm->frames[idx];
            switch (frame_info->status) {
                case FS_DECODER_USING:
                    picture = &frame_info->picture;
                    if(valid) {
                        frame_info->status = FS_DECODER_DISCARD;
                        fbm_enqueue_tail(&fbm->decoded_queue, frame_info);
                    } else {
                        if (fbm->decoder->release_buffer)
                            fbm->decoder->release_buffer(picture, fbm->decoder->sys);
                        frame_info->status = FS_BINDING;
                        fbm_enqueue_tail(&fbm->bind_queue, frame_info);
                    }
                    break;
                case FS_DECODER_SHARED:
                    frame_info->status = FS_DECODER_DISCARD;
                    break;
                case FS_DECODER_SHARE_DISCARD:
                    picture = &frame_info->picture;
                    if (fbm->decoder->unlock_buffer)
                        fbm->decoder->unlock_buffer(picture, fbm->decoder->sys);
                    frame_info->status = FS_BINDING;
                    fbm_enqueue_tail(&fbm->bind_queue, frame_info);
                    break;
                default:
                    printf("Fatal Error: invalid frame(%d) at %s:%s:%d!!!!\n", \
                        frame_info->status, __FILE__, __FUNCTION__, __LINE__);
                case FS_BINDING:
                    break;
            }
        }
        pthread_mutex_unlock(&fbm->mutex);
    }
}

static void fbm_share_decoder_frame(vpicture_t* frame, Handle h)
{
    u8 idx;
    fbm_t* fbm = (fbm_t*)h;
    display_frame_t* frame_info = NULL;
    
    if(fbm && frame) {
        pthread_mutex_lock(&fbm->mutex);
        idx = fbm_pointer_to_index(frame, h);
        if(idx < FBM_MAX_FRAME_NUM) {
            frame_info = &fbm->frames[idx];
            if (frame_info->status == FS_DECODER_USING) {
                frame_info->status = FS_DECODER_SHARED;
                fbm_enqueue_tail(&fbm->decoded_queue, frame_info);
            } else {
                printf("Fatal Error: invalid frame(%d) at %s:%s:%d!!!!\n", \
                    frame_info->status, __FILE__, __FUNCTION__, __LINE__);
            }
        }
        pthread_mutex_unlock(&fbm->mutex);
    }
}

static Handle fbm_request_display_frame(Handle h)
{
    Handle hdl = NULL;
    fbm_t* fbm = (fbm_t*)h;
    display_frame_t* frame_info;
    vpicture_t* vpicture;
    cedarx_picture_t *picture;
    
    if(fbm && fbm->decoder) {
        pthread_mutex_lock(&fbm->mutex);
        frame_info = fbm_dequeue(&fbm->decoded_queue);
        if(frame_info) {
            picture = &frame_info->picture;
            vpicture = &frame_info->vpicture;
            picture->width = vpicture->width;					
            picture->height = vpicture->height;					
            picture->top_offset = vpicture->top_offset;				
            picture->left_offset = vpicture->left_offset;			
            picture->display_width = vpicture->display_width;			
            picture->display_height = vpicture->display_height;
            picture->is_progressive	= vpicture->is_progressive;		
            picture->top_field_first = vpicture->top_field_first;		
            picture->repeat_top_field = vpicture->repeat_top_field;		
            picture->repeat_bottom_field = vpicture->repeat_bottom_field;		
            picture->frame_rate	= vpicture->frame_rate;		
            picture->pts = vpicture->pts;

            if (fbm->decoder->update_buffer)
                fbm->decoder->update_buffer(picture, fbm->decoder->sys);
            hdl = picture->sys;
            switch (frame_info->status) {
                case FS_DECODER_DISCARD:
                    frame_info->status = FS_BINDING;
                    fbm_enqueue_tail(&fbm->bind_queue, frame_info);
                    break;
                case FS_DECODER_SHARED:
                    if (fbm->decoder->lock_buffer)
                        fbm->decoder->lock_buffer(picture, fbm->decoder->sys);
                    frame_info->status = FS_DECODER_SHARE_DISCARD;
                    break;
                default:
                    printf("Fatal Error: invalid frame(%d) at %s:%s:%d!!!!\n", \
                        frame_info->status, __FILE__, __FUNCTION__, __LINE__);
                    break;
            }
        }
        pthread_mutex_unlock(&fbm->mutex);
    }
    return hdl;
}

IFBM_t IFBM = 
{
    fbm_release,
    fbm_request_decoder_frame,
    fbm_return_decoder_frame,
    fbm_share_decoder_frame,
    fbm_init
};

cedarx_result_e libcedarx_decoder_open(cedarx_info_t* info)
{
  cedarx_result_e result;
  vconfig_t config;
  vstream_info_t stream_info;
  cedarx_decoder_t* decoder;
  
  if (!info) 
    return CEDARX_RESULT_INVALID_ARGS;

  if ((info->width > MAX_SUPPORTED_VIDEO_WIDTH) || \
        (info->height > MAX_SUPPORTED_VIDEO_HEIGHT))
    return CEDARX_RESULT_INVALID_ARGS;
    
  decoder = (cedarx_decoder_t*)mem_alloc(sizeof(cedarx_decoder_t));
  if (!decoder)
    return CEDARX_RESULT_NO_ENOUGH_MEMORY;

  mem_set(decoder, 0, sizeof(cedarx_decoder_t));
  cedarx_decoder = decoder;
  
  result = ve_init();
  if (CEDARX_RESULT_OK != result) 
    goto failed3;
          
  mem_set(&config, 0, sizeof(vconfig_t));
  mem_set(&stream_info, 0, sizeof(vstream_info_t));
  
  config.max_video_width  = MAX_SUPPORTED_VIDEO_WIDTH;
  config.max_video_height = MAX_SUPPORTED_VIDEO_HEIGHT;
  config.max_output_width = MAX_SUPPORTED_OUTPUT_WIDTH; 
  config.max_output_height = MAX_SUPPORTED_OUTPUT_HEIGHT;

  switch (info->stream)
  {
    case CEDARX_STREAM_FORMAT_MPEG1:
      stream_info.format = STREAM_FORMAT_MPEG2;
      stream_info.sub_format = MPEG2_SUB_FORMAT_MPEG1;
      break;    
    case CEDARX_STREAM_FORMAT_MPEG2:
      stream_info.format = STREAM_FORMAT_MPEG2;
      stream_info.sub_format = MPEG2_SUB_FORMAT_MPEG2;
      break;    
    case CEDARX_STREAM_FORMAT_XVID:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_XVID;
      break;    
    case CEDARX_STREAM_FORMAT_DIVX1:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_DIVX1;
      break;    
    case CEDARX_STREAM_FORMAT_DIVX2:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_DIVX2;
      break;    
    case CEDARX_STREAM_FORMAT_DIVX3:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_DIVX3;
      break;    
    case CEDARX_STREAM_FORMAT_DIVX4:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_DIVX4;
      break;    
    case CEDARX_STREAM_FORMAT_DIVX5:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_DIVX5;
      break;    
    case CEDARX_STREAM_FORMAT_H263:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_H263;
      break;    
    case CEDARX_STREAM_FORMAT_SORENSSON_H263:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_SORENSSON_H263;
      break;    
    case CEDARX_STREAM_FORMAT_WMV1:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_WMV1;
      break;    
    case CEDARX_STREAM_FORMAT_WMV2:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_WMV2;
      break;    
    case CEDARX_STREAM_FORMAT_VP6:
      stream_info.format = STREAM_FORMAT_MPEG4;
      stream_info.sub_format = MPEG4_SUB_FORMAT_VP6;
      break;    
    case CEDARX_STREAM_FORMAT_REALVIDEO:
      stream_info.format = STREAM_FORMAT_REALVIDEO;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;    
    case CEDARX_STREAM_FORMAT_H264:
      stream_info.format = STREAM_FORMAT_H264;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      stream_info.is_pts_correct = 1;
      decoder->greedy = 1;
      break;    
    case CEDARX_STREAM_FORMAT_VC1:
      stream_info.format = STREAM_FORMAT_VC1;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;    
    case CEDARX_STREAM_FORMAT_AVS:
      stream_info.format = STREAM_FORMAT_AVS;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;    
    case CEDARX_STREAM_FORMAT_MJPEG:
      stream_info.format = STREAM_FORMAT_MJPEG;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;    
    case CEDARX_STREAM_FORMAT_VP8:
      stream_info.format = STREAM_FORMAT_VP8;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;
    default:
      result = CEDARX_RESULT_INVALID_ARGS;
      goto failed2;
  } 

  switch (info->container)
  {
    case CEDARX_CONTAINER_FORMAT_AVI:
      stream_info.container_format = CONTAINER_FORMAT_AVI;
      break;
    case CEDARX_CONTAINER_FORMAT_ASF:
      stream_info.container_format = CONTAINER_FORMAT_ASF;
      break;
    case CEDARX_CONTAINER_FORMAT_DAT:
      stream_info.container_format = CONTAINER_FORMAT_DAT;
      break;
    case CEDARX_CONTAINER_FORMAT_FLV:
      stream_info.container_format = CONTAINER_FORMAT_FLV;
      break;
    case CEDARX_CONTAINER_FORMAT_MKV:
      stream_info.container_format = CONTAINER_FORMAT_MKV;
      break;
    case CEDARX_CONTAINER_FORMAT_MOV:
      stream_info.container_format = CONTAINER_FORMAT_MOV;
      break;
    case CEDARX_CONTAINER_FORMAT_MPG:
      stream_info.container_format = CONTAINER_FORMAT_MPG;
      break;
    case CEDARX_CONTAINER_FORMAT_PMP:
      stream_info.container_format = CONTAINER_FORMAT_PMP;
      break;
    case CEDARX_CONTAINER_FORMAT_RM:
      stream_info.container_format = CONTAINER_FORMAT_RM;
      break;
    case CEDARX_CONTAINER_FORMAT_TS:
      stream_info.container_format = CONTAINER_FORMAT_TS;
      stream_info.is_pts_correct = 1;
      break;
    case CEDARX_CONTAINER_FORMAT_VOB:
      stream_info.container_format = CONTAINER_FORMAT_VOB;
      break;
    case CEDARX_CONTAINER_FORMAT_WEBM:
      stream_info.container_format = CONTAINER_FORMAT_WEBM;
      break;
    case CEDARX_CONTAINER_FORMAT_OGM:
      stream_info.container_format = CONTAINER_FORMAT_OGM;
      break;
    default:
      stream_info.container_format = CONTAINER_FORMAT_UNKNOW;
      break;
  }

  stream_info.video_width = info->width;
  stream_info.video_height = info->height;
  stream_info.frame_rate = info->frame_rate;
  stream_info.frame_duration = info->frame_duration;
  stream_info.aspec_ratio = 1000;

  if (info->data && (info->data_size > 0)) {
    decoder->init_data = mem_alloc(info->data_size);
    if (!decoder->init_data) {
        result = CEDARX_RESULT_NO_ENOUGH_MEMORY;
        goto failed2;
    }
    mem_cpy(decoder->init_data, info->data, info->data_size);
    stream_info.init_data = decoder->init_data;
    stream_info.init_data_len = info->data_size;
  } else {
    stream_info.init_data_len = 0;
    stream_info.init_data = NULL;
  }
  stream_info._3d_mode = _3D_MODE_NONE;

  mem_init(decoder->fd);
  ve_request();

  if (info->request_buffer)
    decoder->request_buffer = info->request_buffer;
  if (info->update_buffer)
    decoder->update_buffer = info->update_buffer;
  if (info->release_buffer)
    decoder->release_buffer = info->release_buffer;
  if (info->lock_buffer)
    decoder->lock_buffer = info->lock_buffer;
  if (info->unlock_buffer)
    decoder->unlock_buffer = info->unlock_buffer;
 
  decoder->sys = info->sys;
  decoder->ve = libve_open(&config, &stream_info, decoder);
  if (!decoder->ve) {
    result = CEDARX_RESULT_VE_FAILED;
    goto failed1;
  }
  
  libve_reset(0, decoder->ve);  
  decoder->vbv = vbv_init();
  if (!decoder->vbv) {
    result = CEDARX_RESULT_NO_ENOUGH_MEMORY;
    goto failed0;
  }
    
  libve_set_vbv(decoder->vbv, decoder->ve);
  return CEDARX_RESULT_OK; 

failed0:
  libve_close(0, decoder->ve);  
failed1:
  ve_release();
  mem_exit();
  if (decoder->init_data)
    mem_free(decoder->init_data);
failed2:
  ve_exit();
failed3:
  mem_free(decoder); 
  cedarx_decoder = NULL;
  return result;    
}

void libcedarx_decoder_close(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if (decoder) {
        if (decoder->ve) {
            libve_flush(0, decoder->ve);  
            libve_close(0, decoder->ve);  
        }
        
        if (decoder->init_data)
            mem_free(decoder->init_data);
        
        if (decoder->vbv) {
            vbv_exit(decoder->vbv);
        }  
        ve_release();
        if (!cedarx_display)
            mem_exit();
        
        ve_exit();
        mem_free(decoder); 
        cedarx_decoder = NULL;
    }
}

cedarx_result_e libcedarx_decoder_add_stream(u8* buf, u32 size, u64 pts, u64 pcr)
{
    vstream_data_t stream_data;
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if ((!decoder) || (!decoder->vbv)) 
      return CEDARX_RESULT_NO_INIT;
    
    memset(&stream_data, 0, sizeof(vstream_data_t));
    stream_data.data = buf;
    stream_data.length = size;
    stream_data.pts = pts;
    stream_data.pcr = pcr;
    stream_data.valid = 1;
    
    return vbv_add_stream_frame(&stream_data, decoder->vbv);
}

cedarx_result_e libcedarx_decoder_decode_stream(int force)
{
    vresult_e res;
    cedarx_result_e result;
    cedarx_decoder_t* decoder = cedarx_decoder;
    
    if ((!decoder) || (!decoder->ve) || (!decoder->vbv)) 
      return CEDARX_RESULT_NO_INIT;
    
    if (!force) {
        if (vbv_check_stream_frame(decoder->vbv) < 0)
            return CEDARX_RESULT_VBV_BUFFER_EMPTY;
    }
    
    res = libve_decode(0, 0, 0, decoder->ve);
    switch (res) {
      case VRESULT_OK:  
      case VRESULT_FRAME_DECODED:  
      case VRESULT_KEYFRAME_DECODED:  
          result = CEDARX_RESULT_OK;
          break;
      case VRESULT_NO_FRAME_BUFFER:  
          printf("Warning: There are not more framebuffer!\n");
          result = CEDARX_RESULT_NO_FRAME_BUFFER;
          break;
      case VRESULT_NO_BITSTREAM:  
          printf("Warning: There are not more bitstream!\n");
          result = CEDARX_RESULT_VBV_BUFFER_EMPTY;
          break;
      default:
          printf("Error: Decode failed(%d), try to reset decoder!\n", res);
          libve_reset(0, decoder->ve);
          return CEDARX_RESULT_VE_FAILED;     
    }
    
    return result;
}

Handle libcedarx_decoder_request_frame(void)
{
    cedarx_decoder_t* decoder = cedarx_decoder;

    if ((!decoder) || (!decoder->fbm)) 
        return NULL;

    return fbm_request_display_frame(decoder->fbm);
}

cedarx_result_e libcedarx_display_open(void)
{
    cedarx_display_t* display;
    
    if (!cedarx_decoder)
        return CEDARX_RESULT_NO_INIT;

    display = (cedarx_display_t*)mem_alloc(sizeof(cedarx_display_t));
    if (!display)
      return CEDARX_RESULT_NO_ENOUGH_MEMORY;
    
    mem_set(display, 0, sizeof(cedarx_display_t));
    cedarx_display = display;

    display->fd = open("/dev/disp", O_RDWR);
    if (display->fd == -1) {
        free(display);
        cedarx_display = NULL;
        return CEDARX_RESULT_DE_FAILED; 
    }

    display->info.layer = -1;
    pthread_mutex_init(&display->mutex, NULL);
   
    return CEDARX_RESULT_OK; 
}

void libcedarx_display_close(void)
{
    int time = 20;
    unsigned long args[4];
    cedarx_display_t* display = cedarx_display;

    if (display) {
        pthread_mutex_lock(&display->mutex);
        if (display->info.enable) {
            args[0] = 0;
            args[1] = display->info.layer;
            args[2] = 0;
            args[3] = 0;
            if (display->info.init) {
                if (display->info.start) {
                    while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->info.last_id) {
                        if (--time < 0)
                            break;    
                        usleep(1000);
                    } 
                }
        
                ioctl(display->fd, DISP_CMD_LAYER_CLOSE, args );
                usleep(20000); 
                ioctl(display->fd, DISP_CMD_VIDEO_STOP, args);
            }
            ioctl(display->fd, DISP_CMD_LAYER_RELEASE, args);
        }

        pthread_mutex_unlock(&display->mutex);
        pthread_mutex_destroy(&display->mutex);
        close(display->fd);
        free(display);    
        if (!cedarx_decoder)
            mem_exit();
        cedarx_display = NULL;
    }
}

cedarx_result_e libcedarx_display_alloc_frame(cedarx_picture_t *picture)
{
    cedarx_display_t* display = cedarx_display;

    if (!picture || !display) 
        return CEDARX_RESULT_NO_INIT;

    if (fbm_alloc_frame_buffer(picture) < 0)
        return CEDARX_RESULT_NO_ENOUGH_MEMORY;
    
     return CEDARX_RESULT_OK;
}

void libcedarx_display_free_frame(cedarx_picture_t *picture)
{
    cedarx_display_t* display = cedarx_display;

    if (display && picture)
        fbm_free_frame_buffer(picture);
}

cedarx_result_e libcedarx_display_request_layer(int top)
{
    int layer;
    cedarx_result_e ret = CEDARX_RESULT_NO_RESOURCE;
    cedarx_display_t* display = cedarx_display;
    unsigned long args[4];

    if (!display) 
        return CEDARX_RESULT_NO_INIT;
        
    pthread_mutex_lock(&display->mutex);
    args[0] = 0;
    args[1] = DISP_LAYER_WORK_MODE_SCALER;
    args[2] = 0;
    args[3] = 0;
    if (!display->info.enable) {
        layer = ioctl(display->fd, DISP_CMD_LAYER_REQUEST, args);
        if (layer > 0) {
            display->info.layer = layer;
            display->info.enable = 1;
            display->info.top = top;
            ret = CEDARX_RESULT_OK;
        }
    }
    pthread_mutex_unlock(&display->mutex);
    
    return ret; 
}

void libcedarx_display_release_layer(void)
{
    int time = 20;
    unsigned long args[4];
    cedarx_display_t* display = cedarx_display;

    if (display) {
        pthread_mutex_lock(&display->mutex);
        if (display->info.enable) {
            display->info.enable = 0;
            args[0] = 0;
            args[1] = display->info.layer;
            args[2] = 0;
            args[3] = 0;
            if (display->info.init) {
                if (display->info.start) {
                    while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->info.last_id) {
                        if (--time < 0)
                            break;    
                        usleep(1000);    
                    } 
                    display->info.start = 0;
                }
            
                ioctl(display->fd, DISP_CMD_LAYER_CLOSE, args );
                usleep(20000); 
                ioctl(display->fd, DISP_CMD_VIDEO_STOP, args);
                display->info.init = 0;
            }
            
            ioctl(display->fd, DISP_CMD_LAYER_RELEASE, args);
            display->info.layer = -1;
        }            
        pthread_mutex_unlock(&display->mutex);
    }
}

cedarx_result_e libcedarx_display_video_frame(cedarx_picture_t *picture)
{
    int time = 20;
    unsigned long args[4];
    cedarx_display_t* display = cedarx_display;

    if (!display) 
        return CEDARX_RESULT_NO_INIT;

    if (!picture)
        return CEDARX_RESULT_INVALID_ARGS;

    pthread_mutex_lock(&display->mutex);
    if (display->info.enable) {
        if (!display->info.init) {
            __disp_layer_info_t layer_info;
            u32 screen_width, screen_height;
            
            args[0] = 0;
            args[1] = 0;
            args[2] = 0;
            args[3] = 0;
            screen_width = ioctl(display->fd, DISP_CMD_SCN_GET_WIDTH, args);
            screen_height = ioctl(display->fd, DISP_CMD_SCN_GET_HEIGHT, args);
            
            memset(&layer_info, 0, sizeof(layer_info));
            
            layer_info.pipe = 1;
            layer_info.mode = DISP_LAYER_WORK_MODE_SCALER;
            layer_info.fb.mode = DISP_MOD_MB_UV_COMBINED;
            layer_info.fb.format = DISP_FORMAT_YUV420;
            layer_info.fb.seq = DISP_SEQ_UVUV;
            layer_info.fb.br_swap = 0;
            layer_info.fb.addr[0] = mem_phy_addr((u32)picture->y[0]);
            layer_info.fb.addr[1] = mem_phy_addr((u32)picture->u[0]);
            
            if (picture->height < 720) {
              layer_info.fb.cs_mode = DISP_BT601;
            } else {
              layer_info.fb.cs_mode = DISP_BT709;
            }
            
            layer_info.fb.size.width = picture->width;
            layer_info.fb.size.height = picture->height; 
            
            layer_info.src_win.x = picture->top_offset;
            layer_info.src_win.y = picture->left_offset;
            layer_info.src_win.width = picture->display_width;
            layer_info.src_win.height = picture->display_height;
            layer_info.scn_win.x = 0;
            layer_info.scn_win.y = 0;
            layer_info.scn_win.width = screen_width;
            layer_info.scn_win.height = screen_height;
            args[0] = 0;
            args[1] = display->info.layer;
            args[2] = (unsigned long)(&layer_info);
            args[3] = 0;
            ioctl(display->fd, DISP_CMD_LAYER_SET_PARA, args);
            if (display->info.top)
                ioctl(display->fd, DISP_CMD_LAYER_TOP, args );
            else
                ioctl(display->fd, DISP_CMD_LAYER_BOTTOM, args );
            ioctl(display->fd, DISP_CMD_VIDEO_START, args);
            ioctl(display->fd, DISP_CMD_LAYER_OPEN, args );
            display->info.last_id = 0;
            display->info.init = 1;
        } else {
            __disp_video_fb_t video;
            args[0] = 0;
            args[1] = display->info.layer;
            args[2] = 0;
            args[3] = 0;
            if (display->info.start) {
                while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->info.last_id) {
                    if (--time < 0) {
                        break; 
                    }   
                    usleep(1000);    
                } 
            } else {
                display->info.start = 1;
            }

            memset(&video, 0, sizeof(__disp_video_fb_t));
            video.id = display->idx;
            video.frame_rate = picture->frame_rate;
            video.addr[0] = mem_phy_addr((u32)picture->y[0]);
            video.addr[1] = mem_phy_addr((u32)picture->u[0]);
            
            args[0] = 0;
            args[1] = display->info.layer;
            args[2] = (unsigned long)(&video);
            args[3] = 0;
            ioctl(display->fd, DISP_CMD_VIDEO_SET_FB, args);
            display->info.last_id = display->idx;
        }
    }    
    display->idx ++;
    pthread_mutex_unlock(&display->mutex);
    return CEDARX_RESULT_OK; 
}

cedarx_result_e libcedarx_get_stream_info(cedarx_info_t *info)
{
	vstream_info_t vstream_info;
	cedarx_decoder_t* decoder = cedarx_decoder;
	vresult_e e;
	if (decoder && decoder->ve) {
			
		if ((e = libve_get_stream_info(&vstream_info, decoder->ve))) {
		      return CEDARX_RESULT_NO_RESOURCE;
		}
		info->width = vstream_info.video_width;
		info->height = vstream_info.video_height;
//		info->frame_rate = vstream_info.frame_rate;
//		info->frame_duration = vstream_info.frame_duration;
		return CEDARX_RESULT_OK;

	}

	return CEDARX_RESULT_NO_INIT;

}

char* libcedarx_version(void)
{
    return LIBCEDARX_VERSION;
}

int cedarv_f23_ic_version(void)
{
	return 1;
}
