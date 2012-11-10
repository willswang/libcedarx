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
#define VBV_MAX_FRAME_NUM                   2048
#define FBM_MAX_FRAME_NUM                   64
#define FBM_MIN_FRAME_NUM                   4
#define MAX_SUPPORTED_VIDEO_WIDTH           3840
#define MAX_SUPPORTED_VIDEO_HEIGHT          2160
#define MAX_SUPPORTED_OUTPUT_WIDTH          3840
#define MAX_SUPPORTED_OUTPUT_HEIGHT         2160
#define MAX_SUPPORTED_DISPLAY_NUM           2

#define DECODE_USE_ASYNC_THREAD             1

typedef struct STREAM_FRAME_T stream_frame_t;
struct STREAM_FRAME_T
{
    u32 id;
    vstream_data_t  vstream;
    stream_frame_t* next;
};

typedef struct
{
    stream_frame_t* in_frames;
    u32 read_index;
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
    u8* read_addr;
    u8* write_addr;
    u32 valid_size;
    u32 max_size;
    stream_fifo_t frame_fifo;
    stream_queue_t frame_queue;    
    pthread_mutex_t mutex;
#ifdef DECODE_USE_ASYNC_THREAD
    sem_t sem_put;
    sem_t sem_get;
#endif
}vbv_t;

typedef enum
{
    FS_EMPTY                         = 0,
    FS_DECODER_USING                 = 1,
    FS_PLAYER_USING                  = 2,
    FS_DISPLAY_USING                 = 3,
    FS_DECODER_SHARE_PLAYER_USING    = 4,
    FS_DECODER_SHARE_DISPLAY_USING   = 5,
    FS_DECODER_SHARE_DISCARD         = 6
}frame_status_e;

typedef struct DISPLAY_FRAME_T display_frame_t;
struct DISPLAY_FRAME_T
{
    vpicture_t picture;
    frame_status_e status;
    display_frame_t* next;
};

typedef struct
{
    display_frame_t* head;
    u32 frame_num;
}display_queue_t;

typedef struct
{
    u32 max_frame_num;
    display_queue_t empty_queue;
    display_queue_t decoded_queue;
    display_queue_t display_queue;
    display_frame_t frames[FBM_MAX_FRAME_NUM];
    pthread_mutex_t mutex;
#ifdef DECODE_USE_ASYNC_THREAD
    u32 first;
    sem_t sem;
#endif
}fbm_t;

typedef struct
{
    Handle  ve;
    Handle  fbm;
    Handle  vbv;
    u8* init_data;
    int fd;
    unsigned int address;
    pthread_mutex_t mutex;
}cedarx_decoder_t;

typedef struct
{
    int fd;
    int num;
    int layer;
    int start;
    int init;
    int last_id;
    pthread_mutex_t mutex;
}cedarx_display_t;

static fbm_t *cedarx_fbm = NULL;
static cedarx_decoder_t *cedarx_decoder = NULL;
static cedarx_display_t *cedarx_display = NULL;

static void mem_init(void)
{
  cedarx_decoder_t* decoder = cedarx_decoder;
  if (decoder && decoder->fd != -1)
    av_heap_init(decoder->fd);
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
  return (void*)av_heap_alloc(size); 
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
    return av_heap_physic_addr((void*)virtual_addr);
  
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
  } else {
      printf("Fatal Error: ve initialization incorrect at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
  }
}

void ve_release(void)
{
  cedarx_decoder_t* decoder = cedarx_decoder;
  if (decoder && decoder->fd != -1) {
      ioctl(decoder->fd, IOCTL_ENGINE_REL, 0);
  } else {
      printf("Fatal Error: ve initialization incorrect at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
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
  } else {
      printf("Fatal Error: ve initialization incorrect at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
  }
}

static void ve_enable_clock(u8 enable, u32 speed)
{
  cedarx_decoder_t* decoder = cedarx_decoder;

  if (decoder && decoder->fd != -1) {
    if(enable)
    {
      ioctl(decoder->fd, IOCTL_ENABLE_VE, 0);
      ioctl(decoder->fd, IOCTL_SET_VE_FREQ, speed/1000000);
    }
    else
    {
      ioctl(decoder->fd, IOCTL_DISABLE_VE, 0);
    }
  } else {
      printf("Fatal Error: ve initialization incorrect at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
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
  } else {
      printf("Fatal Error: ve initialization incorrect at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
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

static void vbv_enqueue_head(stream_frame_t* stream, stream_queue_t* q)
{
    if (q && stream) {
      stream->next = q->head;
      q->head = stream;
      q->frame_num++;
    }
}

static void vbv_enqueue_tail(stream_frame_t* stream, stream_queue_t* q)
{
    stream_frame_t* node;
    
    if (q && stream) {
      node = q->head;
      if(node)
      {
          while(node->next != NULL)
              node = node->next;
          
          node->next = stream;
          stream->next = NULL;
      }
      else
      {
          q->head = stream;
          stream->next = NULL;
      }
      q->frame_num++;
    }
}

static stream_frame_t* vbv_dequeue(stream_queue_t* q)
{
    stream_frame_t* node;
    
    if (q) {
      node = q->head;
      if(node) {
        q->head = q->head->next;
        node->next = NULL;
        q->frame_num--;
        return node;
      }
    }

    return NULL;
}

static Handle vbv_init(void)
{
    u8* vbv_buf;
    vbv_t* vbv;

    vbv = (vbv_t*)mem_alloc(sizeof(vbv_t));
    if (vbv)
    {
      mem_set(vbv, 0, sizeof(vbv_t));

      vbv->frame_fifo.in_frames = (stream_frame_t*)mem_alloc(VBV_MAX_FRAME_NUM * sizeof(stream_frame_t));
      if (!vbv->frame_fifo.in_frames)
      {
          mem_free(vbv);
          return NULL;
      }
      
      mem_set(vbv->frame_fifo.in_frames, 0, VBV_MAX_FRAME_NUM * sizeof(stream_frame_t));

      vbv_buf = (u8*)mem_palloc(VBV_FRAME_SIZE, 1024);
      if (!vbv_buf) {
          mem_free(vbv); 
          mem_free(vbv->frame_fifo.in_frames); 
          return NULL;
      }

      vbv->vbv_buf     = vbv_buf;
      vbv->vbv_buf_end = vbv_buf + VBV_FRAME_SIZE;
      vbv->max_size    = VBV_FRAME_SIZE;
      vbv->read_addr   = vbv_buf;
      vbv->write_addr  = vbv_buf;
      vbv->valid_size  = 0;

      vbv->frame_fifo.frame_num     = 0;
      vbv->frame_fifo.read_index    = 0;
      vbv->frame_fifo.write_index   = 0;
      vbv->frame_fifo.max_frame_num = VBV_MAX_FRAME_NUM;
      
      vbv->frame_queue.frame_num = 0;
      vbv->frame_queue.head      = NULL;

      pthread_mutex_init(&vbv->mutex, NULL);
#ifdef DECODE_USE_ASYNC_THREAD
      sem_init(&vbv->sem_get, 0, 0);
      sem_init(&vbv->sem_put, 0, 3);
#endif
      return (Handle)vbv;
    }

    return NULL;
}

static void vbv_exit(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;

    if (vbv)
    {
#ifdef DECODE_USE_ASYNC_THREAD
        sem_destroy(&vbv->sem_put);
        sem_destroy(&vbv->sem_get);
#endif
        pthread_mutex_destroy(&vbv->mutex);

        if (vbv->frame_fifo.in_frames)
        {
            mem_free(vbv->frame_fifo.in_frames);
        }
        
        if (vbv->vbv_buf)
        {
            mem_pfree(vbv->vbv_buf);
        }

        mem_free(vbv);
    }
}
    
static void vbv_reset(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    
    if (vbv)
    {
      pthread_mutex_lock(&vbv->mutex);
      vbv->read_addr = vbv->write_addr = vbv->vbv_buf;
      vbv->valid_size = 0;
      vbv->frame_fifo.frame_num   = 0;
      vbv->frame_fifo.read_index  = 0;
      vbv->frame_fifo.write_index = 0;
      vbv->frame_queue.frame_num  = 0;
      vbv->frame_queue.head       = NULL;
      pthread_mutex_unlock(&vbv->mutex);
    }
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
    
#ifdef DECODE_USE_ASYNC_THREAD
    sem_wait(&vbv->sem_put);        
#endif
    pthread_mutex_lock(&vbv->mutex);

    new_write_addr = vbv->write_addr + stream->length;
    if (new_write_addr >= vbv->vbv_buf_end)
    { 
      u32 size = vbv->vbv_buf_end - vbv->write_addr;
      mem_cpy(vbv->write_addr, stream->data, size);
      mem_cpy(vbv->vbv_buf, stream->data + size, stream->length - size);
      new_write_addr -= vbv->max_size;
    } else {
      mem_cpy(vbv->write_addr, stream->data, stream->length);
    }
          
    write_index = vbv->frame_fifo.write_index;
    frame = &vbv->frame_fifo.in_frames[write_index];
    mem_cpy(&frame->vstream, stream, sizeof(vstream_data_t));
    frame->vstream.data = vbv->write_addr;
    write_index ++;
    if (write_index >= vbv->frame_fifo.max_frame_num)
    {
        write_index = 0;
    }
    vbv->frame_fifo.write_index = write_index;
    vbv->frame_fifo.frame_num++;
    vbv->valid_size += stream->length;
    vbv->write_addr = new_write_addr;
    vbv_enqueue_tail(frame, &vbv->frame_queue);     //* add this frame to the queue tail.
    pthread_mutex_unlock(&vbv->mutex);
#ifdef DECODE_USE_ASYNC_THREAD
    sem_post(&vbv->sem_get);
    pthread_yield(); 
#endif
    return CEDARX_RESULT_OK;
}

static vstream_data_t* vbv_request_stream_frame(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    stream_frame_t* frame;    
    vstream_data_t* stream = NULL;

    if (vbv)
    {
#ifdef DECODE_USE_ASYNC_THREAD
        sem_wait(&vbv->sem_get);        
#endif
        pthread_mutex_lock(&vbv->mutex);
        frame = vbv_dequeue(&vbv->frame_queue);
        if (frame) {
            frame->vstream.id = frame->id;
            stream = &frame->vstream;
        } else {
            printf("Fatal Error: mismatch queue with semapore at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
        }
        
        pthread_mutex_unlock(&vbv->mutex);
#ifdef DECODE_USE_ASYNC_THREAD
        sem_post(&vbv->sem_put);        
#endif
    }
    
    return stream;
}


static void vbv_return_stream_frame(vstream_data_t* stream, Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    u32 delta, id;
    
    if (vbv)
    {
      if (vbv->frame_fifo.frame_num > 0)
      {
        pthread_mutex_lock(&vbv->mutex);
        
        id = vbv->frame_fifo.in_frames[vbv->frame_fifo.read_index].id;
        if (id < stream->id) {
          delta = stream->id - id;
         } else {
          delta = id - stream->id;
        }
        
        id =( vbv->frame_fifo.read_index + delta) % vbv->frame_fifo.max_frame_num;
        vbv_enqueue_head(&vbv->frame_fifo.in_frames[id], &vbv->frame_queue);
        pthread_mutex_unlock(&vbv->mutex);
#ifdef DECODE_USE_ASYNC_THREAD
        sem_post(&vbv->sem_get);        
#endif
      }
    }
}

static void vbv_flush_stream_frame(vstream_data_t* stream, Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    u32 read_index;
    u8* new_write_addr;

    if (vbv)
    {
      if (vbv->frame_fifo.frame_num > 0)
      {
        pthread_mutex_lock(&vbv->mutex);
        read_index = vbv->frame_fifo.read_index;
        if(stream == &vbv->frame_fifo.in_frames[read_index].vstream)
        {
          read_index++;
          if (read_index >= vbv->frame_fifo.max_frame_num)
          {
              read_index = 0;
          }
      
          vbv->frame_fifo.read_index = read_index;
          vbv->frame_fifo.frame_num--;
          vbv->valid_size -= stream->length;
          new_write_addr = vbv->read_addr + stream->length;
          if (new_write_addr >= vbv->vbv_buf_end)
          {
              new_write_addr  -= vbv->max_size;
          }
          vbv->read_addr = new_write_addr;
        }
        
        pthread_mutex_unlock(&vbv->mutex);
      }
    }
}

static u8* vbv_get_base_addr(Handle h)
{
    vbv_t* vbv = (vbv_t*)h;
    
    if (vbv)
    {
        return vbv->vbv_buf;
    }

    return NULL;
}
    

static u32 vbv_get_buffer_size(Handle h)
{    
    vbv_t* vbv = (vbv_t*)h;
    
    if(vbv)
    {
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

static u8 fbm_pointer_to_index(vpicture_t* frame, Handle h)
{
    u8            i;
    fbm_t*        fbm;
    
    fbm = (fbm_t*)h;
    if(fbm) {
      for(i = 0; i < fbm->max_frame_num; i++)
      {
          if(&fbm->frames[i].picture == frame)
              return i;
      }
    }
   
    return 0xff;
}

static display_frame_t* fbm_index_to_pointer(u32 id, fbm_t* fbm)
{
    u8 i;
    display_frame_t* frame;
    
    if(fbm) {
      for(i = 0; i < fbm->max_frame_num; i++)
      {
          frame = &fbm->frames[i];
          if(frame->picture.id == id)
              return frame;
      }
    }
   
    return NULL;
}

static void fbm_enqueue(display_queue_t* q, display_frame_t* frame)
{
    display_frame_t* node;
    
    if (q && frame) {
      node = q->head;
      if(node)
      {
          while(node->next != NULL)
              node = node->next;
          
          node->next = frame;
      }
      else
      {
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

static display_frame_t* fbm_peek(display_queue_t* q)
{
    display_frame_t* node = NULL;
    
    if (q) {
      node = q->head;
    }
    return node;
}

static s32 fbm_free_frame_buffer(vpicture_t* picture)
{
  if(picture->y)
  {
    mem_pfree(picture->y);
    picture->y = NULL;
  }
  
  if(picture->u)
  {
    mem_pfree(picture->u);
    picture->u = NULL;
  }
  
  if(picture->v)
  {
    mem_pfree(picture->v);
    picture->v = NULL;
  }
  
  if(picture->alpha)
  {
    mem_pfree(picture->alpha);
    picture->alpha = NULL;
  }

  if(picture->y2)
  {
    mem_pfree(picture->y2);
    picture->y2 = NULL;
  }
  
  if(picture->u2)
  {
    mem_pfree(picture->u2);
    picture->u2 = NULL;
  }
  
  if(picture->v2)
  {
    mem_pfree(picture->v2);
    picture->v2 = NULL;
  }
  
  if(picture->alpha2)
  {
    mem_pfree(picture->alpha2);
    picture->alpha2 = NULL;
  }
  
  return 0;
}

static s32 fbm_alloc_frame_buffer(vpicture_t* picture)
{
  picture->y      = NULL;
  picture->u      = NULL;
  picture->v      = NULL;
  picture->alpha  = NULL;
  picture->y2     = NULL;
  picture->v2     = NULL;
  picture->u2     = NULL;
  picture->alpha2 = NULL;
    
  if(picture->size_y)
  {
    picture->y = (u8*)mem_palloc(picture->size_y, 0);
    if(!picture->y) 
      goto err;
  }
    
  if(picture->size_u)
  {
    picture->u = (u8*)mem_palloc(picture->size_u, 0);
    if(!picture->u)
      goto err;
  }
    
  if(picture->size_v)
  {
    picture->v = (u8*)mem_palloc(picture->size_v, 0);
    if(!picture->v)
      goto err;
  }
    
  if(picture->size_alpha)
  {
    picture->alpha = (u8*)mem_palloc(picture->size_alpha, 0);
    if(!picture->alpha)
      goto err;
  }
    
  if(picture->size_y2)
  {
    picture->y2 = (u8*)mem_palloc(picture->size_y2, 0);
    if(!picture->y2) 
      goto err;
  }
    
  if(picture->size_u2)
  {
    picture->u2 = (u8*)mem_palloc(picture->size_u2, 0);
    if(!picture->u2)
      goto err;
  }
    
  if(picture->size_v2)
  {
    picture->v2 = (u8*)mem_palloc(picture->size_v2, 0);
    if(!picture->v2)
      goto err;
  }
    
  if(picture->size_alpha2)
  {
    picture->alpha2 = (u8*)mem_palloc(picture->size_alpha2, 0);
    if(!picture->alpha2)
      goto err;
  }

  return 0;
err:
  fbm_free_frame_buffer(picture);
  return -1;
}

static Handle fbm_init(u32 max_frame_num,
                u32 min_frame_num, 
                u32 size_y[2],
                u32 size_u[2],
                u32 size_v[2],
                u32 size_alpha[2],
                _3d_mode_e out_3d_mode,
                pixel_format_e format,
                void* parent)
{
  s32     i, j;
  fbm_t*  fbm;

  if(max_frame_num < min_frame_num)
    max_frame_num = min_frame_num;
  
  if(min_frame_num > FBM_MAX_FRAME_NUM)
    return NULL;
  
  if(max_frame_num < FBM_MIN_FRAME_NUM)
    max_frame_num = FBM_MIN_FRAME_NUM;
  
  if(min_frame_num < FBM_MIN_FRAME_NUM)
    min_frame_num = FBM_MIN_FRAME_NUM;
  
  if(max_frame_num > FBM_MAX_FRAME_NUM)
    max_frame_num = FBM_MAX_FRAME_NUM;

  if (cedarx_fbm) {
      fbm = cedarx_fbm;

      if ((fbm->frames[0].picture.size_y != size_y[0]) || (fbm->frames[0].picture.size_u != size_u[0]) ||
            (fbm->frames[0].picture.size_v != size_v[0]) || (fbm->frames[0].picture.size_alpha != size_alpha[0])) {
        printf("Fatal Error: framebuffer size mismatch at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
        return NULL;
      }

      if (fbm->max_frame_num < max_frame_num) {
          //* alloc memory frame buffer.
          for (i = fbm->max_frame_num; i < (s32)max_frame_num; i++)
          {
            fbm->frames[i].picture.id = i;
            fbm->frames[i].picture.size_y = size_y[0];
            fbm->frames[i].picture.size_u = size_u[0];
            fbm->frames[i].picture.size_v = size_v[0];
            fbm->frames[i].picture.size_alpha = size_alpha[0];
            fbm->frames[i].picture._3d_mode = out_3d_mode;
            
            if(out_3d_mode == _3D_MODE_DOUBLE_STREAM) {
                fbm->frames[i].picture.size_y2 = size_y[1];
                fbm->frames[i].picture.size_u2 = size_u[1];
                fbm->frames[i].picture.size_v2 = size_v[1];
                fbm->frames[i].picture.size_alpha2 = size_alpha[1];
            }
            
            if(fbm_alloc_frame_buffer(&fbm->frames[i].picture) != 0)
              break;
          }
          
          if( i < (s32)min_frame_num)
          {
            for(; i>=fbm->max_frame_num; i--)
              fbm_free_frame_buffer(&fbm->frames[i].picture);
              
            printf("Fatal Error: no enough memory at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
            return NULL;
          }
          
          //* put all frame to empty frame queue.
          for(j = fbm->max_frame_num; j < i; j++)
          {
              fbm_enqueue(&fbm->empty_queue, &fbm->frames[i]);
#ifdef DECODE_USE_ASYNC_THREAD
              sem_post(&fbm->sem);
#endif
          }
          fbm->max_frame_num = j;
#ifdef DECODE_USE_ASYNC_THREAD
          fbm->first = 0;
#endif
      }    
  } else {
      fbm = (fbm_t*) mem_alloc(sizeof(fbm_t));
      if(!fbm)
        return NULL;
      
      mem_set(fbm, 0, sizeof(fbm_t));
    
      //* alloc memory frame buffer.
      for (i = 0; i < (s32)max_frame_num; i++)
      {
        fbm->frames[i].picture.id = i;
        fbm->frames[i].picture.size_y = size_y[0];
        fbm->frames[i].picture.size_u = size_u[0];
        fbm->frames[i].picture.size_v = size_v[0];
        fbm->frames[i].picture.size_alpha = size_alpha[0];
        fbm->frames[i].picture._3d_mode = out_3d_mode;
        
        if(out_3d_mode == _3D_MODE_DOUBLE_STREAM) {
            fbm->frames[i].picture.size_y2 = size_y[1];
            fbm->frames[i].picture.size_u2 = size_u[1];
            fbm->frames[i].picture.size_v2 = size_v[1];
            fbm->frames[i].picture.size_alpha2 = size_alpha[1];
        }
        
        if(fbm_alloc_frame_buffer(&fbm->frames[i].picture) != 0)
          break;
      }
      
      if( i < (s32)min_frame_num)
      {
        for(; i>=0; i--)
          fbm_free_frame_buffer(&fbm->frames[i].picture);
          
        mem_free(fbm);
        return NULL;
      }
      
      fbm->max_frame_num = i;
      
      //* put all frame to empty frame queue.
      for(i = 0; i < (s32)fbm->max_frame_num; i++)
      {
          fbm_enqueue(&fbm->empty_queue, &fbm->frames[i]);
      }

      pthread_mutex_init(&fbm->mutex, NULL);
#ifdef DECODE_USE_ASYNC_THREAD
      sem_init(&fbm->sem, 0, fbm->max_frame_num);
      fbm->first = 0;
#endif
      cedarx_fbm = fbm;
  }
  
  return (Handle)fbm;
}

static void fbm_free(void)
{
    u32 i;
    fbm_t*  fbm = cedarx_fbm;
  
    if(fbm) {
#ifdef DECODE_USE_ASYNC_THREAD
      sem_destroy(&fbm->sem);
#endif
      pthread_mutex_destroy(&fbm->mutex);
    
      for(i = 0; i < fbm->max_frame_num; i++)
      {
          fbm_free_frame_buffer(&fbm->frames[i].picture);
      }
      
      mem_free(fbm);
      cedarx_fbm = NULL;
    }
}

static void fbm_release(Handle h, void* parent)
{
    if (!cedarx_display)
        fbm_free();
}

static vpicture_t* fbm_request_decoder_frame(Handle h)
{
  fbm_t* fbm;
  display_frame_t* frame_info;
  vpicture_t* picture = NULL;
  
  fbm = (fbm_t*)h;
  
  if(fbm) {
#ifdef DECODE_USE_ASYNC_THREAD
    if (fbm->first) {
    	sem_wait(&fbm->sem);
    } else {
        sem_trywait(&fbm->sem);
    }
#endif
    pthread_mutex_lock(&fbm->mutex);
    frame_info = fbm_dequeue(&fbm->empty_queue);
    if(frame_info != NULL)
    {
        frame_info->status = FS_DECODER_USING;
        picture = &frame_info->picture;
    } else {
#ifdef DECODE_USE_ASYNC_THREAD
        if (!fbm->first) {
            fbm->first = 1;
        } else {
            printf("Fatal Error: mismatch queue with semapore at %s:%s:%d!!!!\n", __FILE__, __FUNCTION__, __LINE__);
        }
#endif
    } 
    
    pthread_mutex_unlock(&fbm->mutex);
  }
  
  return picture;
}

static void fbm_return_decoder_frame(vpicture_t* frame, u8 valid, Handle h)
{
    u8            idx;
    fbm_t*        fbm;
    display_frame_t* frame_info;
    
    fbm = (fbm_t*)h;

    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      idx = fbm_pointer_to_index(frame, h);
      if(idx < fbm->max_frame_num) {
        frame_info = &fbm->frames[idx];
    
        switch (frame_info->status)
        {
          case FS_DECODER_USING:
            if(valid)
            {
                frame_info->status = FS_PLAYER_USING;
                fbm_enqueue(&fbm->decoded_queue, frame_info);
            }
            else
            {
                frame_info->status = FS_EMPTY;
                fbm_enqueue(&fbm->empty_queue, frame_info);
#ifdef DECODE_USE_ASYNC_THREAD
                sem_post(&fbm->sem);
#endif
            }
            break;
          case FS_DECODER_SHARE_PLAYER_USING:
            frame_info->status = FS_PLAYER_USING;
            break;
          case FS_DECODER_SHARE_DISPLAY_USING:
            frame_info->status = FS_DISPLAY_USING;
            break;
          case FS_DECODER_SHARE_DISCARD:
            frame_info->status = FS_EMPTY;
            fbm_enqueue(&fbm->empty_queue, frame_info);
#ifdef DECODE_USE_ASYNC_THREAD
            sem_post(&fbm->sem);
#endif
            break;
          default:
            break;
        }
      }
        
      pthread_mutex_unlock(&fbm->mutex);
    }
}

static void fbm_share_decoder_frame(vpicture_t* frame, Handle h)
{
    u8            idx;
    fbm_t*        fbm;
    display_frame_t* frame_info;
    
    fbm = (fbm_t*)h;

    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      idx = fbm_pointer_to_index(frame, h);
      if(idx < fbm->max_frame_num) {
        frame_info = &fbm->frames[idx];
        frame_info->status = FS_DECODER_SHARE_PLAYER_USING;
        fbm_enqueue(&fbm->decoded_queue, frame_info);
      }
      pthread_mutex_unlock(&fbm->mutex);
    }
}

static vpicture_t* fbm_retrieve_picture(u32 id)
{
    u8 i;
    fbm_t* fbm = cedarx_fbm;
    vpicture_t* picture = NULL;
    
    if(fbm) {
        pthread_mutex_lock(&fbm->mutex);
        for(i = 0; i < fbm->max_frame_num; i++)
        {
            if(fbm->frames[i].picture.id == id) {
              picture = &fbm->frames[i].picture;
              break;            
            }
        }
        pthread_mutex_unlock(&fbm->mutex);
    }
   
    return picture;
}

static vpicture_t* fbm_request_display_frame(void)
{
    fbm_t* fbm = cedarx_fbm;
    display_frame_t* frame_info;
    vpicture_t* picture = NULL;
    
    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      frame_info = fbm_dequeue(&fbm->decoded_queue);
      if(frame_info != NULL) {
          picture = &frame_info->picture;
      }
      pthread_mutex_unlock(&fbm->mutex);
    }
    return picture;
}

static void fbm_return_display_frame(u32 idx)
{
    fbm_t* fbm = cedarx_fbm;
    display_frame_t* frame_info;
    
    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      frame_info = fbm_index_to_pointer(idx, fbm);
      if(frame_info != NULL) {
        frame_info->picture.anaglath_transform_mode = ANAGLAGH_NONE;
        switch (frame_info->status)
        {
          case FS_PLAYER_USING:
            frame_info->status = FS_EMPTY;
            fbm_enqueue(&fbm->empty_queue, frame_info);
#ifdef DECODE_USE_ASYNC_THREAD
            sem_post(&fbm->sem);
#endif
            break;
          case FS_DECODER_SHARE_PLAYER_USING:
            frame_info->status = FS_DECODER_SHARE_DISCARD;
            break;
          default:
            break;
        }
      }
      pthread_mutex_unlock(&fbm->mutex);
    }
}

static void fbm_share_display_frame(u32 idx)
{
    fbm_t* fbm = cedarx_fbm;
    display_frame_t* frame_info;
    
    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      if(idx < fbm->max_frame_num) {
        frame_info = &fbm->frames[idx];
        
        switch (frame_info->status)
        {
          case FS_PLAYER_USING:
            frame_info->status = FS_DISPLAY_USING;
            fbm_enqueue(&fbm->display_queue, frame_info);
            break;
          case FS_DECODER_SHARE_PLAYER_USING:
            frame_info->status = FS_DECODER_SHARE_DISPLAY_USING;
            fbm_enqueue(&fbm->display_queue, frame_info);
            break;
          default:
            break;
        }
      }
      pthread_mutex_unlock(&fbm->mutex);
    }    
}

static void fbm_flush_display_frame(u32 idx)
{
    fbm_t* fbm = cedarx_fbm;
    display_frame_t* frame_info;
    
    if(fbm) {
      pthread_mutex_lock(&fbm->mutex);
      
      while ((frame_info = fbm_peek(&fbm->display_queue))) {
        if (frame_info->picture.id == idx) 
            break;
        
        frame_info = fbm_dequeue(&fbm->display_queue); 
        frame_info->picture.anaglath_transform_mode = ANAGLAGH_NONE;
        switch (frame_info->status)
        {
          case FS_DISPLAY_USING:
            frame_info->status = FS_EMPTY;
            fbm_enqueue(&fbm->empty_queue, frame_info);
#ifdef DECODE_USE_ASYNC_THREAD
            sem_post(&fbm->sem);
#endif
            break;
          case FS_DECODER_SHARE_DISPLAY_USING:
            frame_info->status = FS_DECODER_SHARE_DISCARD;
            break;
          default:
            break;
        }
      }

      pthread_mutex_unlock(&fbm->mutex);
    }
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
    
  decoder = (cedarx_decoder_t*)mem_alloc(sizeof(cedarx_decoder_t));
  if (!decoder)
    return CEDARX_RESULT_NO_ENOUGH_MEMORY;

  mem_set(decoder, 0, sizeof(cedarx_decoder_t));
  cedarx_decoder = decoder;
  
  result = ve_init();
  if (CEDARX_RESULT_OK != result) 
    goto failed2;
          
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
      stream_info.format = STREAM_FORMAT_H264;
      stream_info.sub_format = STREAM_SUB_FORMAT_UNKNOW;
      break;
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
    if (!decoder->init_data)
        goto failed2;
    mem_cpy(decoder->init_data, info->data, info->data_size);
    stream_info.init_data = decoder->init_data;
    stream_info.init_data_len = info->data_size;
  } else {
    stream_info.init_data_len = 0;
    stream_info.init_data = NULL;
  }
  stream_info._3d_mode = _3D_MODE_NONE;

  mem_init();
  ve_request();

  decoder->ve = libve_open(&config, &stream_info, 0);
  if (!decoder->ve) {
    result = CEDARX_RESULT_VE_FAILED;
    goto failed1;
  }
    
  decoder->vbv = vbv_init();
  if (!decoder->vbv) {
    result = CEDARX_RESULT_NO_ENOUGH_MEMORY;
    goto failed1;
  }
    
  libve_set_vbv(decoder->vbv, decoder->ve);
 
  return CEDARX_RESULT_OK; 
   
failed1:
  ve_release();
  mem_exit();
  ve_exit();
  if (decoder->init_data)
    mem_free(decoder->init_data);
failed2:
  mem_free(decoder); 
  cedarx_decoder = NULL;
  return result;    
}

void libcedarx_decoder_close(void)
{
  cedarx_decoder_t* decoder = cedarx_decoder;

  if (decoder) {
    if (decoder->ve) {
      libve_reset(0, decoder->ve);  
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

cedarx_result_e libcedarx_decoder_decode_stream(void)
{
  vresult_e res;
  cedarx_result_e result;
  cedarx_decoder_t* decoder = cedarx_decoder;

  if ((!decoder) || (!decoder->ve)) 
    return CEDARX_RESULT_NO_INIT;

  res = libve_decode(0, 0, 0, decoder->ve);
#ifdef DECODE_USE_ASYNC_THREAD
  pthread_yield();
#endif
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

cedarx_result_e libcedarx_decoder_flush(void)
{
  cedarx_decoder_t* decoder = cedarx_decoder;

  if ((!decoder) || (!decoder->vbv)) 
    return CEDARX_RESULT_NO_INIT;

  libve_flush(1, decoder->ve);
  vbv_reset(decoder->vbv);
  
  return CEDARX_RESULT_OK;
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

    display->start = 0;
    display->init = 0;
    display->layer = -1;

    pthread_mutex_init(&display->mutex, NULL);
   
    return CEDARX_RESULT_OK; 
}

void libcedarx_display_close(void)
{
    unsigned long args[4];
    cedarx_display_t* display = cedarx_display;

    if (display) {
        if (display->layer != -1) {
            pthread_mutex_lock(&display->mutex);
            args[0] = 0;
            args[1] = display->layer;
            args[2] = 0;
            args[3] = 0;
            if (display->init) {
                if (display->start) {
                    while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->last_id) {
                        usleep(1000);    
                    } 
                    fbm_flush_display_frame(display->last_id);
                }
            
                ioctl(display->fd, DISP_CMD_VIDEO_STOP, args);
                ioctl(display->fd, DISP_CMD_LAYER_CLOSE, args );
            }
            
            ioctl(display->fd, DISP_CMD_LAYER_RELEASE, args);
            pthread_mutex_unlock(&display->mutex);
        }

        pthread_mutex_destroy(&display->mutex);
        close(display->fd);
        free(display);    
        cedarx_display = NULL;
        fbm_free();
        mem_exit();
    }
}

cedarx_result_e libcedarx_display_request_layer(void)
{
    cedarx_display_t* display = cedarx_display;
    unsigned long args[4];

    if (!display || (display->layer != -1))
        return CEDARX_RESULT_NO_INIT;
        
    if (display->layer != -1)
        return CEDARX_RESULT_NO_INIT;
        
    pthread_mutex_lock(&display->mutex);
    args[0] = 0;
    args[1] = DISP_LAYER_WORK_MODE_SCALER;
    args[2] = 0;
    args[3] = 0;
    display->layer = ioctl(display->fd, DISP_CMD_LAYER_REQUEST, args);
    pthread_mutex_unlock(&display->mutex);
    
    return CEDARX_RESULT_OK; 
}

void libcedarx_display_release_layer(void)
{
    unsigned long args[4];
    cedarx_display_t* display = cedarx_display;

    if (display && (display->layer != -1)) {
         pthread_mutex_lock(&display->mutex);
         args[0] = 0;
         args[1] = display->layer;
         args[2] = 0;
         args[3] = 0;
         if (display->init) {
             if (display->start) {
                 while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->last_id) {
                     usleep(1000);    
                 } 
                 fbm_flush_display_frame(display->last_id);
                 display->start = 0;
             }
         
             ioctl(display->fd, DISP_CMD_VIDEO_STOP, args);
             ioctl(display->fd, DISP_CMD_LAYER_CLOSE, args );
             display->init = 0;
         }
         
         ioctl(display->fd, DISP_CMD_LAYER_RELEASE, args);
         display->layer = -1;
         pthread_mutex_unlock(&display->mutex);
    }
}

cedarx_result_e libcedarx_display_video_frame(int idx)
{
    unsigned long args[4];
    vpicture_t* picture;
    cedarx_display_t* display = cedarx_display;

    if ((!display) || (display->layer == -1))
        return CEDARX_RESULT_NO_INIT;

    picture = fbm_retrieve_picture(idx);
    if (!picture)
        return CEDARX_RESULT_INVALID_ARGS;

    pthread_mutex_lock(&display->mutex);
    if (!display->init) {
        __disp_layer_info_t layer_info;
        u32 disp_width, disp_height;
        u32 screen_width, screen_height;
        args[0] = 0;
        args[1] = 0;
        args[2] = 0;
        args[3] = 0;
        screen_width = ioctl(display->fd, DISP_CMD_SCN_GET_WIDTH, args);
        screen_height = ioctl(display->fd, DISP_CMD_SCN_GET_HEIGHT, args);
        
        memset(&layer_info, 0, sizeof(layer_info));
        
        layer_info.mode = DISP_LAYER_WORK_MODE_SCALER;
        layer_info.fb.mode = DISP_MOD_MB_UV_COMBINED;
        layer_info.fb.format = DISP_FORMAT_YUV420;
        layer_info.fb.seq = DISP_SEQ_UVUV;
        layer_info.fb.br_swap = 0;
        layer_info.fb.addr[0] = mem_phy_addr((u32)picture->y);
        layer_info.fb.addr[1] = mem_phy_addr((u32)picture->u);
        
        if (picture->height < 720) {
          layer_info.fb.cs_mode = DISP_BT601;
        } else {
          layer_info.fb.cs_mode = DISP_BT709;
        }
        
        layer_info.src_win.x = picture->top_offset;
        layer_info.src_win.y = picture->left_offset;
        layer_info.src_win.width = picture->display_width;
        layer_info.src_win.height = picture->display_height;
        layer_info.fb.size.width = picture->width;
        layer_info.fb.size.height = picture->height; 
               
        if (screen_width * picture->display_height / picture->display_width > screen_height) {
            disp_width = screen_height * picture->display_width / picture->display_height;
            disp_height = screen_height;
        } else {
            disp_width = screen_width;
            disp_height = screen_width * picture->display_height / picture->display_width;
        }
        
        layer_info.scn_win.x = (screen_width - disp_width) / 2;
        layer_info.scn_win.y = (screen_height - disp_height) / 2;
        layer_info.scn_win.width = disp_width;
        layer_info.scn_win.height = disp_height;
        
        args[0] = 0;
        args[1] = display->layer;
        args[2] = (unsigned long)(&layer_info);
        args[3] = 0;
        ioctl(display->fd, DISP_CMD_LAYER_SET_PARA, args);
        ioctl(display->fd, DISP_CMD_LAYER_TOP, args );
        ioctl(display->fd, DISP_CMD_LAYER_OPEN, args );
        ioctl(display->fd, DISP_CMD_VIDEO_START, args);
        display->last_id = idx;
        display->init = 1;
    } else {
        __disp_video_fb_t video;
        args[0] = 0;
        args[1] = display->layer;
        args[2] = 0;
        args[3] = 0;
        if (display->start) {
            while (ioctl(display->fd, DISP_CMD_VIDEO_GET_FRAME_ID, args) != display->last_id) {
                usleep(1000);    
            } 
            fbm_flush_display_frame(display->last_id);
        } else {
            display->start = 1;
        }
    
        memset(&video, 0, sizeof(__disp_video_fb_t));
        video.id = idx;
        //video.interlace = picture->is_progressive ? 0 : 1;
        //video.top_field_first = picture->top_field_first ? 1 : 0;
        video.frame_rate = picture->frame_rate;
        video.addr[0] = mem_phy_addr((u32)picture->y);
        video.addr[1] = mem_phy_addr((u32)picture->u);
        
        args[0] = 0;
        args[1] = display->layer;
        args[2] = (unsigned long)(&video);
        args[3] = 0;
        ioctl(display->fd, DISP_CMD_VIDEO_SET_FB, args);
        fbm_share_display_frame(idx);
        display->last_id = idx;
    }

    pthread_mutex_unlock(&display->mutex);
    return CEDARX_RESULT_OK; 
}

cedarx_result_e libcedarx_display_request_frame(u32 *id, 
            u64 *pts, u32 *frame_rate, u32 *width, u32 *height)
{
  vpicture_t* vpicture = NULL;

  vpicture = fbm_request_display_frame();
  if (!vpicture) 
    return CEDARX_RESULT_NO_DISPLAY_BUFFER;
  
  *id = vpicture->id;
  *pts = vpicture->pts;
  *width = vpicture->width;   
  *height = vpicture->height;
  *frame_rate = vpicture->frame_rate;
  
  return CEDARX_RESULT_OK;
}

cedarx_result_e libcedarx_display_return_frame(int id)
{
  fbm_return_display_frame(id);
  return CEDARX_RESULT_OK;
}

char* libcedarx_version(void)
{
  return LIBCEDARX_VERSION;
}

int cedarv_f23_ic_version(void)
{
	return 1;
}
