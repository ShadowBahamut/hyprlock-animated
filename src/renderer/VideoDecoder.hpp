#pragma once

#include "../defines.hpp"
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <hyprutils/memory/UniquePtr.hpp>
#include "Texture.hpp"
#include <GLES3/gl32.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
}

class CVideoDecoder {
  public:
    CVideoDecoder();
    ~CVideoDecoder();

    bool            load(const std::string& path);
    void            start();
    void            stop();
    bool            isPlaying() const { return m_bPlaying; }
    
    // Get current frame as OpenGL texture
    CTexture*       getCurrentFrame();
    
    // Video properties
    float           getDuration() const { return m_fDuration; }
    float           getCurrentTime() const { return m_fCurrentTime; }
    int             getWidth() const { return m_iWidth; }
    int             getHeight() const { return m_iHeight; }
    float           getFPS() const { return m_fFPS; }
    
    // Control
    void            setLoop(bool loop) { m_bLoop = loop; }
    void            seek(float timeInSeconds);
    
  private:
    struct Frame {
        std::vector<uint8_t>                 rgbaData;
        std::chrono::steady_clock::time_point pts;
        float                                timestamp;
        int                                  width;
        int                                  height;
        bool                                 textureUploaded = false;
        mutable CTexture                     texture; // Created on-demand in main thread
    };
    
    void            decodingThread();
    bool            initHWAccel();
    bool            decodeFrame();
    void            convertFrameToTexture(AVFrame* frame);
    void            uploadToGPU(AVFrame* frame);
    
    // FFmpeg contexts
    AVFormatContext*                m_pFormatCtx = nullptr;
    AVCodecContext*                 m_pCodecCtx = nullptr;
    const AVCodec*                  m_pCodec = nullptr;
    AVBufferRef*                    m_pHWDeviceCtx = nullptr;
    SwsContext*                     m_pSwsCtx = nullptr;
    AVStream*                       m_pVideoStream = nullptr;
    
    // Hardware acceleration
    enum AVHWDeviceType             m_HWDeviceType = AV_HWDEVICE_TYPE_NONE;
    enum AVPixelFormat              m_HWPixFormat = AV_PIX_FMT_NONE;
    bool                           m_bHWAccelEnabled = false;
    
    // Video properties
    int                            m_iVideoStreamIndex = -1;
    int                            m_iWidth = 0;
    int                            m_iHeight = 0;
    float                          m_fFPS = 0.0f;
    float                          m_fDuration = 0.0f;
    float                          m_fCurrentTime = 0.0f;
    
    // Playback state
    std::atomic<bool>              m_bPlaying{false};
    std::atomic<bool>              m_bShouldStop{false};
    bool                           m_bLoop = true;
    
    // Threading
    std::thread                    m_DecodingThread;
    std::mutex                     m_FrameQueueMutex;
    std::condition_variable        m_FrameQueueCV;
    std::queue<UP<Frame>>          m_FrameQueue;
    static constexpr size_t        MAX_FRAME_QUEUE_SIZE = 3;
    
    // Current frame
    std::mutex                     m_CurrentFrameMutex;
    UP<Frame>                      m_pCurrentFrame;
    std::chrono::steady_clock::time_point m_PlaybackStartTime;
    
    // OpenGL/EGL for texture upload
    GLuint                         m_iVAOPBO[2] = {0, 0}; // VAO and PBO for async upload
    size_t                         m_iFrameBufferSize = 0;
    
    // Error handling
    std::string                    m_sLastError;
};