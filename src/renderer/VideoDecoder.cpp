#include "VideoDecoder.hpp"
#include "../helpers/Log.hpp"
#include "../core/hyprlock.hpp"
#include "../core/Egl.hpp"
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>

CVideoDecoder::CVideoDecoder() {
    av_log_set_level(AV_LOG_WARNING);
    Debug::log(LOG, "VideoDecoder: Constructor called");
}

CVideoDecoder::~CVideoDecoder() {
    stop();
    
    if (m_pSwsCtx)
        sws_freeContext(m_pSwsCtx);
    
    if (m_pCodecCtx)
        avcodec_free_context(&m_pCodecCtx);
    
    if (m_pFormatCtx)
        avformat_close_input(&m_pFormatCtx);
    
    if (m_pHWDeviceCtx)
        av_buffer_unref(&m_pHWDeviceCtx);
    
    if (m_iVAOPBO[0]) {
        glDeleteBuffers(1, &m_iVAOPBO[1]);
        glDeleteVertexArrays(1, &m_iVAOPBO[0]);
    }
}

bool CVideoDecoder::load(const std::string& path) {
    Debug::log(LOG, "VideoDecoder: Loading video file: {}", path);
    
    // Open video file
    if (avformat_open_input(&m_pFormatCtx, path.c_str(), nullptr, nullptr) < 0) {
        Debug::log(ERR, "VideoDecoder: Failed to open video file: {}", path);
        return false;
    }
    Debug::log(LOG, "VideoDecoder: Video file opened successfully");
    
    // Get stream info
    if (avformat_find_stream_info(m_pFormatCtx, nullptr) < 0) {
        Debug::log(ERR, "VideoDecoder: Failed to find stream info");
        return false;
    }
    
    // Find video stream
    for (unsigned int i = 0; i < m_pFormatCtx->nb_streams; i++) {
        if (m_pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            m_iVideoStreamIndex = i;
            m_pVideoStream = m_pFormatCtx->streams[i];
            break;
        }
    }
    
    if (m_iVideoStreamIndex == -1) {
        Debug::log(ERR, "VideoDecoder: No video stream found");
        return false;
    }
    
    // Get codec
    m_pCodec = avcodec_find_decoder(m_pVideoStream->codecpar->codec_id);
    if (!m_pCodec) {
        Debug::log(ERR, "VideoDecoder: Codec not found");
        return false;
    }
    
    // Create codec context
    m_pCodecCtx = avcodec_alloc_context3(m_pCodec);
    if (!m_pCodecCtx) {
        Debug::log(ERR, "VideoDecoder: Failed to allocate codec context");
        return false;
    }
    
    if (avcodec_parameters_to_context(m_pCodecCtx, m_pVideoStream->codecpar) < 0) {
        Debug::log(ERR, "VideoDecoder: Failed to copy codec parameters");
        return false;
    }
    
    // Try to initialize hardware acceleration
    m_bHWAccelEnabled = initHWAccel();
    
    // Open codec
    if (avcodec_open2(m_pCodecCtx, m_pCodec, nullptr) < 0) {
        Debug::log(ERR, "VideoDecoder: Failed to open codec");
        return false;
    }
    
    // Get video properties
    m_iWidth = m_pCodecCtx->width;
    m_iHeight = m_pCodecCtx->height;
    AVRational fps = av_guess_frame_rate(m_pFormatCtx, m_pVideoStream, nullptr);
    m_fFPS = fps.num ? (float)fps.num / (float)fps.den : 30.0f;
    m_fDuration = (float)m_pFormatCtx->duration / AV_TIME_BASE;
    
    Debug::log(LOG, "VideoDecoder: Loaded video {}x{} @ {:.2f} FPS, duration: {:.2f}s, HW accel: {}", 
               m_iWidth, m_iHeight, m_fFPS, m_fDuration, m_bHWAccelEnabled);
    
    Debug::log(LOG, "VideoDecoder: Codec: {}, Pixel Format: {}", 
               m_pCodec->name, av_get_pix_fmt_name((AVPixelFormat)m_pCodecCtx->pix_fmt));
    
    // Store frame buffer size for later use
    m_iFrameBufferSize = m_iWidth * m_iHeight * 4; // RGBA
    
    Debug::log(LOG, "VideoDecoder: Video loaded successfully, ready to play");
    
    return true;
}

bool CVideoDecoder::initHWAccel() {
    // Try different hardware acceleration methods in order of preference
    const enum AVHWDeviceType hwTypes[] = {
        AV_HWDEVICE_TYPE_VAAPI,   // Intel/AMD on Linux
        AV_HWDEVICE_TYPE_CUDA,    // NVIDIA
        AV_HWDEVICE_TYPE_VDPAU,   // Older NVIDIA on Linux
        AV_HWDEVICE_TYPE_DRM,     // Direct Rendering Manager
        AV_HWDEVICE_TYPE_NONE
    };
    
    for (int i = 0; hwTypes[i] != AV_HWDEVICE_TYPE_NONE; i++) {
        m_HWDeviceType = hwTypes[i];
        
        // Check if codec supports this HW type
        for (int j = 0;; j++) {
            const AVCodecHWConfig* config = avcodec_get_hw_config(m_pCodec, j);
            if (!config)
                break;
            
            if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
                config->device_type == m_HWDeviceType) {
                m_HWPixFormat = config->pix_fmt;
                
                // Try to create HW device context
                if (av_hwdevice_ctx_create(&m_pHWDeviceCtx, m_HWDeviceType, nullptr, nullptr, 0) >= 0) {
                    m_pCodecCtx->hw_device_ctx = av_buffer_ref(m_pHWDeviceCtx);
                    Debug::log(LOG, "VideoDecoder: Using hardware acceleration: {}", 
                               av_hwdevice_get_type_name(m_HWDeviceType));
                    return true;
                }
            }
        }
    }
    
    Debug::log(WARN, "VideoDecoder: Hardware acceleration not available, using software decoding");
    return false;
}

void CVideoDecoder::start() {
    if (m_bPlaying)
        return;
    
    Debug::log(LOG, "VideoDecoder: Starting playback");
    
    m_bPlaying = true;
    m_bShouldStop = false;
    m_PlaybackStartTime = std::chrono::steady_clock::now();
    
    // Start decoding thread
    m_DecodingThread = std::thread(&CVideoDecoder::decodingThread, this);
}

void CVideoDecoder::stop() {
    if (!m_bPlaying)
        return;
    
    m_bShouldStop = true;
    m_FrameQueueCV.notify_all();
    
    if (m_DecodingThread.joinable())
        m_DecodingThread.join();
    
    m_bPlaying = false;
    
    // Clear frame queue
    std::lock_guard<std::mutex> lock(m_FrameQueueMutex);
    while (!m_FrameQueue.empty())
        m_FrameQueue.pop();
}

void CVideoDecoder::decodingThread() {
    AVPacket* packet = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* swFrame = m_bHWAccelEnabled ? av_frame_alloc() : nullptr;
    
    while (!m_bShouldStop) {
        // Check if queue is full
        {
            std::unique_lock<std::mutex> lock(m_FrameQueueMutex);
            if (m_FrameQueue.size() >= MAX_FRAME_QUEUE_SIZE) {
                m_FrameQueueCV.wait_for(lock, std::chrono::milliseconds(10));
                continue;
            }
        }
        
        // Read packet
        if (av_read_frame(m_pFormatCtx, packet) < 0) {
            if (m_bLoop) {
                // Seek to beginning
                av_seek_frame(m_pFormatCtx, m_iVideoStreamIndex, 0, AVSEEK_FLAG_BACKWARD);
                avcodec_flush_buffers(m_pCodecCtx);
                continue;
            } else {
                break;
            }
        }
        
        if (packet->stream_index != m_iVideoStreamIndex) {
            av_packet_unref(packet);
            continue;
        }
        
        // Decode frame
        int ret = avcodec_send_packet(m_pCodecCtx, packet);
        if (ret < 0) {
            av_packet_unref(packet);
            continue;
        }
        
        while (ret >= 0) {
            ret = avcodec_receive_frame(m_pCodecCtx, frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            
            if (ret < 0) {
                Debug::log(ERR, "VideoDecoder: Error decoding frame");
                break;
            }
            
            // Handle hardware accelerated frame
            AVFrame* processFrame = frame;
            if (m_bHWAccelEnabled && frame->format == m_HWPixFormat) {
                // Transfer data from GPU to CPU
                if (av_hwframe_transfer_data(swFrame, frame, 0) < 0) {
                    Debug::log(ERR, "VideoDecoder: Failed to transfer HW frame");
                    continue;
                }
                processFrame = swFrame;
            }
            
            // Convert and upload to texture
            convertFrameToTexture(processFrame);
        }
        
        av_packet_unref(packet);
    }
    
    av_packet_free(&packet);
    av_frame_free(&frame);
    if (swFrame)
        av_frame_free(&swFrame);
}

void CVideoDecoder::convertFrameToTexture(AVFrame* frame) {
    auto newFrame = makeUnique<Frame>();
    
    // Calculate timestamp properly
    double pts = 0.0;
    if (frame->pts != AV_NOPTS_VALUE && m_pVideoStream->time_base.den > 0) {
        pts = frame->pts * av_q2d(m_pVideoStream->time_base);
    } else {
        // Use frame count as fallback
        static int frameCount = 0;
        pts = frameCount++ / m_fFPS;
    }
    newFrame->timestamp = pts;
    newFrame->pts = std::chrono::steady_clock::now();
    
    Debug::log(TRACE, "VideoDecoder: Converting frame with PTS: {:.2f}", pts);
    
    // Store frame dimensions
    newFrame->width = m_iWidth;
    newFrame->height = m_iHeight;
    
    // Setup conversion context if needed
    if (!m_pSwsCtx) {
        m_pSwsCtx = sws_getContext(
            frame->width, frame->height, (AVPixelFormat)frame->format,
            m_iWidth, m_iHeight, AV_PIX_FMT_RGBA,
            SWS_BILINEAR, nullptr, nullptr, nullptr
        );
        if (!m_pSwsCtx) {
            Debug::log(ERR, "VideoDecoder: Failed to create SwsContext");
            return;
        }
        Debug::log(LOG, "VideoDecoder: Created SwsContext for {}x{} -> {}x{}", 
                   frame->width, frame->height, m_iWidth, m_iHeight);
    }
    
    // Allocate buffer for RGBA data (store in frame, don't upload to texture yet)
    newFrame->rgbaData.resize(m_iWidth * m_iHeight * 4);
    uint8_t* dstData[4] = {newFrame->rgbaData.data(), nullptr, nullptr, nullptr};
    int dstLinesize[4] = {m_iWidth * 4, 0, 0, 0};
    
    // Convert to RGBA
    int ret = sws_scale(m_pSwsCtx, 
                       frame->data, frame->linesize, 0, frame->height,
                       dstData, dstLinesize);
    
    if (ret <= 0) {
        Debug::log(ERR, "VideoDecoder: sws_scale failed with return value: {}", ret);
        return;
    }
    
    Debug::log(TRACE, "VideoDecoder: Frame converted to RGBA data ({}x{} = {} bytes)", 
               m_iWidth, m_iHeight, newFrame->rgbaData.size());
    
    // Add to queue
    {
        std::lock_guard<std::mutex> lock(m_FrameQueueMutex);
        m_FrameQueue.push(std::move(newFrame));
    }
    m_FrameQueueCV.notify_one();
}

CTexture* CVideoDecoder::getCurrentFrame() {
    if (!m_bPlaying) {
        Debug::log(TRACE, "VideoDecoder: Not playing, returning nullptr");
        return nullptr;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_PlaybackStartTime).count() / 1000.0f;
    
    // Get frame from queue that matches current playback time
    std::lock_guard<std::mutex> lock(m_FrameQueueMutex);
    
    bool frameUpdated = false;
    while (!m_FrameQueue.empty()) {
        auto& frontFrame = m_FrameQueue.front();
        
        // Check if this frame should be displayed
        if (frontFrame->timestamp <= elapsed) {
            m_pCurrentFrame = std::move(m_FrameQueue.front());
            m_FrameQueue.pop();
            m_fCurrentTime = m_pCurrentFrame->timestamp;
            m_FrameQueueCV.notify_one();
            frameUpdated = true;
        } else {
            break;
        }
    }
    
    if (frameUpdated) {
        Debug::log(TRACE, "VideoDecoder: Displaying frame at time {:.2f}, queue size: {}", 
                   m_fCurrentTime, m_FrameQueue.size());
    }
    
    if (!m_pCurrentFrame) {
        Debug::log(TRACE, "VideoDecoder: No current frame available yet");
        return nullptr;
    }
    
    // Create texture on-demand in main thread (this runs in main thread during rendering)
    if (!m_pCurrentFrame->textureUploaded) {
        Debug::log(TRACE, "VideoDecoder: Creating texture in main thread");
        
        m_pCurrentFrame->texture.allocate();
        if (m_pCurrentFrame->texture.m_iTexID == 0) {
            Debug::log(ERR, "VideoDecoder: Failed to allocate texture in main thread!");
            return nullptr;
        }
        
        // Upload RGBA data to texture
        glBindTexture(GL_TEXTURE_2D, m_pCurrentFrame->texture.m_iTexID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_pCurrentFrame->width, m_pCurrentFrame->height, 0, 
                     GL_RGBA, GL_UNSIGNED_BYTE, m_pCurrentFrame->rgbaData.data());
        
        // Check for errors
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            Debug::log(ERR, "VideoDecoder: OpenGL error during main thread texture upload: 0x{:x}", error);
            return nullptr;
        }
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
        m_pCurrentFrame->texture.m_vSize = Vector2D(m_pCurrentFrame->width, m_pCurrentFrame->height);
        m_pCurrentFrame->texture.m_iType = TEXTURE_RGBA;
        
        glBindTexture(GL_TEXTURE_2D, 0);
        
        m_pCurrentFrame->textureUploaded = true;
        Debug::log(TRACE, "VideoDecoder: Texture uploaded successfully in main thread, ID: {}", 
                   m_pCurrentFrame->texture.m_iTexID);
    }
    
    return &m_pCurrentFrame->texture;
}

void CVideoDecoder::seek(float timeInSeconds) {
    int64_t timestamp = timeInSeconds * AV_TIME_BASE;
    av_seek_frame(m_pFormatCtx, -1, timestamp, AVSEEK_FLAG_BACKWARD);
    avcodec_flush_buffers(m_pCodecCtx);
    
    // Clear frame queue
    std::lock_guard<std::mutex> lock(m_FrameQueueMutex);
    while (!m_FrameQueue.empty())
        m_FrameQueue.pop();
    
    m_fCurrentTime = timeInSeconds;
    m_PlaybackStartTime = std::chrono::steady_clock::now() - std::chrono::milliseconds((int)(timeInSeconds * 1000));
}