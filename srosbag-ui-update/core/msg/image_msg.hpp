//
// Created by lhx on 18-05-31.
//

#ifndef SROS_CORE_MSG_IMAGE_MSG_H
#define SROS_CORE_MSG_IMAGE_MSG_H

#include "base_msg.h"

#include <memory>

#include <opencv2/core/core.hpp>

namespace sros {
namespace core {

class ImageMsg : public BaseMsg{
public:

    typedef uint16_t d_t;

    typedef d_t* dp_t;
    
    ImageMsg() : BaseMsg("", TYPE_IMAGE, true),
              width_(0),
              height_(0),
              data_(nullptr){}

    ImageMsg(topic_t topic)
            : BaseMsg(topic, TYPE_IMAGE, true),
              width_(0),
              height_(0),
              data_(nullptr)
    {

    };

    virtual ~ImageMsg() {
        delete [] data_;
    };

    virtual void getTime() {

    };

    uint64_t getTimestamp() const {
        return (uint64_t) time_;
    };

    void setTimestamp(uint64_t timestamp) {
        time_ = timestamp;
    };

    bool resize(int width, int height) {
        width_ = width;
        height_ = height;

        delete [] data_;
        data_ = new d_t[width_ * height_];
    }

    int getWidth() const {
        return width_;
    }

    int getHeight() const {
        return height_;
    }

    dp_t getData() {
        return data_;
    }

    void setCameraName(const std::string& name) {
        camera_ = name;
    }

    const std::string& getCameraName() {
        return camera_;
    }

    int getBytesPerPixel() const {
        return bytes_per_pixel_;
    }

    void setBytesPerPixel(int bytes_per_pixel) {
        bytes_per_pixel_ = bytes_per_pixel;
    }

    int getStrideInBytes() const {
        return stride_in_bytes_;
    }

    void setStrideInBytes(int stride_in_bytes) {
        stride_in_bytes_ = stride_in_bytes;
    }

    cv::Mat getMat() {
        return mat_;
    }

    void setMat(cv::Mat& mat) {
        mat_ = mat;
    }

    int width_, height_;

    cv::Mat mat_;

    dp_t data_;

    std::string camera_;

    int bytes_per_pixel_;

    int stride_in_bytes_;
};

typedef std::shared_ptr<ImageMsg> image_msg_ptr;

}
}


#endif // SROS_CORE_MSG_IMAGE_MSG_H
