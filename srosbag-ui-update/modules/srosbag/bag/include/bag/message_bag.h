/**
 * @file message_bag.hpp
 * @author zmy (626670628@qq.com)
 * @brief 基于boost::serialize录制和恢复数据的类
 * @version 0.1
 * @date 2021-05-18
 * 
 * 
 */

#ifndef MESSAGE_BAG_HPP
#define MESSAGE_BAG_HPP

#define USE_EOS

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <google/protobuf/message.h>

#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#ifdef USE_EOS
#include "eos/portable_iarchive.hpp"
#include "eos/portable_oarchive.hpp"
#else
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#endif

#include "../../../serialize_patch/headers.h"
#include "factory/my_factory.h"

#include "../../../../core/msg/base_msg.h"
#include "../../../../core/util/time.h"
// #include <limits>

namespace bag
{

    class MsgBag : public std::enable_shared_from_this<MsgBag>
    {

      

    public:
      typedef struct 
      {
        int64_t begin_time_;
        uint64_t end_time_;
        
        uint64_t time_stamp_;

      }TimeProgress;
        
        ~MsgBag() = default;
        void playBack(std::string bag_file, const bool blocking = false, const bool play_with_time = true);
        void stopRecord();
        void startRecord(const std::vector<std::string> &record_msgs);
        void printAllMsgType();
        void setSpeed(const float speed);
        void setSwitchTime(const uint64_t time);
        void setPause();
        void closePlay();
        inline void setCompressBag(const bool is_compress) { compress_bag_file_ = is_compress; }
        inline void nextStep() { step_ = true; }
       
        
        static std::shared_ptr<MsgBag> Create(const std::string &bag_dir = "/sros/message_bag");

        std::vector<std::string> qtMsgType();//anxixu 返回topic
        TimeProgress timeGet(); //anxixu
        int qtShow(int argc, char *argv[]);//anxixu


    public:
        template <typename Msg>
        auto dumpMsg(const Msg &msg, const std::string &topic);

        template <typename Msg>
        auto setMsgHandle(const std::function<void(const Msg &)> &handle, const std::string &topic = "");
       // MsgBag(const std::string &bag_dir = "/sros/message_bag");//TODO:   private->public  anxixu
    private:
        // MsgBag() = default;
        MsgBag(const std::string &bag_dir = "/sros/message_bag");
        void createBag(const uint64_t &start_time);
        auto playImp(const std::string &bag_file);
        void setMinDeltaTime(std::ifstream &bag);
        void setEndTime(std::ifstream &bag);
        void getPlayOperator();
        bool playOneMsg(std::string &msg_handler);

        template <typename Msg>
        auto playDetail(Msg &&msg, const std::string &msg_header, const uint64_t stamp);
        template <typename Msg>
        auto writeMsgHeader(const std::string &topic);
        template <typename Msg>
        auto setTime(Msg &&msg, const std::string &header, uint64_t &time);

        template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> * = nullptr>
        auto save(const Msg &msg, const std::string &topic, const uint64_t time);

        //zmy XXX: proto是否有一个统一的基类
        template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                                 std::is_base_of<::google::protobuf::MessageLite, Msg>::value> * = nullptr>
        auto save(const Msg &msg, const std::string &topic, const uint64_t time);

        template <typename Msg>
        auto handleMsg(Msg &msg, const std::string &msg_header);

        template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> * = nullptr>
        auto loadMsg(Msg &msg, const std::string &topic);

        template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                                 std::is_base_of<::google::protobuf::MessageLite, Msg>::value> * = nullptr>
        auto loadMsg(Msg &msg, const std::string &topic);


    protected:
        //int64_t begin_time_;
        //uint64_t end_time_;
        
        //uint64_t time_stamp_; //anxixu添加
        std::vector<std::string> msg_vector;
        std::string qt_msg_type;

    private:
        std::atomic<bool> pause_;
        std::atomic<bool> playing_;
        std::atomic<bool> step_;
        std::atomic<bool> time_changed_;
        std::atomic<bool> play_with_time_;
        std::atomic<bool> compress_bag_file_;
        std::atomic<float> speed_;
        double time_marker_;
         int64_t begin_time_;
         uint64_t end_time_;
        uint64_t min_delta_time_;
        uint64_t time_stamp_; //anxixu添加
        std::atomic<int64_t> switch_time_; /// 用于进度条控制
        int begin_cur_;                    ///数据在fsteam中的开始位置
        int end_cur_;                      /// 数据在fsteam中的结束位置
        std::string end_mark_;
        std::string bag_file_;
        std::string bag_dir_;
        std::shared_ptr<factory::Factory> factory_;
        std::unordered_map<std::string, boost::any> msg_handles_;
        std::vector<bool> use_msg_;
        std::vector<std::string> record_topics_;
        std::unordered_map<std::string, bool> is_compress_;
        std::ofstream record_bag_;
        std::ifstream play_bag_;
        std::mutex record_mutex_;
        std::mutex play_mutex_;
        std::future<void> play_thread_;
        std::future<void> play_operator_;
    };

    template <typename Msg>
    auto MsgBag::dumpMsg(const Msg &msg, const std::string &topic)
    {
        std::string msg_topic = topic;
        if (use_msg_[Container::typeToPos<Msg>()] == true && std::find(record_topics_.begin(), record_topics_.end(), msg_topic) != record_topics_.end())
        {
            std::lock_guard<std::mutex> lock(record_mutex_);
            auto time = sros::core::util::get_timestamp_in_us();
            if (!record_bag_.is_open())
            {
                createBag(time);
            }
            save<Msg>(msg, msg_topic, time);
        }
    }

    template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> *>
    auto MsgBag::save(const Msg &msg, const std::string &topic, const uint64_t time)
    {
        // std::lock_guard<std::mutex> lock(record_mutex_);

        record_bag_ << time << ":" << topic << std::endl;
        if (factory_->isCompress(topic))
        {
            std::ostringstream oss;
            {
                boost::iostreams::filtering_stream<boost::iostreams::output> out;
                out.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_speed));
                out.push(oss);
#ifdef USE_EOS
                eos::portable_oarchive oa(out);
#else
                boost::archive::text_oarchive oa(out);
#endif
                oa << msg;
            }
            record_bag_ << oss.str() << end_mark_ << std::endl;
        }
        else
        {
#ifdef USE_EOS
            eos::portable_oarchive oa(record_bag_);
#else
            boost::archive::text_oarchive oa(record_bag_);
#endif
            oa << msg;
            record_bag_ << end_mark_ << std::endl;
        }

        return;
    }

    template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                             std::is_base_of<::google::protobuf::MessageLite, Msg>::value> *>
    auto MsgBag::save(const Msg &msg, const std::string &topic, const uint64_t time)
    {
        //zmy XXX:未测试
        // std::lock_guard<std::mutex> lock(record_mutex_);

        record_bag_ << time << ":" << topic << std::endl;
        if (factory_->isCompress(topic))
        {
            std::string data;
            msg.SerializeToString(&data);

            std::stringstream compressed;
            std::stringstream origin(data);

            boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
            out.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_speed));
            out.push(origin);
            boost::iostreams::copy(out, compressed);
            record_bag_ << compressed.str() << end_mark_ << std::endl;
        }
        else
        {
            std::string data;
            msg.SerializeToString(&data);
            record_bag_ << data << end_mark_ << std::endl;
        }
    }

    template <typename Msg>
    auto MsgBag::setMsgHandle(const std::function<void(const Msg &)> &handle, const std::string &topic)
    {
        if (topic == "")
            return;
        if (msg_handles_.find(topic) == msg_handles_.end())
        {
            msg_handles_[topic] = handle;
        }
        else
        {
            LOG(WARNING) << "The handle of " << topic << " is repeate setting,please check it!!!";
        }
        return;
    }

    template <typename Msg>
    auto MsgBag::handleMsg(Msg &msg, const std::string &msg_header)
    {
        if (msg_handles_.find(msg_header) != msg_handles_.end())
        {
            auto &any_handle = msg_handles_[msg_header];
            if (!any_handle.empty())
            {
                auto handle_msg = boost::any_cast<std::function<void(const Msg &)>>(any_handle);
                handle_msg(msg);
            }
            else
            {
                LOG(WARNING) << "not set handle message function for " << msg_header << "!!!";
            }
        }
    }

    template <typename Msg, std::enable_if_t<!std::is_base_of<::google::protobuf::Message, Msg>::value> *>
    auto MsgBag::loadMsg(Msg &msg, const std::string &topic)
    {
        if (is_compress_.at(topic))
        {
            std::string msg_data;
            getline(play_bag_, msg_data);
            auto pos = msg_data.rfind(end_mark_);
            while (pos == std::string::npos || (msg_data.size() - pos != end_mark_.size()))
            {
                std::string app;
                getline(play_bag_, app);
                msg_data = msg_data + '\n' + app;
                pos = msg_data.rfind(end_mark_);
            }
            msg_data.erase(pos);

            std::istringstream iss(msg_data);
            boost::iostreams::filtering_stream<boost::iostreams::input> in;
            in.push(boost::iostreams::zlib_decompressor());
            in.push(iss);
#ifdef USE_EOS
            eos::portable_iarchive ia(in);
#else
            boost::archive::text_iarchive ia(in);
#endif
            ia >> msg;
        }
        else
        {
#ifdef USE_EOS
            eos::portable_iarchive ia(play_bag_);
#else
            boost::archive::text_iarchive ia(play_bag_);
#endif
            ia >> msg;
            //移除流中的msgType
            std::string app;
            getline(play_bag_, app);
            if (app != end_mark_)
            {
                LOG(WARNING) << "The postfix of " << topic << " is not match!!!";
            }
        }

        return;
    }

    template <typename Msg, std::enable_if_t<std::is_base_of<::google::protobuf::Message, Msg>::value ||
                                             std::is_base_of<::google::protobuf::MessageLite, Msg>::value> *>
    auto MsgBag::loadMsg(Msg &msg, const std::string &topic)
    {
        //zmy XXX:未测试
        std::string msg_data;
        getline(play_bag_, msg_data);
        auto pos = msg_data.rfind(end_mark_);
        while (pos == std::string::npos || (msg_data.size() - pos != end_mark_.size()))
        {
            std::string app;
            getline(play_bag_, app);
            msg_data = msg_data + '\n' + app;
            pos = msg_data.rfind(end_mark_);
        }
        msg_data.erase(pos);

        if (is_compress_.at(topic))
        {
            std::stringstream compressed(msg_data);
            std::stringstream decompressed;

            boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
            out.push(boost::iostreams::zlib_decompressor());
            out.push(compressed);
            boost::iostreams::copy(out, decompressed);

            msg.ParseFromString(decompressed.str());
        }
        else
        {
            msg.ParseFromString(msg_data);
        }

        //zmy TODO: return time_stamp
        // return time_stamp
        return;
    }

    template <typename Msg>
    auto MsgBag::playDetail(Msg &&msg, const std::string &msg_header, const uint64_t stamp)
    {

        time_stamp_=stamp;
        if (stamp < begin_time_ || stamp > end_time_)
            return;
        static bool jump_forward = false;
        static bool jump_backward = false;
        if (switch_time_ == -1)
        {
            loadMsg<Msg>(msg, msg_header);
            //zmy TODO: 时间有效性判断
            if (time_changed_)
            {
                time_marker_ = sros::core::util::get_timestamp_in_us() - static_cast<double>((stamp - begin_time_) / speed_);
                time_changed_ = false;
            }

            if (play_with_time_)
            {
                std::this_thread::sleep_until(std::chrono::time_point<
                                              std::chrono::high_resolution_clock, std::chrono::duration<double, std::micro>>(
                    std::chrono::duration<double, std::micro>(static_cast<double>((stamp - begin_time_) / speed_ + time_marker_))));
            }

            handleMsg<Msg>(msg, msg_header);

             if (end_time_ != 0)
                 LOG(INFO) << "Bag Time:" << stamp
                           << "  Duration: " << (stamp - begin_time_) / 1e6 << " / "
                           << (end_time_ - begin_time_) / 1e6;
        }
        else if (!jump_forward && static_cast<int64_t>(time_stamp_) - switch_time_ > 1.2 * min_delta_time_)
        {


            jump_backward = true;
            int pos = (static_cast<double>(switch_time_ - begin_time_) * 0.8) / (end_time_ - begin_time_) * (end_cur_ - begin_cur_);
            play_bag_.seekg(pos, std::ios_base::beg);
            time_changed_ = true;
            
            return;
        }
        else if (!jump_backward && switch_time_ - static_cast<int64_t>(time_stamp_) > 1.2 * min_delta_time_)
        {
            time_changed_ = true;
            jump_forward = true;
            return;
        }
        else
        {
            switch_time_ = -1;
            time_changed_ = true;
            jump_forward = false;
            jump_backward = false;
            return;
        }
    }

    template <typename Msg>
    auto MsgBag::writeMsgHeader(const std::string &topic)
    {
        LOG(INFO) << topic << "\n";
        // std::lock_guard<std::mutex> lock(record_mutex_);
        record_bag_ << topic << "," << factory_->isCompress(topic) << ';';
    }

    template <typename Msg>
    auto MsgBag::setTime(Msg &&msg, const std::string &header, uint64_t &time)
    {
        time = loadMsg<Msg>(msg, header);
    }

} // namespace bag

#endif
