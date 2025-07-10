// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/**
 * @file HelloWorldPublisher.cpp
 *
 */

#include "HelloWorldPubSubTypes.hpp"

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

class HelloWorldPublisher
{
private:

    HelloWorld hello_; // 用于存储要发布的数据

    DomainParticipant* participant_; // DDS 域参与者，负责管理通信域

    Publisher* publisher_; // 发布者对象

    Topic* topic_; // 主题对象，定义数据的类型和名称

    DataWriter* writer_; // 数据写入器，用于向主题发布数据

    TypeSupport type_; // 类型支持，用于序列化和反序列化数据

    // 内部类：监听器，用于监听 DataWriter 的事件
    class PubListener : public DataWriterListener
    {
    public:

        PubListener()
            : matched_(0) // 初始化匹配计数器
        {
        }

        ~PubListener() override
        {
        }

        // 当发布者与订阅者匹配时触发
        void on_publication_matched(
                DataWriter*,
                const PublicationMatchedStatus& info) override
        {
            if (info.current_count_change == 1) // 新的订阅者匹配
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1) // 订阅者断开
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else // 无效的匹配状态
            {
                std::cout << info.current_count_change
                        << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_; // 匹配的订阅者数量

    } listener_; // 监听器实例

public:

    // 构造函数，初始化成员变量
    HelloWorldPublisher()
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , type_(new HelloWorldPubSubType()) // 初始化类型支持
    {
    }

    // 析构函数
    virtual ~HelloWorldPublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_); // 删除数据写入器
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_); // 删除发布者
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_); // 删除主题
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_); // 删除域参与者
    }

    // 初始化发布者
    bool init()
    {
        hello_.index(0); // 初始化数据索引
        hello_.message("HelloWorld"); // 初始化消息内容

        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher"); // 设置域参与者名称
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // 注册数据类型
        type_.register_type(participant_);

        // 创建话题
        topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // 创建发布者
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // 创建数据写入器
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    // 发布数据
    bool publish()
    {
        if (listener_.matched_ > 0) // 确保至少有一个订阅者
        {
            hello_.index(hello_.index() + 1); // 更新数据索引
            writer_->write(&hello_); // 写入数据
            return true;
        }
        return false; // 无订阅者，发布失败
    }

    // 运行发布者
    void run(
            uint32_t samples) // 发布的消息数量
    {
        uint32_t samples_sent = 0;
        while (samples_sent < samples)
        {
            if (publish())
            {
                samples_sent++;
                std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                            << " SENT" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 每秒发布一次
        }
    }
};

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting publisher." << std::endl;
    uint32_t samples = 10; // 要发布的消息数量

    HelloWorldPublisher* mypub = new HelloWorldPublisher(); // 创建发布者实例
    if(mypub->init())
    {
        mypub->run(samples);
    }

    delete mypub;
    return 0;
}
