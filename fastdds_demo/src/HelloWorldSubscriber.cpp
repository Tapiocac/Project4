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
 * @file HelloWorldSubscriber.cpp
 *
 */

#include "HelloWorldPubSubTypes.hpp"

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

// 定义一个订阅者类
class HelloWorldSubscriber
{
private:

    DomainParticipant* participant_; // DDS 域参与者，负责管理通信域

    Subscriber* subscriber_; // 订阅者对象

    DataReader* reader_; // 数据读取器，用于从话题接收数据

    Topic* topic_; // 话题对象，定义数据的类型和名称

    TypeSupport type_; // 类型支持，用于序列化和反序列化数据

    // 内部类：监听器，用于监听 DataReader 的事件
    class SubListener : public DataReaderListener
    {
    public:

        SubListener()
            : samples_(0) // 初始化接收的样本计数
        {
        }

        ~SubListener() override
        {
        }

        // 当订阅者与发布者匹配时触发
        void on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1) // 新的发布者匹配
            {
                std::cout << "Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1) // 发布者断开
            {
                std::cout << "Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                          << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        // 当有新数据可用时触发
        void on_data_available(
                DataReader* reader) override
        {
            SampleInfo info; // 样本信息
            if (reader->take_next_sample(&hello_, &info) == eprosima::fastdds::dds::RETCODE_OK)
            {
                if (info.valid_data) // 检查数据是否有效
                {
                    samples_++; // 增加接收的样本计数
                    std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                              << " RECEIVED." << std::endl;
                }
            }
        }

        HelloWorld hello_; // 用于存储接收到的数据

        std::atomic_int samples_; // 接收的样本计数

    }
    listener_; // 监听器实例

public:

    HelloWorldSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new HelloWorldPubSubType()) // 初始化类型支持
    {
    }

    virtual ~HelloWorldSubscriber()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_); // 删除数据读取器
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_); // 删除主题
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_); // 删除订阅者
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_); // 删除域参与者
    }

    // 初始化订阅者
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber"); // 设置域参与者名称
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

        // 创建订阅者
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        // 创建DataReader
        reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //!Run the Subscriber
    void run(
            uint32_t samples)
    {
        while (listener_.samples_ < samples) // 循环等待接收指定数量的消息
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 每 100 毫秒检查一次
        }
    }

};

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    uint32_t samples = 10;

    HelloWorldSubscriber* mysub = new HelloWorldSubscriber(); // 创建订阅者实例
    if (mysub->init()) // 初始化订阅者
    {
        mysub->run(samples); // 运行订阅者
    }

    delete mysub; // 删除订阅者实例
    return 0;
}
