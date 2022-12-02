// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <condition_variable>
#include <csignal>
#include <mutex>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/InstanceState.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/types/TypesBase.h>

#include "HelloWorldPubSubTypes.h"

class HelloWorldSubscriber : public eprosima::fastdds::dds::DataReaderListener
{
public:

    HelloWorldSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new dds2vulcanexus::idl::HelloWorldPubSubType())
    {
    }

    virtual ~HelloWorldSubscriber()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_);
        }
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    bool init()
    {
        auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
        participant_ = factory->create_participant(0, eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT);
        if (participant_ == nullptr)
        {
            return false;
        }

        participant_->register_type(type_);

        topic_ = participant_->create_topic("rt/HelloWorld",
                        type_.get_type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr)
        {
            return false;
        }

        subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
        if (subscriber_ == nullptr)
        {
            return false;
        }

        reader_ = subscriber_->create_datareader(topic_, eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT, this);
        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override
    {
        eprosima::fastdds::dds::SampleInfo info;
        if (reader->take_next_sample(&sample_, &info) == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
        {
            if (info.instance_state == eprosima::fastdds::dds::InstanceStateKind::ALIVE_INSTANCE_STATE)
            {
                std::cout << "Receiving: '" << sample_.message() << " " << sample_.index() << "'" << std::endl;
            }
        }
    }

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataReader* reader_;
    eprosima::fastdds::dds::TypeSupport type_;
    dds2vulcanexus::idl::HelloWorld sample_;
};

std::condition_variable cv;
std::mutex mtx;
std::atomic_bool running;

void signal_handler_callback(
        int signum)
{
    std::cout << std::endl << "Caught signal " << signum << "; closing down..." << std::endl;
    running.store(false);
    cv.notify_one();
}

int main()
{
    signal(SIGINT, signal_handler_callback);

    HelloWorldSubscriber subscriber;
    running.store(subscriber.init());

    std::unique_lock<std::mutex> lck(mtx);
    cv.wait(lck, [&]()
            {
                return !running.load();
            });
}
