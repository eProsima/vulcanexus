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

/**
* @file monitor.cpp
*
*/

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds_statistics_backend/listener/DomainListener.hpp>
#include <fastdds_statistics_backend/StatisticsBackend.hpp>
#include <fastdds_statistics_backend/types/EntityId.hpp>
#include <fastdds_statistics_backend/types/types.hpp>

using namespace eprosima::statistics_backend;
using namespace eprosima::fastdds::dds;

class Monitor
{
public:

    Monitor(
            uint32_t domain,
            uint32_t n_bins,
            uint32_t t_interval)
        : domain_(domain)
        , n_bins_(n_bins)
        , t_interval_(t_interval)
    {
    }

    ~Monitor()
    {
        StatisticsBackend::stop_monitor(monitor_id_);
    }

    bool init()
    {
        monitor_id_ = StatisticsBackend::init_monitor(domain_);
        if (!monitor_id_.is_valid())
        {
            std::cout << "Error creating monitor" << std::endl;
            return 1;
        }

        StatisticsBackend::set_physical_listener(&physical_listener_);

        return true;
    }

    void run()
    {
        stop_.store(false);
        std::cout << "Monitor running. Please press CTRL+C to stop the Monitor at any time." << std::endl;
        signal(SIGINT, [](int signum)
                {
                    std::cout << "\nSIGINT received, stopping Monitor execution." << std::endl;
                    static_cast<void>(signum);
                    Monitor::stop();
                });

        while (!stop_.load())
        {
            stop_.load();

            get_fastdds_latency_mean();
            get_publication_throughput_mean();
        }
    }

    static void stop()
    {
        stop_.store(true);
    }


    std::vector<StatisticsData> get_fastdds_latency_mean()
    {
        std::vector<StatisticsData> latency_data{};

        std::vector<EntityId> topics = StatisticsBackend::get_entities(EntityKind::TOPIC);
        EntityId chatter_topic_id = -1;
        Info topic_info;
        for (auto topic_id : topics)
        {
            topic_info = StatisticsBackend::get_info(topic_id);
            if (topic_info["name"] == "rt/chatter" && topic_info["data_type"] == "std_msgs::msg::dds_::String_")
            {
                chatter_topic_id = topic_id;
                break;
            }
        }

        if (chatter_topic_id < 0)
        {
            return latency_data;
        }

        std::vector<EntityId> topic_datawriters = StatisticsBackend::get_entities(
            EntityKind::DATAWRITER,
            chatter_topic_id);
        std::vector<EntityId> topic_datareaders = StatisticsBackend::get_entities(
            EntityKind::DATAREADER,
            chatter_topic_id);

        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

        latency_data = StatisticsBackend::get_data(
            DataKind::FASTDDS_LATENCY,                                   // DataKind
            topic_datawriters,                                           // Source entities
            topic_datareaders,                                           // Target entities
            n_bins_,                                                     // Number of bins
            now - std::chrono::seconds(t_interval_),                     // t_from
            now,                                                         // t_to
            StatisticKind::MEAN);                                        // Statistic

        for (auto latency : latency_data)
        {
            if (std::isnan(latency.second))
            {
                return latency_data;
            }

            std::int64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                latency.first.time_since_epoch()).count();
            fastdds_latency_mean_->Set(latency.second / 1000, timestamp_ms);

            std::cout << "ROS 2 Latency in topic " << topic_info["name"] << ": ["
                    << timestamp_to_string(latency.first) << ", " << latency.second / 1000 << " μs]" << std::endl;
        }

        return latency_data;
    }

    std::vector<StatisticsData> get_publication_throughput_mean()
    {
        std::vector<StatisticsData> publication_throughput_data{};

        std::vector<EntityId> topics = StatisticsBackend::get_entities(EntityKind::TOPIC);
        EntityId chatter_topic_id = -1;
        Info topic_info;
        for (auto topic_id : topics)
        {
            topic_info = StatisticsBackend::get_info(topic_id);
            if (topic_info["name"] == "rt/chatter" && topic_info["data_type"] == "std_msgs::msg::dds_::String_")
            {
                chatter_topic_id = topic_id;
                break;
            }
        }

        if (chatter_topic_id < 0)
        {
            return publication_throughput_data;
        }

        std::vector<EntityId> chatter_datawriters = StatisticsBackend::get_entities(
            EntityKind::DATAWRITER,
            chatter_topic_id);

        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

        publication_throughput_data = StatisticsBackend::get_data(
            DataKind::PUBLICATION_THROUGHPUT,                            // DataKind
            chatter_datawriters,                                         // Source entities
            n_bins_,                                                     // Number of bins
            now - std::chrono::seconds(t_interval_),                     // t_from
            now,                                                         // t_to
            StatisticKind::MEAN);                                        // Statistic

        for (auto publication_throughput : publication_throughput_data)
        {
            if (std::isnan(publication_throughput.second))
            {
                return publication_throughput_data;
            }

            std::int64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                publication_throughput.first.time_since_epoch()).count();
            publication_throughput_mean_->Set(publication_throughput.second, timestamp_ms);

            std::cout << "Publication throughput in topic " << topic_info["name"] << ": ["
                    << timestamp_to_string(publication_throughput.first) << ", "
                    << publication_throughput.second << " B/s]" << std::endl;
        }

        return publication_throughput_data;
    }

    std::string timestamp_to_string(
            const Timestamp timestamp)
    {
        auto timestamp_t = std::chrono::system_clock::to_time_t(timestamp);
        auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()).count();
        msec %= 1000;
        std::stringstream ss;

        ss << std::put_time(localtime(&timestamp_t), "%F %T") << "." << std::setw(3) << std::setfill('0') << msec;

        return ss.str();
    }

protected:

    class Listener : public eprosima::statistics_backend::PhysicalListener
    {
    public:

        Listener()
        {
        }

        ~Listener() override
        {
        }

        //! Callback when a new Host is discovered
        void on_host_discovery(
                EntityId host_id,
                const DomainListener::Status& status) override
        {
            Info host_info = StatisticsBackend::get_info(host_id);

            if (status.current_count_change == 1)
            {
                std::cout << "Host " << std::string(host_info["name"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "Host " << std::string(host_info["name"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new User is discovered
        void on_user_discovery(
                EntityId user_id,
                const DomainListener::Status& status) override
        {
            Info user_info = StatisticsBackend::get_info(user_id);

            if (status.current_count_change == 1)
            {
                std::cout << "User " << std::string(user_info["name"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "User " << std::string(user_info["name"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new Process is discovered
        void on_process_discovery(
                EntityId process_id,
                const DomainListener::Status& status) override
        {
            Info process_info = StatisticsBackend::get_info(process_id);

            if (status.current_count_change == 1)
            {
                std::cout << "Process " << std::string(process_info["name"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "Process " << std::string(process_info["name"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new Locator is discovered
        void on_locator_discovery(
                EntityId locator_id,
                const DomainListener::Status& status) override
        {
            Info locator_info = StatisticsBackend::get_info(locator_id);

            if (status.current_count_change == 1)
            {
                std::cout << "Locator " << std::string(locator_info["name"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "Locator " << std::string(locator_info["name"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new Topic is discovered
        void on_topic_discovery(
                EntityId domain_id,
                EntityId topic_id,
                const DomainListener::Status& status) override
        {
            Info topic_info = StatisticsBackend::get_info(topic_id);
            Info domain_info = StatisticsBackend::get_info(domain_id);

            if (status.current_count_change == 1)
            {
                std::cout << "Topic " << std::string(topic_info["name"])
                        << " [" << std::string(topic_info["data_type"])
                        << "] discovered." << std::endl;
            }
            else
            {
                std::cout << "Topic " << std::string(topic_info["name"])
                        << " [" << std::string(topic_info["data_type"])
                        << "] updated info." << std::endl;
            }
        }

        //! Callback when a new Participant is discovered
        void on_participant_discovery(
                EntityId domain_id,
                EntityId participant_id,
                const DomainListener::Status& status) override
        {
            static_cast<void>(domain_id);
            Info participant_info = StatisticsBackend::get_info(participant_id);

            if (status.current_count_change == 1)
            {
                std::cout << "Participant with GUID " << std::string(participant_info["guid"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "Participant with GUID " << std::string(participant_info["guid"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new DataReader is discovered
        void on_datareader_discovery(
                EntityId domain_id,
                EntityId datareader_id,
                const DomainListener::Status& status) override
        {
            static_cast<void>(domain_id);
            Info datareader_info = StatisticsBackend::get_info(datareader_id);

            if (status.current_count_change == 1)
            {
                std::cout << "DataReader with GUID " << std::string(datareader_info["guid"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "DataReader with GUID " << std::string(datareader_info["guid"]) << " update info." << std::endl;
            }
        }

        //! Callback when a new DataWriter is discovered
        void on_datawriter_discovery(
                EntityId domain_id,
                EntityId datawriter_id,
                const DomainListener::Status& status) override
        {
            static_cast<void>(domain_id);
            Info datawriter_info = StatisticsBackend::get_info(datawriter_id);

            if (status.current_count_change == 1)
            {
                std::cout << "DataWriter with GUID " << std::string(datawriter_info["guid"]) << " discovered." << std::endl;
            }
            else
            {
                std::cout << "DataWriter with GUID " << std::string(datawriter_info["guid"]) << " update info." << std::endl;
            }
        }

    }
    physical_listener_;

    DomainId domain_;

    uint32_t n_bins_;

    uint32_t t_interval_;

    EntityId monitor_id_;

    static std::atomic<bool> stop_;
};

int main(
        int argc,
        char** argv)
{
    int columns;

    columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 80;

    int domain = 0;
    int n_bins = 1;
    int t_interval = 5;


    Monitor monitor(static_cast<uint32_t>(domain), static_cast<uint32_t>(n_bins),
            static_cast<uint32_t>(t_interval));

    if (monitor.init())
    {
        monitor.run();
    }

    return 0;
}
