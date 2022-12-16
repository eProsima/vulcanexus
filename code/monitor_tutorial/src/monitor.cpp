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
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fastdds_statistics_backend/listener/DomainListener.hpp>
#include <fastdds_statistics_backend/StatisticsBackend.hpp>
#include <fastdds_statistics_backend/types/EntityId.hpp>
#include <fastdds_statistics_backend/types/types.hpp>

using namespace eprosima::statistics_backend;

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
        , monitor_id_(EntityId::invalid())
        , topic_id_(EntityId::invalid())
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
        while (true)
        {
            if (!topic_id_.is_valid())
            {
                topic_id_ = get_topic_id("rt/chatter", "std_msgs::msg::dds_::String_");
            }
            else
            {
                get_fastdds_latency_mean();
                get_publication_throughput_mean();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(t_interval_*1000));
        }
    }

    //! Get the id of the topic searching by topic_name and data_type
    EntityId get_topic_id(
            std::string topic_name,
            std::string data_type)
    {
        // Get the list of all topics available
        std::vector<EntityId> topics = StatisticsBackend::get_entities(EntityKind::TOPIC);
        Info topic_info;
        // Iterate over all topic searching for the one with specified topic_name and data_type
        for (auto topic_id : topics)
        {
            topic_info = StatisticsBackend::get_info(topic_id);
            if (topic_info["name"] == topic_name && topic_info["data_type"] == data_type)
            {
                return topic_id;
            }
        }


        return EntityId::invalid();
    }

    // Get communications latency mean
    void get_fastdds_latency_mean()
    {
        // Vector of Statistics Data to store the latency data
        std::vector<StatisticsData> latency_data{};

        // Publishers on a specific topic
        std::vector<EntityId> publishers = StatisticsBackend::get_entities(
            EntityKind::DATAWRITER,
            topic_id_);

        // Subscriptions on a specific topic
        std::vector<EntityId> subscriptions = StatisticsBackend::get_entities(
            EntityKind::DATAREADER,
            topic_id_);

        // Get current time
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

        /*
        * Get the mean of the FASTDDS_LATENCY of the last time interval
        * between the Publishers and Subscriptions publishing in and subscribed to a given topic
        */
        latency_data = StatisticsBackend::get_data(
            DataKind::FASTDDS_LATENCY,                  // DataKind
            publishers,                                 // Source entities
            subscriptions,                              // Target entities
            n_bins_,                                    // Number of bins
            now - std::chrono::seconds(t_interval_),    // t_from
            now,                                        // t_to
            StatisticKind::MEAN);                       // Statistic

        // Iterate over the retrieve data
        for (auto latency : latency_data)
        {
            // Check if there are meningful values in retrieved vector
            if (std::isnan(latency.second))
            {
                return;
            }

            // Print the latency data
            std::cout << "ROS 2 Latency in topic " << StatisticsBackend::get_info(topic_id_)["name"] << ": ["
                    << timestamp_to_string(latency.first) << ", " << latency.second / 1000 << " μs]" << std::endl;
        }
    }

    //! Get publication thougput mean
    void get_publication_throughput_mean()
    {
        // Vector of Statistics Data to store the publication throughput data
        std::vector<StatisticsData> publication_throughput_data{};

        // Publishers on a specific topic
        std::vector<EntityId> publishers = StatisticsBackend::get_entities(
            EntityKind::DATAWRITER,
            topic_id_);

        // Get current time
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

        /*
        * Get the mean of the PUBLICATION_THROUGHPUT of the last time interval
        * of the Publishers publishing in a given topic
        */
        publication_throughput_data = StatisticsBackend::get_data(
            DataKind::PUBLICATION_THROUGHPUT,           // DataKind
            publishers,                                 // Source entities
            n_bins_,                                    // Number of bins
            now - std::chrono::seconds(t_interval_),    // t_from
            now,                                        // t_to
            StatisticKind::MEAN);                       // Statistic

        // Iterate over the retrieve data
        for (auto publication_throughput : publication_throughput_data)
        {
            // Check if there are meningful values in retrieved vector
            if (std::isnan(publication_throughput.second))
            {
                return;
            }

            // Print the publication througput data
            std::cout << "Publication throughput in topic " << StatisticsBackend::get_info(topic_id_)["name"] << ": ["
                    << timestamp_to_string(publication_throughput.first) << ", "
                    << publication_throughput.second << " B/s]" << std::endl;
        }
    }

    //! Convert timestamp to string
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
                std::cout << "Host " << std::string(host_info["name"]) << " updated info." << std::endl;
            }
        }

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
                std::cout << "User " << std::string(user_info["name"]) << " updated info." << std::endl;
            }
        }

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
                std::cout << "Process " << std::string(process_info["name"]) << " updated info." << std::endl;
            }
        }

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
                std::cout << "Participant with GUID " << std::string(participant_info["guid"]) << " updated info." << std::endl;
            }
        }

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
                std::cout << "DataReader with GUID " << std::string(datareader_info["guid"]) << " updated info." << std::endl;
            }
        }

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
                std::cout << "DataWriter with GUID " << std::string(datawriter_info["guid"]) << " updated info." << std::endl;
            }
        }

    } physical_listener_;

    DomainId domain_;
    uint32_t n_bins_;
    uint32_t t_interval_;

    EntityId monitor_id_;
    EntityId topic_id_;
};

int main()
{
    int domain = 0;
    int n_bins = 1;
    int t_interval = 5;

    Monitor monitor(domain, n_bins, t_interval);

    if (monitor.init())
    {
        monitor.run();
    }

    return 0;
}
