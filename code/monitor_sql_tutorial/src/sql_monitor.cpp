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
* @file sql_monitor.cpp
*
*/

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sqlite3.h>

#include <fastdds_statistics_backend/listener/DomainListener.hpp>
#include <fastdds_statistics_backend/StatisticsBackend.hpp>
#include <fastdds_statistics_backend/types/EntityId.hpp>
#include <fastdds_statistics_backend/types/types.hpp>
#include <fastdds_statistics_backend/exception/Exception.hpp>

using namespace eprosima::statistics_backend;

constexpr const char* DB_FILENAME = "vulcanexus_monitor.db";

constexpr uint32_t DOMAIN = 0;
constexpr uint32_t INTERVAL_MS = 5000;
constexpr const char* TOPIC_NAME = "rt/chatter";
constexpr const char* DATA_TYPE_NAME = "std_msgs::msg::dds_::String_";

class Monitor
{
public:

    Monitor()
        : monitor_id_(EntityId::invalid())
        , topic_id_(EntityId::invalid())
        , database_(nullptr)
    {
        // Initialize sqlite database
        open_or_create_database();
        if (!database_)
        {
            throw Error("Error initializing database.");
        }

        // Initialize Monitor
        monitor_id_ = StatisticsBackend::init_monitor(DOMAIN);
        if (!monitor_id_.is_valid())
        {
            throw Error("Error initializing monitor.");
        }

        std::cout << "Backend to SQL running." << std::endl;
    }

    ~Monitor()
    {
        StatisticsBackend::stop_monitor(monitor_id_);
        close_database();
    }

    void run()
    {
        while (true)
        {
            if (!topic_id_.is_valid())
            {
                // If topic still not exist, do nothing
                topic_id_ = get_topic_id(TOPIC_NAME, DATA_TYPE_NAME);
                std::cout << "Topic " << TOPIC_NAME << " does not exist yet." << std::endl;
            }
            else
            {
                // If it exist, store data in database
                store_to_db();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL_MS));
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

    // Get communications latency median
    StatisticsData get_latency_data()
    {
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
        * Get the median of the FASTDDS_LATENCY of the last time interval
        * between the Publishers and Subscriptions publishing in and subscribed to a given topic
        */
        auto data = StatisticsBackend::get_data(
            DataKind::FASTDDS_LATENCY,                      // DataKind
            publishers,                                     // Source entities
            subscriptions,                                  // Target entities
            1,                                              // Number of bins
            now - std::chrono::milliseconds(INTERVAL_MS),   // t_from
            now,                                            // t_to
            StatisticKind::MEDIAN);                         // Statistic

        // There is only one value, check it is not nan
        return data[0];
    }

    //! Get publication thougput mean
    StatisticsData get_publication_throughput_mean()
    {
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
        auto data = StatisticsBackend::get_data(
            DataKind::PUBLICATION_THROUGHPUT,               // DataKind
            publishers,                                     // Source entities
            1,                                              // Number of bins
            now - std::chrono::milliseconds(INTERVAL_MS),   // t_from
            now,                                            // t_to
            StatisticKind::MEAN);                           // Statistic

        // There is only one value, check it is not nan
        return data[0];
    }

    void open_or_create_database()
    {
        // Open database
        int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX | SQLITE_OPEN_SHAREDCACHE;
        if (sqlite3_open_v2(DB_FILENAME, &database_, flags, 0) != SQLITE_OK)
        {
            std::cerr << "Error opening or creating database." << std::endl;
            sqlite3_close(database_);
            database_ = nullptr;
            return;
        }

        // Create table if it does not exist
        std::cout << "Creating table with query: \"" << create_table_statement_ << "\"." << std::endl;
        if (sqlite3_exec(database_, create_table_statement_, 0, 0, 0) != SQLITE_OK)
        {
            std::cerr << "Error creating table." << std::endl;
            sqlite3_close(database_);
            database_ = nullptr;
            return;
        }
    }

    void close_database()
    {
        sqlite3_close(database_);
    }

    void store_to_db()
    {
        // Get data
        auto latency_data = get_latency_data();
        auto throughput_data = get_publication_throughput_mean();

        // Parse the query to add values. Use only latency timestamp as both would be almost the same
        sprintf(
            query_,
            insert_data_query_,
            std::chrono::duration_cast<std::chrono::milliseconds>(latency_data.first.time_since_epoch()).count(),
            (std::isnan(latency_data.second) ? 0 : latency_data.second),
            (std::isnan(throughput_data.second) ? 0 : throughput_data.second));

        std::cout << "Storing data in SQLite DataBase with query: \"" << query_ << "\"." << std::endl;

        // Insert in database (if fails, do nothing)
        if(sqlite3_exec(database_, query_, 0, 0, 0) != SQLITE_OK)
        {
            std::cerr << "An error ocurred inserting new data: " << sqlite3_errmsg(database_) << std::endl;
        }
    }

protected:

    static constexpr const char* create_table_statement_ =
        "CREATE TABLE IF NOT EXISTS data("
        "timestamp BIGINT,"
        "latency_median FLOAT,"
        "throughput_mean FLOAT,"
        "PRIMARY KEY(timestamp)"
        ") WITHOUT ROWID;";

    static constexpr const char* insert_data_query_ =
        "INSERT INTO data (timestamp,latency_median,throughput_mean) VALUES(%lu,%f,%f);";

    char query_[200];

    DomainId domain_;
    uint32_t t_interval_;

    EntityId monitor_id_;
    EntityId topic_id_;

    sqlite3* database_;
};

int main()
{
    // Create Monitor instance
    Monitor monitor;

    // Start Monitor
    monitor.run();

    return 0;
}
