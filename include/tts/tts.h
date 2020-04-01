
#pragma once

#include "ros/ros.h"
#include "tts/TextToSpeechSrv.h"
#include "tts/TextToSpeechMsg.h"
#include "std_srvs/Trigger.h"

#include <string>

namespace tts
{
    constexpr auto SRV_SAY      = "/tts/say";
    constexpr auto SRV_ADD      = "/tts/add";
    constexpr auto SRV_REMOVE   = "/tts/remove";
    constexpr auto SRV_PRINT    = "/tts/print";
    constexpr auto QUEUE_SIZE   = 100;
    constexpr auto LOOP_FREQ    = 0.3;  // Hz
    constexpr auto INIT_TIMEOUT = 5;    // s

    inline ros::NodeHandle* m   = nullptr;

    bool        addToQueue(const std::string &data_str, int pri);                               // This method adds a new service to setSrv, and rebuilds the queue.
    bool        removeFromQueue(tts::TextToSpeechSrv::Request &srv);                            // Removes a service from setSrv and rebuilds the queue.
    bool        say(const std::string &str, int priority = 1);                                  // This method ensures that whatever string is given, is the next srv being said.
    void        printSrvs();                                                                    // Prints a table of the current services.
    inline bool callService(ros::ServiceClient &client, const std::string &data, int priority);
    bool        init();
} 

namespace tts
{

    bool init()
    {
        if (m == nullptr)
        {
            if (not ros::service::waitForService(SRV_SAY,ros::Duration(INIT_TIMEOUT)))
            {
                ROS_ERROR("Could not communicate with tts node, please ensure its running!");
                throw;
                return false;
            }
            m = new ros::NodeHandle;
        }
        return true;
    }

    inline bool callService(ros::ServiceClient &client, const std::string &data, int priority)
    {
        static tts::TextToSpeechSrv srv;

        srv.request.data = data;
        srv.request.priority = priority;

        return (client.call(srv) and srv.response.success);
    }

    void printSrvs()
    {
        if (not init())
            return;

        if (not ros::service::waitForService(SRV_PRINT))
            return;
        
        static ros::ServiceClient client = m->serviceClient<std_srvs::Trigger>(SRV_PRINT);

        std_srvs::Trigger srv;

        std::cout << std::boolalpha << client.call(srv) << std::endl;

        if (client.call(srv) and srv.response.success)
            std::cout << srv.response.message << std::endl;
    }

    bool addToQueue(const std::string &data, int priority)
    {
        if (not init())
            return false;

        if (not ros::service::waitForService(SRV_ADD))
            return false;

        static ros::ServiceClient client    = m->serviceClient<tts::TextToSpeechSrv>(SRV_ADD);

        return callService(client,data,priority); 
    }

    bool removeFromQueue(const std::string &data, int priority)
    {
        if (not init())
            return false;

        if (not ros::service::waitForService(SRV_REMOVE))
            return false;

        static ros::ServiceClient client = m->serviceClient<tts::TextToSpeechSrv>(SRV_REMOVE);

        return callService(client,data,priority);
    }

    bool say(const std::string &data, int priority)
    {
        if (not init())
            return false;

        if (not ros::service::waitForService(SRV_ADD))
            return false;
        
        static ros::ServiceClient client = m->serviceClient<tts::TextToSpeechSrv>(SRV_SAY);

        return callService(client,data,priority);
    }
} 


