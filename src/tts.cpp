

#include "ros/ros.h"
#include "ros/package.h"
#include "tts/TextToSpeechSrv.h"
#include "tts/TextToSpeechMsg.h"
#include "tts/tts.h"

#include <algorithm>
#include <list>
#include <sstream>
#include <set>
#include <mutex>
#include <tuple>

/*

 msg1.priority  = 1;
 msg2.prority   = 2;
 msg3.prority   = 3;
 msg4.prority   = 4;
 Queue: [1][2][3][4] [1][][][] [1][2][][] [1][3][][] [1][2][4][] [1][][][][] [1][2][3][] [1][][][] [1][2][4][] [1][3][][] [1][2][][] [1][][][]

*/


namespace tts
{
    // Attributes.
    std::list<tts::TextToSpeechSrv::Request>    queueSrv;                                                                                           // The queue containing waves weighted by priority.
    std::set<tts::TextToSpeechSrv::Request>     setSrv;                                                                                             // Contains one copy of every srv in queueSrv.
    int                                         queueSize = QUEUE_SIZE;                                                                             // The queue size is set in the launch file as a ROS parameter.
    std::mutex                                  mut_queueSrv;                                                                                       // Mutex protecting queueSrv. 
    std::mutex                                  mut_setSrv;                                                                                         // Mutex protecting setSrv.

    // Callbacks.
    bool                                        addToQueueCallback(tts::TextToSpeechSrv::Request &req, tts::TextToSpeechSrv::Response &res);        // This method adds a new service to setSrv, and rebuilds the queue.
    bool                                        removeFromQueueCallback(tts::TextToSpeechSrv::Request &srv);                                        // Removes a service from setSrv and rebuilds the queue.
    bool                                        sayCallback(tts::TextToSpeechSrv::Request &srv);                                                    // Removes a service from setSrv and rebuilds the queue.
    bool                                        printSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);                // Print the table of services.

    // Methods.
    bool                                        msgExsists(tts::TextToSpeechSrv::Request &srv);                                                     // Checks if a msg exsists in setSrv.
    void                                        buildQueue();                                                                                       // Builds the wave architecture in the queue.
    void                                        say(tts::TextToSpeechSrv::Request &srv);                                                            // Says whatever is in the data attribute.

    // Overloads for custom services.
    inline bool                                 operator==(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs);
    inline bool                                 operator!=(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs);
    inline bool                                 operator<(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs);
    inline bool                                 operator>(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs);
}

namespace tts
{

    // Callbacks ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool sayCallback(tts::TextToSpeechSrv::Request &req, tts::TextToSpeechSrv::Response &res)
    {
        say(req);        
        res.success = true;
        return true;
    }

    bool addToQueueCallback(tts::TextToSpeechSrv::Request &req, tts::TextToSpeechSrv::Response &res)
    {
        ROS_DEBUG("Adding an element to queue with priority [%ld]",req.priority);

        mut_setSrv.lock();
        if (auto [it, success] = setSrv.insert(req); not success)
        {
            ROS_WARN("Attempted to add a message to tts service set, which already exsists! Service overridden.");
        }
        mut_setSrv.unlock();

        mut_queueSrv.lock();
        queueSrv.push_back(req);
        mut_queueSrv.unlock();

        buildQueue();

        res.success = true;
        return true;
    }

    bool removeFromQueueCallback(tts::TextToSpeechSrv::Request &req, tts::TextToSpeechSrv::Response &res)
    {

        ROS_DEBUG("Removing an element from queue with priority [%ld]", req.priority);
        
        if (msgExsists(req))
        {
            
            mut_setSrv.lock();
            setSrv.erase(setSrv.find(req));
            mut_setSrv.unlock();

            ROS_DEBUG("Element with priority [%ld] has successfully been removed", req.priority);
            res.success = true;
            buildQueue();
            return true;
        }
        else
        {
            ROS_DEBUG("Element with priority [%ld] has not been removed since it does not exsist", req.priority);
            return false;
        }
    }

    bool printSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
    
        ROS_DEBUG("Printing table of services.");
        std::lock_guard lock(mut_setSrv);

        std::stringstream   ss;
        size_t              length = 7;

        ss << std::endl;
        
        // Calculate the message column width.
        for (const auto &s : setSrv)
            if (s.data.length() > length) { length = s.data.length(); }
        
        // Add table header
        ss << "| " << std::left << std::setfill(' ') << std::setw(7) << "Element | " << std::setw(length) << "Message" << " | " << std::setw(8) << "Priority" << " |" << std::endl;
        ss << "|" << std::string(9,'-') << "|" << std::string(length + 2,'-') << "|" << std::string(10,'-') << "|" << std::endl;
        size_t              i = 0;

        // Add table content
        for (const auto &s : setSrv)
        {
            ss << "| " << std::left << std::setfill(' ') << std::setw(7) << i << " | " << std::setw(length) << s.data << " | " << std::setw(8) << s.priority << " |" << std::endl;
            i++;
        }

        ss << std::endl;

        res.message = ss.str().c_str();

        res.success = not ss.str().empty();

        return true;
    }

    // Methods //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void say(tts::TextToSpeechSrv::Request &srv)
    {
        ROS_DEBUG("say: [%s]",srv.data.c_str());
        system(std::string("rosrun tts tts.py " + srv.data).c_str());
    }

    bool msgExsists(tts::TextToSpeechSrv::Request &srv)
    {
        std::lock_guard lock(mut_setSrv);
        return (setSrv.count(srv) != 0);
    }

    void printQueue()
    {        
        ROS_DEBUG("Printing queue of services.");
        for (auto s : queueSrv)
            std::cout << "data = " << s.data << " p = " << s.priority << std::endl;
    }

    void buildQueue()
    {
        std::lock_guard lock_queueSrv(mut_queueSrv);
        std::lock_guard lock_setSrv(mut_setSrv);

        queueSrv.clear();
        for (size_t x = 0; x < queueSize; x++)
        {
            for (const auto& s : setSrv)
            {
                // Check for root.
                if (1e-12 > abs(sin((std::pow(double(x), 2) * M_PI) / double(s.priority))))
                {
                    // Check for negative f(x).
                    if (0.0 > sin(2.0 * M_PI / double(s.priority) * (double(x) - 0.1)))
                    {
                        queueSrv.push_back(s);
                    }
                }
            }
        }
        queueSrv.resize(QUEUE_SIZE); // crop
        ROS_DEBUG("Built queue with [%i] elements",QUEUE_SIZE);
    }

    // Operator overloads ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    inline bool operator==(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs)
    {
        return ((lhs.data == rhs.data));
    }

    inline bool operator!=(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs)
    {
        return ((lhs.data != rhs.data));
    }

    inline bool operator<(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs)
    {
        return (lhs.priority < rhs.priority || lhs.priority == rhs.priority);
    }

    inline bool operator>(const tts::TextToSpeechSrv::Request &lhs, const tts::TextToSpeechSrv::Request &rhs)
    {
        return (lhs.priority > rhs.priority || lhs.priority == rhs.priority);
    }
}


int main(int argc, char **argv)
{
    // Initiate the service node.
    ros::init(argc, argv, "tts");
    ros::NodeHandle n;

    // State debugging lv.
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

    // Initiate services
    ros::ServiceServer srv_add      = n.advertiseService(tts::SRV_ADD, tts::addToQueueCallback);
    ros::ServiceServer srv_remove   = n.advertiseService(tts::SRV_REMOVE, tts::removeFromQueueCallback);
    ros::ServiceServer srv_say      = n.advertiseService(tts::SRV_SAY, tts::sayCallback);
    ros::ServiceServer srv_print    = n.advertiseService(tts::SRV_PRINT, tts::printSrvCallback);

    // async callbacks
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // rest of while loop here

    ros::Rate loop_rate(tts::LOOP_FREQ);

    size_t i = 0;
    while (ros::ok())
    {
        tts::mut_queueSrv.lock();
        if (not tts::queueSrv.empty())
        {
            auto it = std::next(tts::queueSrv.begin(), i);          // Declare iterator.
            tts::say(*it);                                          // Say the element.
            ROS_DEBUG("queueSrv size: [%lu]", tts::queueSrv.size());
        }
        tts::mut_queueSrv.unlock();

        i = (i == tts::queueSize - 1) ? 0 : i + 1;                  // Increment iterator.

        loop_rate.sleep();
    }

    // exit
    ros::waitForShutdown();

    return 0;
}
