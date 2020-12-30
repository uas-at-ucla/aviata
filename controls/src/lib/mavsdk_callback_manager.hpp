#ifndef MAVSDK_CALLBACK_MANAGER_HPP
#define MAVSDK_CALLBACK_MANAGER_HPP

#include <functional>
#include <map>
#include <mutex>

class MavsdkCallbackManager
{
public:
    template<typename T>
    void subscribe_mavsdk_callback(std::function<void(std::function<void(T)>)> subscribe, std::function<void(T)> user_callback) {
        uint32_t this_id = id;
        subscribe([this, this_id, user_callback](T arg) {
            std::lock_guard<std::mutex> guard(mutex);
            mavsdk_callback_queue[this_id] = [user_callback, arg]() {
                user_callback(arg);
            };
        });
        id++;
    }

    void call_queued_mavsdk_callbacks() {
        std::lock_guard<std::mutex> guard(mutex);
        auto iterator = mavsdk_callback_queue.begin();
        while (iterator != mavsdk_callback_queue.end()) {
            iterator->second();
            iterator = mavsdk_callback_queue.erase(iterator);
        }
    }

private:
    static uint32_t id;
    std::map<uint32_t, std::function<void()>> mavsdk_callback_queue;
    std::mutex mutex;
};

#endif
