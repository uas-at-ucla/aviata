#ifndef MAVSDK_CALLBACK_MANAGER_HPP
#define MAVSDK_CALLBACK_MANAGER_HPP

#include <functional>
#include <map>
#include <mutex>
#include <memory>
#include <list>

class MavsdkCallbackManager
{
public:
    template<typename T>
    void subscribe_mavsdk_callback(const std::function<void(const std::function<void(T)>&)>& subscribe, std::function<void(T)> user_callback) {
        uint32_t this_id = id;
        subscribe([this, this_id, user_callback](T arg) {
            std::lock_guard<std::mutex> guard(mutex);
            mavsdk_callback_queue[this_id] = std::make_unique<std::function<void()>>([user_callback, arg]() {
                user_callback(arg);
            });
        });
        id++;
    }

    void call_queued_mavsdk_callbacks() {
        std::map<uint32_t, std::unique_ptr<std::function<void()>>> queued_callbacks;
        {
            std::lock_guard<std::mutex> guard(mutex);
            queued_callbacks.merge(mavsdk_callback_queue);
        }
        for (const auto& [id, callback] : queued_callbacks) {
            (*callback)();
        }
    }

private:
    static uint32_t id;
    std::map<uint32_t, std::unique_ptr<std::function<void()>>> mavsdk_callback_queue;
    std::mutex mutex;
};

#endif
