#include "ThreadedFunctionQueue.hpp"

ThreadedFunctionQueue::ThreadedFunctionQueue(size_t maxQueueSize):maxQueueSize(maxQueueSize) {
    thread = std::thread([&](){
        bool run = true;
        while (run) {
            std::shared_ptr<Function> function;
            bool has_function = false;
            {  // locking  scope to free mutex before end of function
                std::unique_lock<std::mutex> lock(this->queueMutex);
                // wait for job in queue arrives
                this->queueHasNewJob.wait(lock, [this](){
                    // predicate to check if wait can be stopped
                    // if the queue has a size or the thread should be stopped waiting ends
                    // only runs when queueHasNewJob.notify_* is called
                    return this->queue.size() || this->stopThread;
                });
                if (this->stopThread) {
                    // stop the infinite while loop and proceed to let the thread stop
                    run = false;
                    return;
                }
                function = queue.front();
                queue.pop_front();
                has_function = true;
            }
            if (has_function) {
                // this is intentionally not run insire locking scope of the queue
                function->run();
                function->setProcessed();
            }
        }
    });
}

ThreadedFunctionQueue::~ThreadedFunctionQueue(){
    stop();
}

bool ThreadedFunctionQueue::addFunctionCall(std::shared_ptr<Function> function, const bool& replaceLatestIfFull) {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (queue.size() >= maxQueueSize) {  // should never be > but anyways...
        if (replaceLatestIfFull) {
            queue.pop_back();
        }else{
            return false;
        }
    }
    queue.push_back(function);
    queueHasNewJob.notify_one();
    return true;
}

void ThreadedFunctionQueue::stop() {
    stopThread = true;
    queueHasNewJob.notify_all();
    thread.join();
}
