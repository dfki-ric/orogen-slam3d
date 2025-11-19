#pragma once

#include <thread>
#include <functional>
#include <future>
#include <memory>
#include <atomic>
#include <deque>


/**
 * @brief Thread that allows to add arbitrary function calls to a queue be executed by the thread
 * 
 * The function call needs to be wrapped into the ThreadedFunctionQueue::Function class
 * This allows to add any type of function with paramaters, even mixed calls, to the queue.
 * 
 * Here is an example implementation for calling a class function with paramaters:
 * WARNING: the parameters for the function need to be copied to the class
 * 
 * 		class MappingTask : public ThreadedFunctionQueue::Function {
		 public:
			MappingTask():Function(){};
			MappingTask(std::function<void(VertexObjectList vertices)> function, VertexObjectList vertices):Function(),function(function),vertices(vertices) {}
			void run() override {
				function(vertices);
			}
		 private:
			std::function<void(VertexObjectList vertices)> function;
			VertexObjectList vertices;
		};
 *
 * you can add your Function implementation by adding the shared_ptr directly (it is automatically casted)
  	std::shared_ptr<MappingTask> task = std::make_shared<MappingTask>(std::bind(&PointcloudMapper::sendPointcloud, this, std::placeholders::_1), vertices);
	mapThread.addFunctionCall(task, true);
 *
 * You can also keep your the shared_ptr<MappingTask> and use it for return values (to be set in the run() function)
 * isProcessed() will return true after the run() function was called anf is finished by the thread
 *
 */
class ThreadedFunctionQueue {
 public:

    /**
     * @brief base Function to be implemented for adding calls to the Thread
     */
    class Function {
     public:
        Function():finished(false){}

        virtual void run() = 0;

        bool isProcessed() {
            return finished;
        }

     protected:
        friend class ThreadedFunctionQueue;
        void setProcessed() {
            finished = true;
        }
     private:
        std::atomic<bool> finished;
    };

    /**
     * @brief Construct a new Threaded Function Queue object
     * 
     * @param maxQueueSize set the maximum queue size
     * 
     * default: just have one pending task in addition to the currently running task (when the function calculates some result, 
     * you want to have one additional update call with the latest data afer the running calculation (on older data) finishes).
     * Use replaceLatestIfFull = true for calling addFunctionCall to get this behavior
     */
    ThreadedFunctionQueue(size_t maxQueueSize = 1);
    
    ~ThreadedFunctionQueue();

    /**
     * @brief add a Function call to the queue to be called by the thread
     * 
     * @param function shared ptr to your child class of Function (will be auto-casted) shared_ptr<MyFunctionClass>
     * @param replaceLatestIfFull if true, the last entry will be replaced if queue is full
     * @return true if the Function was added to the queue
     * @return false if the Function was not added to the queue (queue full and no replace)
     */
    bool addFunctionCall(std::shared_ptr<Function> function, const bool& replaceLatestIfFull = false);

    /**
     * @brief stops the thread (you might not want to call this)
     */
    void stop();

 private:
    std::deque<std::shared_ptr<Function>> queue;
    std::mutex queueMutex;
    std::condition_variable queueHasNewJob;

    std::thread thread;
    std::atomic<bool> stopThread;
    size_t maxQueueSize;

};
