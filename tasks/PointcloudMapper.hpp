#ifndef SLAM3D_POINTCLOUDMAPPER_TASK_HPP
#define SLAM3D_POINTCLOUDMAPPER_TASK_HPP

#include <slam3d/PointcloudMapperBase.hpp>
#include <slam3d/core/Mapper.hpp>
#include <slam3d/sensor/pcl/PointCloudSensor.hpp>

#include <maps/grid/MLSMap.hpp>

#include <deque>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>

#include <thread>
#include <functional>
#include <future>

#include "GridConfiguration.hpp"

namespace slam3d
{	
	class RockOdometry;

	class PointcloudMapper : public PointcloudMapperBase
	{
	friend class PointcloudMapperBase;
	protected:


		class MappingThread {
		 public:
			struct MappingTask {
				MappingTask(){};
				MappingTask(std::function<void(VertexObjectList vertices)> function, VertexObjectList vertices ):function(function),vertices(vertices) {}
				std::function<void(VertexObjectList vertices)> function;
				VertexObjectList vertices;
			};

			MappingThread(size_t maxQueueSize = 1):maxQueueSize(maxQueueSize) {
				thread = std::thread([&](){
					bool run = true;
					while (run) {
						MappingTask task;
						bool has_task = false;
						{  // locking  scope to free mutex before end of function
							std::unique_lock<std::mutex> lock(this->queueMutex);
							// wait for job in queue arrives
							this->queueHasNewJob.wait(lock, [this](){
								// predicate to check if wait can be stopped
								// if the queue has a size or the thread should be stopped waiting ends
								return this->queue.size() || this->stopThread;
							});
							if (this->stopThread) {
								// stop the infinite while loop and proceed to let the thread stop
								run = false;
								return;
							}
							task = queue.front();
							queue.pop_front();
							has_task = true;
						}
						if (has_task) {
							// this is intentionally not run insire locking scope of the queue
							task.function(task.vertices);
						}
					}
				});
			}

			~MappingThread(){
				stop();
			}

			bool addTask(const MappingTask& task, const bool& replaceLatestIfFull = false) {
				std::lock_guard<std::mutex> lock(queueMutex);
				if (queue.size() >= maxQueueSize) {
					if (replaceLatestIfFull) {
						queue.pop_back();
					}else{
						return false;
					}
				}
				queue.push_back(task);
				queueHasNewJob.notify_one();
				return true;
			}

			void stop() {
				stopThread = true;
				queueHasNewJob.notify_all();
				thread.join();
			}

			private:
				std::deque<MappingTask> queue;
				std::mutex queueMutex;
				std::condition_variable queueHasNewJob;

				std::thread thread;
				std::atomic<bool> stopThread;
				size_t maxQueueSize;

		};


		// Operations
		virtual bool generate_cloud();
		virtual bool generate_map();
		virtual bool optimize();
		virtual bool force_add();
		virtual bool write_graph();
		virtual bool write_envire();
		virtual bool write_ply(const std::string& folder);
		
		// Callbacks
		virtual bool setLog_level(boost::int32_t value);
		virtual void scanTransformerCallback(const base::Time &ts, const base::samples::Pointcloud &scan_sample);
		void transformerCallback(const base::Time &time);

		// Internal methods
		PointCloud::Ptr buildPointcloud(const VertexObjectList& vertices);
		void sendPointcloud(const VertexObjectList& vertices);
		virtual void handleNewScan(const VertexObject& scan);
		virtual void addScanToMap(PointCloudMeasurement::Ptr scan, const Transform& pose);
		virtual void clearMap();
		virtual void rebuildMap(const VertexObjectList& vertices);
		virtual void sendMap();
		bool loadPLYMap(const std::string& path);
	
		// Members
		slam3d::Clock* mClock;
		slam3d::Logger* mLogger;
		slam3d::MeasurementStorage* mStorage;
		slam3d::Graph* mGraph;
		slam3d::Mapper* mMapper;
		slam3d::PointCloudSensor* mPclSensor;
		slam3d::Solver* mSolver;
		slam3d::Solver* mPatchSolver;
		RockOdometry* mOdometry;
		boost::shared_mutex mGraphMutex;
		boost::shared_mutex mMapMutex;
		
		MappingThread pointcloudThread;
		MappingThread mapThread;

		int mScansAdded;
		int mScansReceived; 
		bool mForceAdd;
		bool mStartPoseInitialized;

		// Parameters for creation of MLS
		maps::grid::MLSMapSloped mMultiLayerMap;
		GridConfiguration mGridConf;
		
		// Current state of transformations
		Eigen::Affine3d mCurrentDrift;
		base::Time mCurrentTime;

	public:
		PointcloudMapper(std::string const& name = "slam3d::PointcloudMapper");
		PointcloudMapper(std::string const& name, RTT::ExecutionEngine* engine);
		~PointcloudMapper();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif
