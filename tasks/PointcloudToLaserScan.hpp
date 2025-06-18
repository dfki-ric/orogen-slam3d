/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SLAM3D_POINTCLOUDTOLASERSCAN_TASK_HPP
#define SLAM3D_POINTCLOUDTOLASERSCAN_TASK_HPP

#include "slam3d/PointcloudToLaserScanBase.hpp"
#include <base/samples/LaserScan.hpp>
#include <base/samples/Pointcloud.hpp>
#include <base-logging/Logging.hpp>

#include <cmath>
#include <vector>

namespace slam3d{

    /*! \class PointcloudToLaserScan
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','slam3d::PointcloudToLaserScan')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class PointcloudToLaserScan : public PointcloudToLaserScanBase
    {
	friend class PointcloudToLaserScanBase;
    protected:

        /** The maximum and minimum height in meters.
         * The points from the pointcloud that are outside of this range
         * will not be included in the scan.**/
        double max_height;
        double min_height;

        /** The maximum and minimum range of the scan in meters.
         * The range is the distance from the sensor to the object.
         * **/
        double range_max;
        double range_min;

        /** The minimum and maximum angle of the scan in radians.
         * angle 0 is at the front of the device, 
         * and the angle increases counter-clockwise when the z-axis points upwards.
         * **/
        double angle_min;
        double angle_max;

        /** The angle resolution of the scan in radians.**/
        double angle_increment;
    
        base::samples::LaserScan* constructed_scan_ptr;
        base::samples::Pointcloud pointcloud;

    public:
        /** TaskContext constructor for PointcloudToLaserScan
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PointcloudToLaserScan(std::string const& name = "slam3d::PointcloudToLaserScan", TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of PointcloudToLaserScan
         */
	~PointcloudToLaserScan();

        void convertPointCloudToLaserScan(base::samples::Pointcloud& pointcloud, std::vector<uint32_t>& range_data);

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

