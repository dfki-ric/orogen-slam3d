/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PointcloudToLaserScan.hpp"

using namespace slam3d;

PointcloudToLaserScan::PointcloudToLaserScan(std::string const& name, TaskCore::TaskState initial_state)
    : PointcloudToLaserScanBase(name, initial_state)
{
}

PointcloudToLaserScan::~PointcloudToLaserScan()
{
}

void PointcloudToLaserScan::convertPointCloudToLaserScan(base::samples::Pointcloud& pointcloud,  std::vector<uint32_t>& range_data)
{
    // This function will convert the pointcloud to a laser scan.
    // It will iterate over the points in the pointcloud and add them to the scan
    // if they are within the specified height and range limits.
    // The angle of each point will be calculated based on its position in the pointcloud.

    // The LaserScan datatype and visualization in Rock uses uint32 and the data units are in millimeter(mm). That is why there are some variables names _mm which are in millimeters. 

    base::samples::Pointcloud cropped_cloud;

    std::uint32_t range_max_mm = static_cast<uint32_t>(range_max*1000); // convert max range value to mm

    LOG_DEBUG("All values are in (m); Max height: %f, Min height: %f, Max range: %f, Min range: %f", max_height, min_height, range_max, range_min);

    if((max_height < min_height)){
        LOG_ERROR("Max height is less than Min height");
        return;
    }
    
    else if(range_max < range_min){
        LOG_ERROR("Max range is less than min range");
        return;
    }

    else{

        // size for range_data = (max_angle - min_angle) / angle_increment
        int range_size = std::ceil((angle_max - angle_min)/angle_increment);
        range_data.clear();
        range_data.resize(range_size);

        //use max value for scans with no obstacles
        for (size_t i = 0; i < range_data.size(); ++i) {
            range_data[i] = range_max_mm; // TODO: check if inf is possible here
        }
        
        for(std::vector<base::Vector3d>::const_iterator it = pointcloud.points.begin(); it < pointcloud.points.end(); ++it)
        {
            // if height(z value) of the point is outside the [min_height, max_height] then skip the point
            if ( ((*it)[2]) > max_height || ((*it)[2]) <= min_height )
                continue;

            double sq_distance = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]); 
            // square computation is computationally costly so squared distance is compared with squared range
            if ( (sq_distance > range_max*range_max) || ( sq_distance < range_min*range_min) )
                continue;

            double current_angle = atan2( (*it)[1] ,(*it)[0] );
            double current_range = hypot( (*it)[1], (*it)[0] );
            int index = (current_angle - angle_min) / angle_increment;

            if(index < 0 || index >= range_data.size()){
                LOG_DEBUG("The index value %d is out of range for the scan_range array with size %d", (index, range_data.size()));
                LOG_ERROR("Index out of range");
            }
            else{

                std::uint32_t current_range_mm = static_cast<uint32_t>(current_range * 1000); // convert the current range from meter to millimeters

                // add the closest obstacle distance for a given angle 
                if(current_range_mm < range_data[index]){
                    range_data[index] = current_range_mm;
                }
                else{
                    LOG_DEBUG("Current range value %f mm is larger than existing range value %f mm at the angle %f rad", (current_range_mm, range_data[index], current_angle))
                    continue;
                }
            }
            cropped_cloud.points.push_back(*it);
        }
    }

    if (cropped_cloud.points.empty()) {
        LOG_DEBUG("No points in the pointcloud are within the specified height and range limits.");
        return; // no points to process
    }
    else{
        // Log the number of points in the cropped cloud
        LOG_DEBUG("Cropped point cloud has %d points within the specified height and range limits.", cropped_cloud.points.size());
        _debug_cropped_cloud.write(cropped_cloud);
    }

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PointcloudToLaserScan.hpp for more detailed
// documentation about them.
bool PointcloudToLaserScan::configureHook()
{
    if (! PointcloudToLaserScanBase::configureHook())
        return false;

    // set parameters
    max_height = _max_height.get();
    min_height = _min_height.get();
    range_max = _range_max.get();
    range_min = _range_min.get();

    angle_max = _angle_max.get();
    angle_min = _angle_min.get();

    angle_increment = _angle_increment.get();

    // allocate memory for the constructed scan
    constructed_scan_ptr = new base::samples::LaserScan();

    return true;
}
bool PointcloudToLaserScan::startHook()
{
    if (! PointcloudToLaserScanBase::startHook())
        return false;
    return true;
}
void PointcloudToLaserScan::updateHook()
{
    PointcloudToLaserScanBase::updateHook();

    if(_pointcloud.read(pointcloud, false) != RTT::NewData)
        return;

    // clear the scan
    if(constructed_scan_ptr == nullptr){
        constructed_scan_ptr = new base::samples::LaserScan();
    } else {
        constructed_scan_ptr->reset();
    }
    
    std::vector<uint32_t> range_data; // ranges of the scan, data to be added to the scan
    convertPointCloudToLaserScan(pointcloud, range_data);

    double speed = 30 * (angle_max - angle_min); // 30 scans per second so the unit is rad/sec

    constructed_scan_ptr->time = pointcloud.time;
    constructed_scan_ptr->start_angle = angle_min; // start at minimum angle value and scan anticlockwise
    constructed_scan_ptr->angular_resolution = angle_increment;
    constructed_scan_ptr->minRange = range_min*1000; // Convert range to millimeter
    constructed_scan_ptr->maxRange = range_max*1000; // Convert range to millimeter
    constructed_scan_ptr->speed = speed; // This is not actual sensor value, for visualiazation purposes
    constructed_scan_ptr->ranges = range_data;

    // write the constructed scan to the output port
    _scan.write(*constructed_scan_ptr);
   
}
void PointcloudToLaserScan::errorHook()
{
    PointcloudToLaserScanBase::errorHook();
}
void PointcloudToLaserScan::stopHook()
{
    PointcloudToLaserScanBase::stopHook();
}
void PointcloudToLaserScan::cleanupHook()
{
    PointcloudToLaserScanBase::cleanupHook();

    if (constructed_scan_ptr != nullptr) {
        delete constructed_scan_ptr;
        constructed_scan_ptr = nullptr;
    }

}
