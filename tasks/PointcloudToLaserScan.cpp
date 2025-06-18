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

    base::samples::Pointcloud cropped_cloud;
    std::vector<double> range_data_double; // temporary vector to hold the ranges

    LOG_DEBUG("Max height: %f, Min height: %f, Max range: %f, Min range: %f", max_height, min_height, range_max, range_min);

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

    range_data_double.clear();
    range_data_double.resize(range_size);

    //use max value for scans with no obstacles
    for (size_t i = 0; i < range_data_double.size(); ++i) {
        range_data_double[i] = range_max; // TODO: check if inf is possible here
    }
        for(std::vector<base::Vector3d>::const_iterator it = pointcloud.points.begin(); it < pointcloud.points.end(); ++it)
        {
            double sq_distance = ((*it)[0] * (*it)[0]) + ((*it)[1] * (*it)[1]) + ((*it)[2] * (*it)[2]); 
            // if height(z value) of the point is outside the [min_height, max_height] then skip the point
            if ( ((*it)[2]) > max_height || ((*it)[2]) <= min_height )
                continue;
            
            // square computation is computationally costly so squared distance is compared with squared range
            if ( (sq_distance > range_max*range_max) || ( sq_distance < range_min*range_min) )
                continue;
            cropped_cloud.points.push_back(*it);
        }
    }

    if (cropped_cloud.points.empty()) {
        LOG_DEBUG("No points in the pointcloud are within the specified height and range limits.");
        return; // no points to process
    }
    else{
        // Log the number of points in the cropped cloud
        LOG_DEBUG("Cropped cloud has %d points within the specified height and range limits.", cropped_cloud.points.size());
    }

    _debug_cropped_cloud.write(cropped_cloud);

    // Find the range of points using x,y of the point
    // move the following logic into the previous loop to save time
    for(std::vector<base::Vector3d>::const_iterator it = cropped_cloud.points.begin(); it < cropped_cloud.points.end(); ++it)
    {
        // theta = tan_inv(x/y) 
        double current_angle = atan2( (*it)[1] ,(*it)[0] );
        double current_range = sqrt( (*it)[1] * (*it)[1] + (*it)[0] * (*it)[0] );

        int index = (current_angle - angle_min) / angle_increment;

        if(current_range < range_data_double[index]){
            range_data_double[index] = current_range;
        }
        else
        {
            LOG_DEBUG("Current range %f is larger than existing range %f at the angle %f", (current_range, range_data_double[index], current_angle))
            continue;
        }
    }

    // Convert the ranges from meters to millimeters
    for (size_t i = 0; i < range_data_double.size(); ++i)
    {
        range_data[i] = static_cast<uint32_t>(range_data_double[i] * 1000); // convert to millimeters and store in range_data
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
    
    constructed_scan_ptr->time = pointcloud.time;
    constructed_scan_ptr->start_angle = angle_min; // start at 90 degrees and scan anticlockwise
    constructed_scan_ptr->angular_resolution = angle_increment;
    constructed_scan_ptr->minRange = range_min*1000;
    constructed_scan_ptr->maxRange = range_max*1000;
    constructed_scan_ptr->speed = 6.0; // TODO: this is not actual value, used for visualization purposes
    std::vector<uint32_t> range_data; // ranges of the scan, data to be added to the scan

    convertPointCloudToLaserScan(pointcloud, range_data);

    // validate the length of the range_data vector and 
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
}
