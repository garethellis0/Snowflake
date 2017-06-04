//
// Created by sb on 25/03/17.
//

#include <ZedFilter.h>

// The constructor 
ZedFilter::ZedFilter(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/point_cloud/cloud_registered";
    int queue_size = 1;
    raw_image_subscriber = nh.subscribe(camera_image_topic_name, queue_size, &ZedFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    filtered_image_publisher = nh.advertise<PointCloudRGB>(filtered_image_topic_name, queue_size);

    // Get Parameters
    SB_getParam(private_nh, "base_frame", base_link_name, std::string("base_link"));

    // Start dynamic reconfigure stuff
    boost::recursive_mutex::scoped_lock dyn_reconf_lock(config_mutex);
    dyn_reconf_lock.unlock();
    f = boost::bind(&ZedFilter::dynamicReconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Obtain default filter value parameters
    zed_filter::ZedHSVFilterConfig config;
    server.getConfigDefault(config);
    server.updateConfig(config);

}

void ZedFilter::dynamicReconfigureCallback(zed_filter::ZedHSVFilterConfig &config, uint32_t level){
    filter_values.h_min = config.h_min;   
    filter_values.h_max = config.h_max;   
    filter_values.s_min = config.s_min;   
    filter_values.s_max = config.s_max;   
    filter_values.v_min = config.v_min;   
    filter_values.v_max = config.v_max;   
}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {

    filter = PointCloudFilter(filter_values);

    sensor_msgs::PointCloud2 transformed_input;
    SB_doTransform(*zed_camera_output, transformed_input, base_link_name);

    // Conversion to PCL datatype
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(transformed_input, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);

    // Filter Values
    PointCloudRGB::Ptr output_cloud(new PointCloudRGB);
    filter.filterCloud(point_cloud_RGB, output_cloud);
    //output_cloud->header.frame_id = "/zed_current_frame";
    // Publish output
    filtered_image_publisher.publish(output_cloud);
}
